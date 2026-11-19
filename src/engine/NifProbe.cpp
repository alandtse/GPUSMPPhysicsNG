// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "engine/NifProbe.h"

#include "RE/B/BGSHeadPart.h"
#include "RE/B/BSResourceNiBinaryStream.h"
#include "RE/T/TESObjectARMA.h"
#include "RE/T/TESObjectARMO.h"
#include "Utils/StringUtils.h"
#include "Utils/TestBench/TestCrowd.h"
#include "smp/NifPhysicsScan.h"
#include "smp/XmlAudit.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace NifProbe
{
	namespace
	{
		constexpr std::uint32_t kHeaderBytes = 65536;
		constexpr std::uint32_t kChunk = 4096;

		struct Target
		{
			std::string armor;
			RE::FormID formID{};
			std::vector<std::string> models;
		};

		struct Hit
		{
			std::string armor;
			RE::FormID formID{};
			std::string model;
			std::vector<std::string> xml;
		};

		std::mutex mutex;
		/** @brief A thread that is joined when it is destroyed, so a finished scan does not call std::terminate when the DLL unloads. */
		struct JoiningThread
		{
			std::thread thread;
			~JoiningThread()
			{
				if (thread.joinable())
					thread.join();
			}
		};

		JoiningThread worker;
		std::atomic<bool> running{ false };
		std::atomic<std::size_t> done{ 0 };
		std::size_t total = 0;
		std::size_t armors = 0;
		std::size_t missing = 0;
		std::size_t bytesRead = 0;
		double milliseconds = 0.0;
		std::vector<Hit> hits;
		SmpPhysics::XmlAudit audit;
		std::size_t xmlOnDisk = 0;
		std::atomic<std::uint32_t> generation{ 0 };

		std::string ReadHeader(const std::string& a_model, bool& a_found)
		{
			RE::BSResourceNiBinaryStream stream("Meshes\\" + a_model);
			a_found = stream.good();
			std::string bytes;
			if (!a_found)
				return bytes;
			std::vector<char> buffer(kChunk);
			while (bytes.size() < kHeaderBytes) {
				const std::uint32_t before = stream.tell();
				const bool whole = stream.read(buffer.data(), kChunk);
				const std::uint32_t got = std::min<std::uint32_t>(stream.tell() - before, kChunk);
				bytes.append(buffer.data(), got);
				if (!whole || got == 0)
					break;
			}
			return bytes;
		}

		constexpr const char* kDefaultMap = "Data/SKSE/Plugins/hdtSkinnedMeshConfigs/defaultBBPs.xml";
		constexpr std::size_t kMaxReadBytes = 1u << 20;
		constexpr std::size_t kSystemSniffBytes = 2048;  ///< The "<system" tag of a physics file sits at its start.
		constexpr std::size_t kDataPrefixLength = 5;     ///< Length of "Data/", which the returned paths drop.

		std::string ReadWhole(const std::filesystem::path& a_path, std::size_t a_limit = kMaxReadBytes)
		{
			std::ifstream file(a_path, std::ios::binary);
			std::string text(a_limit, '\0');
			text.resize(static_cast<std::size_t>(file.read(text.data(), static_cast<std::streamsize>(a_limit)).gcount()));
			return text;
		}

		/** @brief Loose physics xml files under the config folder and meshes, as Data-relative paths; files packed in archives are not listed. */
		std::vector<std::string> ListPhysicsXml()
		{
			namespace fs = std::filesystem;
			std::vector<std::string> found;
			std::error_code error;
			for (const char* root : { "Data/SKSE/Plugins/hdtSkinnedMeshConfigs", "Data/Meshes" }) {
				for (fs::recursive_directory_iterator entry(root, fs::directory_options::skip_permission_denied, error), end; !error && entry != end; entry.increment(error)) {
					const fs::path& path = entry->path();
					if (path.extension() != ".xml" && path.extension() != ".XML")
						continue;
					if (ReadWhole(path, kSystemSniffBytes).find("<system") == std::string::npos)
						continue;
					found.push_back(Util::PathUtf8(path).substr(kDataPrefixLength));
				}
				error.clear();
			}
			return found;
		}

		void Run(std::vector<Target> a_targets)
		{
			const auto begin = std::chrono::steady_clock::now();
			std::map<std::string, SmpPhysics::NifPhysicsHint> seen;
			std::size_t missed = 0;
			std::size_t read = 0;
			std::vector<Hit> found;
			std::size_t checked = 0;
			for (const Target& target : a_targets) {
				for (const std::string& model : target.models) {
					auto known = seen.find(model);
					if (known == seen.end()) {
						bool exists = false;
						const std::string bytes = ReadHeader(model, exists);
						++checked;
						SmpPhysics::NifPhysicsHint hint;
						if (exists) {
							read += bytes.size();
							hint = SmpPhysics::ScanNifHeader(bytes);
						} else {
							++missed;
						}
						known = seen.emplace(model, std::move(hint)).first;
					}
					if (known->second.hasExtraData)
						found.push_back({ target.armor, target.formID, model, known->second.xmlPaths });
				}
				done.fetch_add(1, std::memory_order_relaxed);
			}
			const std::vector<std::string> onDisk = ListPhysicsXml();
			std::vector<std::string> named = SmpPhysics::ParseDefaultBbpFiles(ReadWhole(kDefaultMap));
			for (const Hit& hit : found)
				named.insert(named.end(), hit.xml.begin(), hit.xml.end());
			SmpPhysics::XmlAudit newAudit = SmpPhysics::AuditXmlFiles(onDisk, named);
			const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
			const std::scoped_lock lock(mutex);
			total = checked;
			missing = missed;
			bytesRead = read;
			milliseconds = elapsed;
			hits = std::move(found);
			audit = std::move(newAudit);
			xmlOnDisk = onDisk.size();
			generation.fetch_add(1, std::memory_order_release);
			running.store(false, std::memory_order_release);
		}
	}

	void Start()
	{
		if (running.load(std::memory_order_acquire))
			return;
		if (worker.thread.joinable())
			worker.thread.join();
		std::vector<Target> targets;
		std::set<RE::FormID> listed;
		const auto addArmor = [&](const RE::TESObjectARMO* a_armor) {
			if (!a_armor || !listed.insert(a_armor->GetFormID()).second)
				return;
			Target target{ a_armor->GetName(), a_armor->GetFormID(), {} };
			for (const RE::TESObjectARMA* addon : a_armor->armorAddons) {
				if (!addon)
					continue;
				for (const auto& model : addon->bipedModels) {
					if (const char* path = model.GetModel(); path && *path)
						target.models.emplace_back(path);
				}
			}
			targets.push_back(std::move(target));
		};
		for (const auto& list : { TestBench::TestCrowd::ArmorForms(), TestBench::TestCrowd::CloakForms() }) {
			for (const RE::TESObjectARMO* armor : list)
				addArmor(armor);
		}
		// Every other armor and every head part is read too, so an xml they use is not reported as unused.
		if (auto* handler = RE::TESDataHandler::GetSingleton()) {
			for (const RE::TESObjectARMO* armor : handler->GetFormArray<RE::TESObjectARMO>())
				addArmor(armor);
			for (const RE::BGSHeadPart* part : handler->GetFormArray<RE::BGSHeadPart>()) {
				if (const char* path = part ? part->GetModel() : nullptr; path && *path)
					targets.push_back({ part->GetFormEditorID(), part->GetFormID(), { path } });
			}
		}
		{
			const std::scoped_lock lock(mutex);
			armors = targets.size();
			total = missing = bytesRead = 0;
			milliseconds = 0.0;
			hits.clear();
			audit = {};
			xmlOnDisk = 0;
		}
		done.store(0, std::memory_order_relaxed);
		running.store(true, std::memory_order_release);
		worker.thread = std::thread([scan = std::move(targets)]() mutable {
			try {
				Run(std::move(scan));
			} catch (const std::exception& error) {
				logger::error("[GPU SMP Physics] The physics scan stopped: {}", error.what());
				running.store(false, std::memory_order_release);
			}
		});
	}

	bool Running()
	{
		return running.load(std::memory_order_acquire);
	}

	std::uint32_t Generation()
	{
		return generation.load(std::memory_order_acquire);
	}

	std::pair<std::size_t, std::size_t> Progress()
	{
		const std::scoped_lock lock(mutex);
		return { done.load(std::memory_order_relaxed), armors };
	}

	std::vector<std::string> XmlPaths(RE::FormID a_armor)
	{
		std::vector<std::string> paths;
		const std::scoped_lock lock(mutex);
		for (const Hit& hit : hits) {
			if (hit.formID != a_armor)
				continue;
			for (const std::string& path : hit.xml) {
				if (std::ranges::find(paths, path) == paths.end())
					paths.push_back(path);
			}
		}
		return paths;
	}

	std::vector<Match> Find(std::string_view a_needle)
	{
		std::vector<Match> matches;
		const std::scoped_lock lock(mutex);
		for (const Hit& hit : hits) {
			std::vector<std::string> files;
			for (const std::string& path : hit.xml) {
				if (SmpPhysics::XmlPathMatches(path, a_needle))
					files.push_back(path);
			}
			if (!files.empty())
				matches.push_back({ hit.armor, hit.formID, hit.model, std::move(files) });
		}
		return matches;
	}

	json XmlAuditResult()
	{
		const std::scoped_lock lock(mutex);
		return { { "xmlOnDisk", xmlOnDisk }, { "orphans", audit.orphans }, { "dangling", audit.dangling } };
	}

	json Result()
	{
		const std::scoped_lock lock(mutex);
		json rows = json::array();
		for (const Hit& hit : hits)
			rows.push_back({ { "armor", hit.armor }, { "formId", hit.formID }, { "model", hit.model }, { "xml", hit.xml } });
		const bool finished = !running.load(std::memory_order_acquire);
		return { { "running", !finished }, { "armorsDone", done.load(std::memory_order_relaxed) }, { "armors", armors }, { "modelsRead", total },
			{ "modelsMissing", missing }, { "bytesRead", bytesRead }, { "elapsedMs", milliseconds },
			{ "msPerModel", total ? milliseconds / static_cast<double>(total) : 0.0 }, { "withPhysicsExtraData", hits.size() }, { "hits", std::move(rows) } };
	}
}
