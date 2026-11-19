// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "engine/HairLook.h"

#include "RE/B/BGSHeadPart.h"
#include "RE/B/BSModelDB.h"
#include "RE/T/TESNPC.h"
#include "RE/T/TESObjectARMA.h"
#include "RE/T/TESObjectARMO.h"
#include "Utils/ActorUtils.h"
#include "Utils/SkinVertexBlock.h"
#include "Utils/StringUtils.h"
#include "engine/NifProbe.h"
#include "smp/HairShape.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <optional>
#include <set>

namespace HairLook
{
	namespace
	{
		constexpr std::uint32_t kDynamicVertexStride = 16;
		constexpr std::size_t kMaxTriangles = 400000;
		constexpr int kDefaultBudgetMs = 3000;
		constexpr std::size_t kDefaultTop = 10;
		constexpr std::size_t kMaxTop = 500;
		constexpr int kFemale = 1;
		constexpr std::uint16_t kMaxSkinBones = 256;  ///< A skin instance of a model not yet attached to an actor does not report its bone count.
		constexpr float kMaxCoordinate = 100000.0f;   ///< Beyond this a vertex was read from the wrong layout, not placed by an artist.

		struct Shape
		{
			std::optional<SmpPhysics::HairDescriptor> descriptor;
			std::string reason;
			std::size_t triangles{};
		};

		std::mutex mutex;
		std::map<std::string, Shape> shapes;
		json lastResult = json::object();

		void Publish(json a_result)
		{
			const std::scoped_lock lock(mutex);
			lastResult = std::move(a_result);
		}

		void AddTriangles(const std::uint8_t* a_vertices, std::uint32_t a_stride, const std::uint16_t* a_indices, std::uint32_t a_triangleCount, std::uint32_t a_vertexCount,
			const RE::NiTransform* a_toRoot, std::vector<SmpPhysics::HairTriangle>& a_out)
		{
			if (!a_vertices || !a_indices || a_stride < sizeof(float) * 3)
				return;
			const auto point = [&](std::uint16_t a_index) -> SmpPhysics::Float3 {
				const auto* position = reinterpret_cast<const float*>(a_vertices + static_cast<std::size_t>(a_index) * a_stride);
				RE::NiPoint3 p{ position[0], position[1], position[2] };
				if (a_toRoot)
					p = *a_toRoot * p;
				return { p.x, p.y, p.z };
			};
			for (std::uint32_t triangle = 0; triangle < a_triangleCount && a_out.size() < kMaxTriangles; ++triangle) {
				const std::uint16_t a = a_indices[triangle * 3];
				const std::uint16_t b = a_indices[triangle * 3 + 1];
				const std::uint16_t c = a_indices[triangle * 3 + 2];
				if (a >= a_vertexCount || b >= a_vertexCount || c >= a_vertexCount)
					continue;
				const SmpPhysics::HairTriangle read{ point(a), point(b), point(c) };
				const bool sane = std::ranges::all_of(std::array{ read.a, read.b, read.c }, [](const SmpPhysics::Float3& a_point) {
					return std::ranges::all_of(a_point, [](float a_value) { return std::abs(a_value) < kMaxCoordinate; });
				});
				if (sane)
					a_out.push_back(read);
			}
		}

		RE::NiTransform ToRoot(const RE::NiAVObject* a_object)
		{
			RE::NiTransform transform;
			for (const RE::NiAVObject* object = a_object; object; object = object->parent)
				transform = object->local * transform;
			return transform;
		}

		/** @brief Physics files carry stand-in shapes, such as a ground plane, that are not hair. */
		bool IsCollisionShape(const char* a_name)
		{
			const std::string name = Util::ToLowerAscii(a_name ? a_name : "");
			return name.find("ground") != std::string::npos || name.find("collision") != std::string::npos;
		}

		void AddGeometry(RE::BSGeometry* a_geometry, std::vector<SmpPhysics::HairTriangle>& a_out, std::vector<SmpPhysics::HairTriangle>& a_plain)
		{
			if (IsCollisionShape(a_geometry->name.c_str()))
				return;
			const auto& runtime = a_geometry->GetGeometryRuntimeData();
			const RE::NiSkinPartition* partitions = runtime.skinInstance ? runtime.skinInstance->skinPartition.get() : nullptr;
			if (partitions && partitions->numPartitions) {
				const RE::NiSkinInstance* skin = runtime.skinInstance.get();
				const auto* dynamic = a_geometry->AsDynamicTriShape();
				const void* dynamicData = dynamic ? dynamic->GetDynamicTrishapeRuntimeData().dynamicData : nullptr;
				const std::uint32_t dynamicCount = dynamic ? dynamic->GetDynamicTrishapeRuntimeData().dataSize / kDynamicVertexStride : 0;
				std::map<std::uint16_t, RE::NiTransform> boneToRoot;
				const auto toRoot = [&](std::uint16_t a_bone) -> const RE::NiTransform& {
					const auto found = boneToRoot.find(a_bone);
					if (found != boneToRoot.end())
						return found->second;
					RE::NiTransform transform;
					if (a_bone < kMaxSkinBones && skin->skinData && skin->bones) {
						const RE::NiAVObject* bone = skin->bones[a_bone];
						transform = (bone ? ToRoot(bone) : RE::NiTransform{}) * skin->skinData->GetBoneDataSkinToBone(a_bone);
					}
					return boneToRoot.emplace(a_bone, transform).first->second;
				};
				for (std::uint32_t index = 0; index < partitions->numPartitions; ++index) {
					const auto& part = partitions->partitions[index];
					if (!part.buffData)
						continue;
					RE::BSGraphics::VertexDesc desc = part.vertexDesc;
					const bool hasPosition = desc.HasFlag(RE::BSGraphics::Vertex::VF_VERTEX);
					if (!hasPosition && (!dynamicData || !part.vertexMap))
						continue;
					const std::uint32_t stride = desc.GetSize();
					const bool skinned = desc.HasFlag(RE::BSGraphics::Vertex::VF_SKINNED) && part.bones && part.buffData->rawVertexData;
					const std::uint32_t skinOffset = desc.GetAttributeOffset(RE::BSGraphics::Vertex::VA_SKINNING);
					std::vector<float> positions(static_cast<std::size_t>(part.vertices) * 3, 0.0f);
					for (std::uint32_t vertex = 0; vertex < part.vertices; ++vertex) {
						const float* from = nullptr;
						if (hasPosition && part.buffData->rawVertexData)
							from = reinterpret_cast<const float*>(part.buffData->rawVertexData + static_cast<std::size_t>(vertex) * stride);
						else if (dynamicData && part.vertexMap[vertex] < dynamicCount)
							from = reinterpret_cast<const float*>(static_cast<const std::uint8_t*>(dynamicData) + static_cast<std::size_t>(part.vertexMap[vertex]) * kDynamicVertexStride);
						if (!from)
							continue;
						RE::NiPoint3 point{ from[0], from[1], from[2] };
						if (skinned && skin) {
							const Util::SkinVertexBlock block = Util::ReadSkinVertexBlock(part.buffData->rawVertexData + static_cast<std::size_t>(vertex) * stride + skinOffset);
							std::size_t strongest = 0;
							for (std::size_t slot = 1; slot < Util::kSkinInfluences; ++slot)
								strongest = block.weight[slot] > block.weight[strongest] ? slot : strongest;
							if (block.paletteSlot[strongest] < part.numBones)
								point = toRoot(part.bones[block.paletteSlot[strongest]]) * point;
						}
						positions[static_cast<std::size_t>(vertex) * 3] = point.x;
						positions[static_cast<std::size_t>(vertex) * 3 + 1] = point.y;
						positions[static_cast<std::size_t>(vertex) * 3 + 2] = point.z;
					}
					AddTriangles(reinterpret_cast<const std::uint8_t*>(positions.data()), sizeof(float) * 3, part.buffData->rawIndexData, part.triangles, part.vertices, nullptr, a_out);
				}
				return;
			}
			const auto* tri = a_geometry->AsTriShape();
			if (!tri || !runtime.rendererData)
				return;
			const std::uint32_t count = tri->GetTrishapeRuntimeData().triangleCount;
			const std::uint32_t vertices = tri->GetTrishapeRuntimeData().vertexCount;
			const RE::NiTransform toRoot = ToRoot(a_geometry);
			if (const auto* dynamic = a_geometry->AsDynamicTriShape()) {
				const auto& dynamicRuntime = dynamic->GetDynamicTrishapeRuntimeData();
				if (dynamicRuntime.dynamicData)
					AddTriangles(static_cast<const std::uint8_t*>(dynamicRuntime.dynamicData), kDynamicVertexStride, runtime.rendererData->rawIndexData, count, vertices, &toRoot, a_plain);
				return;
			}
			AddTriangles(runtime.rendererData->rawVertexData, RE::BSGraphics::VertexDesc(runtime.vertexDesc).GetSize(), runtime.rendererData->rawIndexData, count, vertices, &toRoot, a_plain);
		}

		/** @brief Loads a_model through the game's own model cache and keeps its shape; a failure is kept too, so it is not tried again. */
		const Shape& ShapeOf(const std::string& a_model)
		{
			const std::string key = Util::SearchKey(a_model);
			{
				const std::scoped_lock lock(mutex);
				if (const auto found = shapes.find(key); found != shapes.end())
					return found->second;
			}
			Shape shape;
			RE::NiPointer<RE::NiNode> root;
			RE::BSModelDB::DBTraits::ArgsType args;
			auto error = RE::BSModelDB::Demand(a_model.c_str(), root, args);
			if (error != RE::BSResource::ErrorCode::kNone || !root)
				shape.reason = "the game could not load the model";
			else {
				std::vector<SmpPhysics::HairTriangle> triangles;
				std::vector<SmpPhysics::HairTriangle> plain;
				Util::ForEachGeometry(root.get(), [&](RE::BSGeometry* a_geometry) { AddGeometry(a_geometry, triangles, plain); });
				if (triangles.empty())
					triangles = std::move(plain);
				shape.triangles = triangles.size();
				SmpPhysics::HairDescriptor descriptor = SmpPhysics::DescribeHair(triangles);
				if (descriptor.samples.empty())
					shape.reason = "the model has no triangles the plugin can read";
				else
					shape.descriptor = std::move(descriptor);
			}
			const std::scoped_lock lock(mutex);
			return shapes.emplace(key, std::move(shape)).first->second;
		}

		bool IsCached(const std::string& a_model)
		{
			const std::scoped_lock lock(mutex);
			return shapes.contains(Util::SearchKey(a_model));
		}

		std::string ModelOf(const RE::TESObjectARMO& a_armor, int a_sex)
		{
			for (const RE::TESObjectARMA* addon : a_armor.armorAddons) {
				if (!addon)
					continue;
				if (const char* path = addon->bipedModels[a_sex].GetModel(); path && *path)
					return path;
			}
			return {};
		}

		bool IsHairArmor(const RE::TESObjectARMO& a_armor)
		{
			return a_armor.HasPartOf(RE::BGSBipedObjectForm::BipedObjectSlot::kHair) || a_armor.HasPartOf(RE::BGSBipedObjectForm::BipedObjectSlot::kLongHair);
		}

		json SummaryOf(const SmpPhysics::HairDescriptor& a_descriptor, std::size_t a_triangles)
		{
			return { { "triangles", a_triangles }, { "area", a_descriptor.area }, { "boundsMin", a_descriptor.boundsMin }, { "boundsMax", a_descriptor.boundsMax } };
		}
	}

	void Rank(const json& a_args)
	{
		const auto begin = std::chrono::steady_clock::now();
		const auto elapsedMs = [&begin] { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count(); };
		const double budget = a_args.value("budgetMs", kDefaultBudgetMs);
		const std::size_t top = std::min<std::size_t>(a_args.value("top", kDefaultTop), kMaxTop);
		const std::string filter = Util::SearchKey(a_args.value("filter", std::string{}));
		const std::string name = Util::SearchKey(a_args.value("name", std::string{}));
		const bool physicsOnly = a_args.value("physicsOnly", NifProbe::Generation() != 0);
		int sex = a_args.value("sex", kFemale);

		auto* handler = RE::TESDataHandler::GetSingleton();
		if (!handler) {
			Publish({ { "error", "the game's data is not loaded" } });
			return;
		}

		std::string sourceModel = a_args.value("model", std::string{});
		std::string sourceName = sourceModel;
		RE::FormID sourceArmor = 0;
		if (sourceModel.empty() && a_args.contains("actorFormID")) {
			const auto* actor = RE::TESForm::LookupByID<RE::Actor>(a_args.value("actorFormID", 0u));
			auto* base = actor ? const_cast<RE::TESNPC*>(actor->GetActorBase()) : nullptr;
			RE::BGSHeadPart* hair = base ? base->GetCurrentHeadPartByType(RE::BGSHeadPart::HeadPartType::kHair) : nullptr;
			if (!hair) {
				Publish({ { "error", "that actor has no hair head part" } });
				return;
			}
			sex = base->GetSex() == RE::SEX::kFemale ? 1 : 0;
			sourceModel = hair->GetModel() ? hair->GetModel() : "";
			sourceName = std::string(hair->GetFormEditorID());
		} else if (sourceModel.empty() && a_args.contains("headPart")) {
			const auto* part = RE::TESForm::LookupByEditorID<RE::BGSHeadPart>(a_args.value("headPart", std::string{}));
			if (!part || !part->GetModel()) {
				Publish({ { "error", "no head part with that editor ID has a model" } });
				return;
			}
			sourceModel = part->GetModel();
			sourceName = std::string(part->GetFormEditorID());
		} else if (sourceModel.empty() && !name.empty()) {
			for (const RE::TESObjectARMO* armor : handler->GetFormArray<RE::TESObjectARMO>()) {
				if (armor && IsHairArmor(*armor) && Util::SearchKey(armor->GetName()).find(name) != std::string::npos && !ModelOf(*armor, sex).empty()) {
					sourceModel = ModelOf(*armor, sex);
					sourceName = armor->GetName();
					sourceArmor = armor->GetFormID();
					break;
				}
			}
		}
		if (sourceModel.empty()) {
			Publish({ { "error", "give actorFormID, model or name to pick the hair to match" } });
			return;
		}
		const Shape& source = ShapeOf(sourceModel);
		if (!source.descriptor) {
			Publish({ { "error", "the target hair " + source.reason }, { "model", sourceModel } });
			return;
		}

		struct Candidate
		{
			const RE::TESObjectARMO* armor;
			std::string model;
			SmpPhysics::HairMatch match;
		};
		std::vector<Candidate> ranked;
		json failed = json::array();
		std::size_t pending = 0;
		std::size_t considered = 0;
		std::set<std::string> seen;
		for (const RE::TESObjectARMO* armor : handler->GetFormArray<RE::TESObjectARMO>()) {
			if (!armor || armor->GetFormID() == sourceArmor || !IsHairArmor(*armor))
				continue;
			if (!filter.empty() && Util::SearchKey(armor->GetName()).find(filter) == std::string::npos)
				continue;
			if (physicsOnly && NifProbe::XmlPaths(armor->GetFormID()).empty())
				continue;
			std::string model = ModelOf(*armor, sex);
			if (model.empty() || !seen.insert(Util::SearchKey(model)).second)
				continue;
			++considered;
			if (!IsCached(model) && elapsedMs() > budget) {
				++pending;
				continue;
			}
			const Shape& shape = ShapeOf(model);
			if (!shape.descriptor) {
				failed.push_back({ { "name", armor->GetName() }, { "model", model }, { "reason", shape.reason } });
				continue;
			}
			ranked.push_back({ armor, std::move(model), SmpPhysics::CompareHair(*source.descriptor, *shape.descriptor) });
		}
		std::ranges::sort(ranked, [](const Candidate& a_left, const Candidate& a_right) { return a_left.match.chamfer < a_right.match.chamfer; });

		json rows = json::array();
		for (std::size_t index = 0; index < ranked.size() && index < top; ++index) {
			const Candidate& candidate = ranked[index];
			rows.push_back({ { "name", candidate.armor->GetName() }, { "formId", candidate.armor->GetFormID() }, { "model", candidate.model },
				{ "chamfer", candidate.match.chamfer }, { "reach", candidate.match.reach }, { "coverage", candidate.match.coverage }, { "overlap", candidate.match.overlap },
				{ "boxOverlap", candidate.match.boxOverlap }, { "areaRatio", candidate.match.areaRatio },
				{ "files", NifProbe::XmlPaths(candidate.armor->GetFormID()) } });
		}
		Publish({ { "target", { { "name", sourceName }, { "model", sourceModel }, { "shape", SummaryOf(*source.descriptor, source.triangles) } } },
			{ "considered", considered }, { "compared", ranked.size() }, { "pending", pending }, { "failed", std::move(failed) }, { "elapsedMs", elapsedMs() },
			{ "matches", std::move(rows) } });
	}

	json Result()
	{
		const std::scoped_lock lock(mutex);
		return lastResult;
	}
}
