#include "ClaimTally.h"

#include <algorithm>
#include <cctype>

namespace SmpPhysics
{
	namespace
	{
		std::string Lower(std::string_view a_text)
		{
			std::string lower(a_text);
			std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char a_c) { return static_cast<char>(std::tolower(a_c)); });
			return lower;
		}

		constexpr const char* kOtherReason = "<other reasons>";
	}

	std::string ClaimTally::Normalize(std::string_view a_reason)
	{
		std::string out;
		out.reserve(a_reason.size());
		for (std::size_t i = 0; i < a_reason.size(); ++i) {
			const char c = a_reason[i];
			if (c == '\'') {
				const std::size_t close = a_reason.find('\'', i + 1);
				if (close != std::string_view::npos) {
					out += "'?'";
					i = close;
					continue;
				}
			}
			if (std::isdigit(static_cast<unsigned char>(c))) {
				while (i + 1 < a_reason.size() && std::isdigit(static_cast<unsigned char>(a_reason[i + 1])))
					++i;
				out += 'N';
				continue;
			}
			out += c;
		}
		return out;
	}

	void ClaimTally::Record(Kind a_kind, std::uint32_t a_actor, std::string_view a_file, bool a_accepted, std::string_view a_reason)
	{
		const std::scoped_lock lock(mutex);
		KindCount& count = kinds[static_cast<std::size_t>(a_kind)];
		++count.attempts;
		const std::string file = Lower(a_file);

		const auto fileEntry = files.find(file);
		FileRow* fileRow = nullptr;
		if (fileEntry != files.end())
			fileRow = &fileEntry->second;
		else if (files.size() < kMaxFiles)
			fileRow = &files.emplace(file, FileRow{ file }).first->second;
		if (fileRow)
			++fileRow->attempts;

		std::string reasonKey;
		if (a_accepted) {
			++count.accepts;
			if (fileRow)
				++fileRow->accepts;
		} else {
			++count.declines;
			if (fileRow)
				++fileRow->declines;
			reasonKey = Normalize(a_reason);
			if (!reasons.contains(reasonKey) && reasons.size() >= kMaxReasons)
				reasonKey = kOtherReason;
			ReasonData& data = reasons[reasonKey];
			if (data.example.empty())
				data.example = std::string(a_reason);
			++data.declines;
			++data.byKind[static_cast<std::size_t>(a_kind)];
			if (data.sampleFiles.size() < kSampleFiles && std::find(data.sampleFiles.begin(), data.sampleFiles.end(), file) == data.sampleFiles.end())
				data.sampleFiles.push_back(file);
		}

		const auto key = std::make_pair(a_actor, file);
		const auto pair = pairs.find(key);
		if (pair != pairs.end()) {
			pair->second = { a_accepted, std::move(reasonKey) };
		} else if (pairs.size() < kMaxPairs) {
			pairs.emplace(key, PairState{ a_accepted, std::move(reasonKey) });
		} else {
			pairsTruncated = true;
		}
	}

	ClaimTally::Snapshot ClaimTally::Take() const
	{
		const std::scoped_lock lock(mutex);
		Snapshot snapshot;
		snapshot.kinds = kinds;
		snapshot.pairsTruncated = pairsTruncated;

		std::map<std::string, std::size_t> pairsByReason;
		for (const auto& entry : pairs) {
			if (entry.second.accepted)
				++snapshot.pairsAccepted;
			else {
				++snapshot.pairsDeclined;
				++pairsByReason[entry.second.reason];
			}
		}

		for (const auto& entry : reasons) {
			ReasonRow row;
			row.reason = entry.first;
			row.example = entry.second.example;
			row.declines = entry.second.declines;
			row.byKind = entry.second.byKind;
			row.sampleFiles = entry.second.sampleFiles;
			if (const auto found = pairsByReason.find(entry.first); found != pairsByReason.end())
				row.pairs = found->second;
			snapshot.reasons.push_back(std::move(row));
		}
		std::sort(snapshot.reasons.begin(), snapshot.reasons.end(), [](const ReasonRow& a_left, const ReasonRow& a_right) {
			return a_left.pairs != a_right.pairs ? a_left.pairs > a_right.pairs : a_left.declines > a_right.declines;
		});

		for (const auto& entry : files)
			snapshot.files.push_back(entry.second);
		std::sort(snapshot.files.begin(), snapshot.files.end(), [](const FileRow& a_left, const FileRow& a_right) {
			return a_left.declines != a_right.declines ? a_left.declines > a_right.declines : a_left.file < a_right.file;
		});
		return snapshot;
	}

	void ClaimTally::Reset()
	{
		const std::scoped_lock lock(mutex);
		kinds = {};
		reasons.clear();
		files.clear();
		pairs.clear();
		pairsTruncated = false;
	}
}
