#include "smp/RepairLog.h"

#include "Utils/StringUtils.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <format>

namespace SmpPhysics
{
	void RepairLog::Record(std::string_view a_file, std::string_view a_problem, std::string_view a_action)
	{
		if (a_file.empty() || a_problem.empty())
			return;
		const std::string key = Util::ToLowerAscii(a_file);
		const std::scoped_lock lock(mutex);
		auto report = std::ranges::find_if(reports, [&key](const FileReport& a_report) { return Util::ToLowerAscii(a_report.file) == key; });
		if (report == reports.end()) {
			if (reports.size() >= kMaxFiles)
				return;
			report = reports.insert(reports.end(), FileReport{ std::string(a_file), {} });
		}
		if (report->entries.size() >= kMaxEntriesPerFile)
			return;
		const bool known = std::ranges::any_of(report->entries, [&](const Entry& a_entry) { return a_entry.problem == a_problem; });
		if (!known)
			report->entries.push_back({ std::string(a_problem), std::string(a_action) });
	}

	std::vector<RepairLog::FileReport> RepairLog::Take() const
	{
		const std::scoped_lock lock(mutex);
		std::vector<FileReport> sorted = reports;
		std::ranges::sort(sorted, [](const FileReport& a_left, const FileReport& a_right) { return Util::ToLowerAscii(a_left.file) < Util::ToLowerAscii(a_right.file); });
		return sorted;
	}

	void RepairLog::Reset()
	{
		const std::scoped_lock lock(mutex);
		reports.clear();
	}

	std::string RepairLog::Format(const FileReport& a_report)
	{
		std::string text = std::format("Physics file: {}\n\nGPU SMP Physics found {} problem(s) in this file:\n", a_report.file, a_report.entries.size());
		for (std::size_t index = 0; index < a_report.entries.size(); ++index)
			text += std::format("{}. {}\n   What GPU SMP Physics did: {}\n", index + 1, a_report.entries[index].problem, a_report.entries[index].action);
		text += "\nhdtSMP64 may reject or mis-simulate a file with these problems.\n";
		return text;
	}

	bool RepairLog::IsFileFault(std::string_view a_reason)
	{
		constexpr std::array<std::string_view, 5> kNotTheFile = {
			"collision is switched off", "the GPU solver is not available", "the rigid-body solver is not available", "the mesh has no physics file", "claiming threw"
		};
		return std::ranges::none_of(kNotTheFile, [a_reason](std::string_view a_prefix) { return a_reason.starts_with(a_prefix); });
	}
}
