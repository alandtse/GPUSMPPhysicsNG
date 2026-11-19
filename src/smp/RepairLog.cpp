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
		std::string text = std::format("Physics file: {}\n\n{} problem(s) found by GPU SMP Physics:\n", a_report.file, a_report.entries.size());
		for (std::size_t index = 0; index < a_report.entries.size(); ++index) {
			text += std::format("{}. {}\n", index + 1, a_report.entries[index].problem);
			if (!a_report.entries[index].action.empty())
				text += std::format("   -> {}\n", a_report.entries[index].action);
		}
		text += "\nFix these in the file so every loader reads it the same way.\n";
		return text;
	}

	bool RepairLog::IsFileFault(std::string_view a_reason)
	{
		constexpr auto kNotTheFile = std::to_array<std::string_view>({ "collision is switched off", "the GPU solver is not available", "the rigid-body solver is not available",
			"the mesh has no physics file", "claiming threw", "the mesh has no skeleton node", "the actor has no scene graph", "none of the hair meshes",
			"the rest pose is not usable by the solver", "the rigid claim", "the rig could not be packed", "the rig has more collision sphere lists",
			"a chain has more bones than the compute solver", "the rig needs ", "the GPU solver has no free slot" });
		return std::ranges::none_of(kNotTheFile, [a_reason](std::string_view a_prefix) { return a_reason.starts_with(a_prefix); });
	}
}
