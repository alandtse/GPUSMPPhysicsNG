#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace SmpPhysics
{
	/** @brief What was wrong in each physics file and what was done about it, so it can be passed on to the file's author. */
	class RepairLog
	{
	public:
		static constexpr std::size_t kMaxFiles = 512;
		static constexpr std::size_t kMaxEntriesPerFile = 32;

		struct Entry
		{
			std::string problem;
			std::string action;
		};

		struct FileReport
		{
			std::string file;
			std::vector<Entry> entries;
		};

		/** @brief Notes one problem in a file; the same problem noted twice counts once, and files are matched without regard to case. */
		void Record(std::string_view a_file, std::string_view a_problem, std::string_view a_action);

		/** @brief Every file with a problem, sorted by name. */
		[[nodiscard]] std::vector<FileReport> Take() const;

		void Reset();

		/** @brief A number that changes whenever the log does, so a reader can tell when to look again. */
		[[nodiscard]] std::uint32_t Revision() const noexcept { return revision.load(std::memory_order_relaxed); }

		/** @brief A report as text to paste to the file's author. */
		[[nodiscard]] static std::string Format(const FileReport& a_report);

		/** @brief Whether a decline reason is about the file, not about this plugin's settings or the machine. */
		[[nodiscard]] static bool IsFileFault(std::string_view a_reason);

	private:
		mutable std::mutex mutex;
		std::atomic<std::uint32_t> revision{};
		std::vector<FileReport> reports;
	};
}
