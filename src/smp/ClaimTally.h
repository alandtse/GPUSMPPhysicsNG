#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace SmpPhysics
{
	/** @brief Why meshes were offered and not taken, counted so the commonest reasons can be ranked. */
	class ClaimTally
	{
	public:
		enum class Kind : std::uint8_t
		{
			Armor,
			HeadPart
		};

		static constexpr std::size_t kKinds = 2;
		/** @brief Distinct (actor, file) pairs remembered; past this the event counts keep growing but the pair counts stop. */
		static constexpr std::size_t kMaxPairs = 65536;
		static constexpr std::size_t kMaxReasons = 512;
		static constexpr std::size_t kMaxFiles = 4096;
		static constexpr std::size_t kSampleFiles = 5;

		struct KindCount
		{
			std::uint64_t attempts{};
			std::uint64_t accepts{};
			std::uint64_t declines{};
		};

		struct ReasonRow
		{
			std::string reason;   ///< With quoted names and numbers replaced, so one cause is one row.
			std::string example;  ///< The first full reason seen.
			std::uint64_t declines{};
			std::size_t pairs{};  ///< Distinct (actor, file) pairs whose latest outcome is this reason.
			std::array<std::uint64_t, kKinds> byKind{};
			std::vector<std::string> sampleFiles;
		};

		struct FileRow
		{
			std::string file;
			std::uint64_t attempts{};
			std::uint64_t accepts{};
			std::uint64_t declines{};
		};

		struct Snapshot
		{
			std::array<KindCount, kKinds> kinds{};
			std::size_t pairsAccepted{};  ///< Distinct pairs whose latest attempt was taken.
			std::size_t pairsDeclined{};
			bool pairsTruncated{};
			std::vector<ReasonRow> reasons;  ///< Most pairs first.
			std::vector<FileRow> files;      ///< Most declines first.
		};

		/** @brief Counts one offer; a_reason is empty when it was taken. */
		void Record(Kind a_kind, std::uint32_t a_actor, std::string_view a_file, bool a_accepted, std::string_view a_reason);

		[[nodiscard]] Snapshot Take() const;

		/** @brief The (actor, lowercase file) pairs whose latest outcome was a decline; actor 0 (no actor) is left out. */
		[[nodiscard]] std::vector<std::pair<std::uint32_t, std::string>> DeclinedPairs() const;

		void Reset();

		/** @brief a_reason with each quoted name replaced by '?' and each run of digits by N. */
		[[nodiscard]] static std::string Normalize(std::string_view a_reason);

	private:
		struct ReasonData
		{
			std::string example;
			std::uint64_t declines{};
			std::array<std::uint64_t, kKinds> byKind{};
			std::vector<std::string> sampleFiles;
		};

		struct PairState
		{
			bool accepted{};
			std::string reason;
		};

		mutable std::mutex mutex;
		std::array<KindCount, kKinds> kinds{};
		std::map<std::string, ReasonData> reasons;
		std::map<std::string, FileRow> files;
		std::map<std::pair<std::uint32_t, std::string>, PairState> pairs;
		bool pairsTruncated{};
	};
}
