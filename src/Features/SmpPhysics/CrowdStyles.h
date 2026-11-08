#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace SmpPhysics
{
	/** @brief A hair-slot armor that could give a test character a hairstyle. */
	struct WigCandidate
	{
		std::string name;   ///< Editor name; two armors with the same name count as one style.
		std::string model;  ///< Worn model path, which is where hdtSMP64 physics content usually shows in the name.
	};

	/**
	 * @brief Picks up to a_count candidates that look like different hairstyles.
	 * @details One candidate per distinct name, ignoring case. Candidates whose model path mentions hdt or smp come first, since
	 *          those are the ones hdtSMP64 can simulate, then the rest, each group in name order so the pick is repeatable.
	 * @return Indices into a_candidates, fewer than a_count when there are not that many distinct styles.
	 */
	[[nodiscard]] std::vector<std::size_t> PickDistinctWigs(const std::vector<WigCandidate>& a_candidates, std::size_t a_count);

	/** @brief A vanilla hair colour a test crowd can wear. */
	struct HairColorChoice
	{
		std::string_view name;
		std::uint32_t formId;  ///< The Skyrim.esm BGSColorForm.
	};

	/** @brief The vanilla hair colours the crowd can be set to, by name, darkest first. */
	[[nodiscard]] const std::vector<HairColorChoice>& TestCrowdHairColors();

	/** @return The form ID of the hair colour named a_name, ignoring case, spaces and underscores, or none. */
	[[nodiscard]] std::optional<std::uint32_t> ResolveHairColor(std::string_view a_name);
}
