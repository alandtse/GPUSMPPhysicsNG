// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <cstdint>

namespace SmpPhysics
{
	inline constexpr std::uint32_t kHairPartMaleFlag = 1u << 1;    ///< BGSHeadPart::Flag::kMale.
	inline constexpr std::uint32_t kHairPartFemaleFlag = 1u << 2;  ///< BGSHeadPart::Flag::kFemale.
	inline constexpr std::uint32_t kHairPartExtraFlag = 1u << 3;   ///< BGSHeadPart::Flag::kIsExtraPart.

	/** @brief Whether a hair head part with a_flags is made for a character of the given sex; an extra part (a beard-like add-on) is never a hairstyle on its own. */
	[[nodiscard]] constexpr bool HairPartFits(std::uint32_t a_flags, bool a_female) noexcept
	{
		if ((a_flags & kHairPartExtraFlag) != 0)
			return false;
		return (a_flags & (a_female ? kHairPartFemaleFlag : kHairPartMaleFlag)) != 0;
	}
}
