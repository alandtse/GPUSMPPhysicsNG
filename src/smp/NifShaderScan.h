// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace SmpPhysics
{
	/** @brief The shader types a model's lighting shaders use, read without loading the model. */
	struct NifShaderHint
	{
		std::size_t lightingShaders{};  ///< Skyrim lighting shader blocks in the file.
		std::size_t hairTintShaders{};  ///< Of those, the ones whose shader type tints from the character's hair colour.
	};

	/** @brief The shader type that takes the hair colour of the character wearing the model. */
	inline constexpr std::uint32_t kHairTintShaderType = 6;

	/**
	 * @brief Counts a NIF file's lighting shader blocks and how many take the hair colour.
	 * @details Walks the header's block table (types, sizes) to each lighting shader block and reads its shader type. A file
	 *          that is not a Skyrim NIF, or is cut short, reports nothing.
	 */
	[[nodiscard]] NifShaderHint ScanNifShaders(std::string_view a_bytes);
}
