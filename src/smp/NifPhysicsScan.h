// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace SmpPhysics
{
	/** @brief The name hdtSMP64 gives the NiStringExtraData that points a model at its physics file. */
	inline constexpr std::string_view kPhysicsExtraDataName = "HDT Skinned Mesh Physics Object";

	/** @brief What a model's header says about its physics, found without loading the model. */
	struct NifPhysicsHint
	{
		bool hasExtraData{};                ///< The header names the physics extra data, so hdtSMP64 will take its path.
		std::vector<std::string> xmlPaths;  ///< Every string of the header that ends in .xml; the extra data's value is one of them.
	};

	/**
	 * @brief Reads a NIF file's leading bytes for the physics extra data, without parsing the file.
	 * @details A NIF header keeps its strings (block names and extra data names and values) as a table of
	 *          length-prefixed text, so both the name and the path are found by looking for that shape. A model whose
	 *          physics comes from hdtSMP64's default map rather than extra data reports nothing here.
	 */
	[[nodiscard]] NifPhysicsHint ScanNifHeader(std::string_view a_bytes);
}
