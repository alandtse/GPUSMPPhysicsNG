// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <span>
#include <string>

namespace SmpPhysics
{
	/**
	 * @brief The mesh that stands for a hair head part: its own, else the first extra part's whole-hair mesh.
	 * @details An SMP hair head part often has no model of its own; its extra parts hold the whole hair, its split pieces
	 *          (paths containing "extraParts") and its collision meshes (paths containing "collision").
	 * @param a_own The head part's own model path.
	 * @param a_extras The models of its extra parts, in order.
	 * @return The path, or empty when none qualifies.
	 */
	[[nodiscard]] std::string HairPartModel(const std::string& a_own, std::span<const std::string> a_extras);
}
