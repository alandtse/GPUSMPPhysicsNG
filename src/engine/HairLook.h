// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

namespace HairLook
{
	/**
	 * @brief Ranks the hair-slot armors by how closely their shape matches one hair, and keeps the answer for hairLookResult.
	 * @details Args: actorFormID (number, whose own hair is the target), model (string, a model path under Meshes), name (string, part of a
	 *          hair-slot armor's name), sex (0 male, 1 female), filter (string, part of a candidate's name), physicsOnly (bool, default
	 *          true once a scan has run), budgetMs (number, default 3000), top (number, default 10). Shapes are read once and kept, so
	 *          repeat the command while the result says pending. Game thread only.
	 */
	void Rank(const json& a_args);

	/** @brief The outcome of the last Rank. */
	[[nodiscard]] json Result();
}
