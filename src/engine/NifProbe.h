// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

namespace NifProbe
{
	/** @brief Starts reading the leading bytes of every body armor's and cloak's models on a worker thread, timing it. Game thread only. */
	void Start();

	/** @brief Progress, timings and the models found carrying physics extra data. */
	[[nodiscard]] json Result();
}
