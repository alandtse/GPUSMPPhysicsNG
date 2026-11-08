// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <functional>
#include <string>
#include <vector>

/** @brief What the settings page shows, as plain data so the page code needs neither the plugin's PCH nor its feature types. */
namespace SettingsUi
{
	struct Row
	{
		enum class Kind
		{
			Checkbox,
			Slider,
			IntSlider,
			Header,
			Note
		};

		Kind kind{ Kind::Note };
		std::string label;
		std::string tooltip;
		bool* boolValue{};
		float* floatValue{};
		int* intValue{};
		float minimum{};
		float maximum{};
		const char* format{ "%.2f" };
		std::function<void()> onChanged;  ///< Runs after the user edits the control; may be empty.
	};

	struct Model
	{
		std::vector<Row> rows;
		std::function<std::string()> status;      ///< Text shown above the controls, refreshed every frame.
		std::function<void()> onSave;             ///< Writes Settings.json.
		std::function<void()> onReevaluate;       ///< Asks hdtSMP64 to offer every actor's meshes again.
	};

	/** @brief Registers the page with SKSE Menu Framework; a no-op when it is not installed. The model must outlive the game session. */
	void Register(Model* a_model);
}
