// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <array>
#include <cstdint>
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
			Note,
			Button,  ///< Runs onChanged when pressed; sameLine puts it beside the previous control.
			Combo,   ///< Picks one of items into intValue.
			List     ///< A filter box over a list box of items; a click sets intValue and runs onChanged.
		};

		Kind kind{ Kind::Note };
		std::string label;
		std::string tooltip;
		bool* boolValue{};
		float* floatValue{};
		int* intValue{};
		std::vector<std::string> items;                     ///< The choices of a Combo.
		std::function<std::vector<std::string>()> itemsFn;  ///< Fills items the first time the page draws the Combo, for lists the game builds late.
		std::string joinedItems;                            ///< items as the framework wants them; built once by the page code.
		std::function<std::string()> textFn;                ///< A Note shows this instead of its label when set, refreshed every frame.
		std::function<int()> selectedFn;                    ///< A List's highlighted item, read every frame so changes made elsewhere show; intValue when empty.
		std::array<char, 64> filterText{};                  ///< What a List's filter box holds.
		int lastSelected{ -2 };                             ///< The item a List last scrolled to.
		bool sameLine{};
		float minimum{};
		float maximum{};
		const char* format{ "%.2f" };
		std::function<bool()> enabled;    ///< When set and false the control is drawn greyed out and cannot be edited.
		std::function<void()> onChanged;  ///< Runs after the user edits the control; may be empty.
	};

	/** @brief One claimed physics file and whether the GPU simulates it, for the Claims page. */
	struct ClaimLine
	{
		std::string text;
		int enabled{};  ///< 1 when the GPU simulates the file, 0 when hdtSMP64 does, -1 for a line that is only text.
		std::uint64_t id{};
	};

	struct Model
	{
		std::string title;  ///< The page's name in the framework's menu.
		std::vector<Row> rows;
		std::function<std::vector<ClaimLine>()> claims;      ///< When set, the page lists these with a GPU switch each.
		std::function<void(std::uint64_t, bool)> onEnabled;  ///< Runs with a line's id and whether the GPU should simulate it.
		bool showFooter{ true };                             ///< Whether the page ends with Re-evaluate and Save.
		std::function<std::string()> status;                 ///< Text shown above the controls, refreshed every frame.
		std::function<void()> onSave;                        ///< Writes Settings.json.
		std::function<void()> onReevaluate;                  ///< Asks hdtSMP64 to offer every actor's meshes again.
		std::function<void()> onReset;                       ///< Puts every setting back to its default; the page then shows them and Save writes them.
	};

	/** @brief Registers one page per model, in order, with SKSE Menu Framework; a no-op when it is not installed. The models must outlive the game session. */
	void Register(const std::vector<Model*>& a_models);
}
