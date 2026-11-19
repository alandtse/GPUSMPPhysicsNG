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
	/** @brief An order a List can show its rows in besides the built-in ones, such as closest to a hair target. */
	struct SortChoice
	{
		std::string label;
		std::function<std::vector<int>()> orderFn;          ///< The item indexes in display order, read every frame; an order of another length is ignored.
		std::function<std::vector<std::string>()> badgeFn;  ///< Text after each item's name, indexed by item, while this order is chosen.
	};

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
		std::function<std::string()> clipboardFn;           ///< A Button copies this text to the clipboard when pressed, and is greyed out while it is empty.
		std::function<std::vector<int>()> itemStatusFn;     ///< A List's per-item state read every frame: 0 unknown, 1 working (green), 2 repaired (amber), 3 broken (red). Adds a Show filter.
		std::function<void()> onActivated;                  ///< Runs when a List item is double-clicked, after that click has selected it.
		std::function<void()> onShowFiltered;               ///< Runs when a List's Show filter is switched away from All, so the data it filters by can be gathered.
		int showMode{};                                     ///< Which items a List with itemStatusFn shows: 0 all, 1 with physics, 2 needing attention.
		std::array<char, 64> filterText{};                  ///< What a List's filter box holds.
		int pickedIndex{ -1 };                              ///< The item the user last clicked in a List, so the list does not scroll under the cursor.
		int lastSelected{ -2 };                             ///< The item a List last scrolled to.
		/** @brief A List's physics file paths per item, in the order of items; the filter box matches them as well as the names. */
		std::function<std::vector<std::string>()> itemFilesFn;
		/** @brief A List shows a progress bar while this reports a total above zero: what is done of what. */
		std::function<std::pair<std::size_t, std::size_t>()> progressFn;
		/** @brief Whether the filter box has already asked for the data its filter needs. */
		bool scanAsked{};
		/** @brief Orders a List offers first, before its built-in ones; the first is the one the list opens with. */
		std::vector<SortChoice> sorts;
		int sortIndex{};  ///< The chosen order: an entry of sorts, then the built-in ones.
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
		/** Why the GPU is not stepping a simulated line this frame; empty while it is. */
		std::string state;
		/** Whether a saved rule can name this line's character type. */
		bool canRememberBase{};
	};

	/** @brief One physics file's problems as text, for the Reports page to show and copy. */
	struct ReportBlock
	{
		std::string title;
		std::string text;
	};

	struct Model
	{
		std::string title;  ///< The page's name in the framework's menu.
		std::vector<Row> rows;
		std::function<std::vector<ClaimLine>()> claims;      ///< When set, the page lists these with a GPU switch each.
		std::function<void(std::uint64_t, bool)> onEnabled;  ///< Runs with a line's id and whether the GPU should simulate it.
		bool showFooter{ true };                             ///< Whether the page ends with Re-evaluate and Save.
		std::function<std::vector<ReportBlock>()> reports;   ///< When set, the page lists these, each with a Copy button.
		std::function<std::string()> status;                 ///< Text shown above the controls, refreshed every frame.
		std::function<std::vector<std::string>()> alerts;    ///< Problems drawn in red above the status on every refresh; none when the list is empty.
		std::function<std::string()> lockedReason;           ///< While this returns text the page shows only that text, not its controls.
		std::function<void()> onSave;                        ///< Writes Settings.json.
		std::function<void()> onReevaluate;                  ///< Asks hdtSMP64 to offer every actor's meshes again.
		std::function<void()> onReset;                       ///< Puts every setting back to its default; the page then shows them and Save writes them.
		/** Saves a rule keeping a line's file with hdtSMP64; true for its character type only, false for every character. */
		std::function<void(std::uint64_t, bool)> onRemember;
		/** When set, the page lists the saved rules, each with a Remove button. */
		std::function<std::vector<std::string>()> rules;
		/** Drops the saved rule at this index. */
		std::function<void(std::size_t)> onRemoveRule;
	};

	/** @brief Registers one page per model, in order, with SKSE Menu Framework; a no-op when it is not installed. The models must outlive the game session. */
	void Register(const std::vector<Model*>& a_models);
}
