// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

// Built without the plugin PCH: the SMF header's typedefs conflict with imgui.h.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#include <RE/Skyrim.h>
#include <REX/REX.h>
#include <SKSE/SKSE.h>

#include <cctype>
#include <string>
#include <vector>

namespace logger = SKSE::log;

#include "SKSEMenuFramework.h"

#include "Utils/StringUtils.h"
#include "ui/UiModel.h"

namespace SettingsUi
{
	namespace
	{
		constexpr std::size_t kMaxPages = 6;
		std::vector<Model*> models;

		void Tip(const std::string& a_text)
		{
			if (!a_text.empty() && ImGuiMCP::IsItemHovered(ImGuiMCP::ImGuiHoveredFlags_AllowWhenDisabled))
				ImGuiMCP::SetTooltip("%s", a_text.c_str());
		}

		/** @brief Draws a filter box above a list of row.items and returns true when an item was clicked; the page scrolls to the selected item when it changes elsewhere. */
		bool RenderList(Row& row, const std::string& a_id)
		{
			if (row.items.empty() && row.itemsFn)
				row.items = row.itemsFn();
			constexpr std::size_t kMaxRows = 300;
			constexpr float kWidth = 420.0f;
			constexpr float kListBoxRows = 8.0f;
			ImGuiMCP::TextUnformatted(row.label.c_str());
			Tip(row.tooltip);
			ImGuiMCP::SetNextItemWidth(kWidth);
			ImGuiMCP::InputTextWithHint(("##gsp_filter_" + row.label).c_str(), "Filter", row.filterText.data(), row.filterText.size());
			const std::string filter = Util::ToLowerAscii(row.filterText.data());
			std::vector<int> status;
			if (row.itemStatusFn) {
				status = row.itemStatusFn();
				ImGuiMCP::SetNextItemWidth(kWidth);
				const int before = row.showMode;
				ImGuiMCP::Combo(("##gsp_show_" + row.label).c_str(), &row.showMode, "All\0With physics\0Needs attention\0", -1);
				if (row.showMode != before && row.showMode != 0 && row.onShowFiltered)
					row.onShowFiltered();
			}
			const auto statusOf = [&status](std::size_t a_index) { return a_index < status.size() ? status[a_index] : 0; };
			const int selected = row.selectedFn ? row.selectedFn() : *row.intValue;
			const bool scroll = selected != row.lastSelected;
			bool clicked = false;
			if (ImGuiMCP::BeginListBox(a_id.c_str(), { kWidth, kListBoxRows * ImGuiMCP::GetTextLineHeightWithSpacing() })) {
				std::size_t shown = 0;
				std::size_t hidden = 0;
				for (std::size_t index = 0; index < row.items.size(); ++index) {
					const std::string& name = row.items[index];
					constexpr const char* kStateTags[] = { "", "  [OK]", "  [REPAIRED]", "  [FAULT]" };
					if (!filter.empty() && Util::ToLowerAscii(name).find(filter) == std::string::npos)
						continue;
					const int state = statusOf(index);
					if ((row.showMode == 1 && state == 0) || (row.showMode == 2 && state < 2))
						continue;
					if (shown >= kMaxRows) {
						++hidden;
						continue;
					}
					++shown;
					const bool isSelected = static_cast<int>(index) == selected;
					const bool coloured = state > 0;
					if (coloured) {
						constexpr ImGuiMCP::ImVec4 kColours[] = { { 0.40f, 0.85f, 0.45f, 1.0f }, { 0.95f, 0.65f, 0.10f, 1.0f }, { 0.95f, 0.35f, 0.35f, 1.0f } };
						ImGuiMCP::PushStyleColor(ImGuiMCP::ImGuiCol_Text, kColours[state - 1]);
					}
					if (ImGuiMCP::Selectable((name + kStateTags[state < 3 ? state : 3] + "##gsp_item_" + std::to_string(index)).c_str(), isSelected, 0, { 0, 0 })) {
						*row.intValue = static_cast<int>(index);
						clicked = true;
					}
					if (coloured)
						ImGuiMCP::PopStyleColor(1);
					if (isSelected && scroll)
						ImGuiMCP::SetScrollHereY(0.5f);
				}
				if (hidden)
					ImGuiMCP::TextDisabled("%zu more match; type to narrow the list.", hidden);
				ImGuiMCP::EndListBox();
			}
			row.lastSelected = selected;
			return clicked;
		}

		void RenderPage(Model* model)
		{
			if (!model)
				return;
			if (model->alerts) {
				constexpr ImGuiMCP::ImVec4 kAlertBackground{ 0.45f, 0.05f, 0.05f, 1.0f };
				constexpr ImGuiMCP::ImVec4 kAlertText{ 1.0f, 0.92f, 0.92f, 1.0f };
				const std::vector<std::string> alerts = model->alerts();
				if (!alerts.empty()) {
					ImGuiMCP::PushStyleColor(ImGuiMCP::ImGuiCol_ChildBg, kAlertBackground);
					ImGuiMCP::PushStyleColor(ImGuiMCP::ImGuiCol_Text, kAlertText);
					if (ImGuiMCP::BeginChild("##gsp_alerts", { 0, 0 }, ImGuiMCP::ImGuiChildFlags_Border | ImGuiMCP::ImGuiChildFlags_AutoResizeY, 0)) {
						for (const std::string& alert : alerts)
							ImGuiMCP::TextWrapped("%s", alert.c_str());
					}
					ImGuiMCP::EndChild();
					ImGuiMCP::PopStyleColor(2);
				}
			}
			if (model->lockedReason) {
				const std::string reason = model->lockedReason();
				if (!reason.empty()) {
					ImGuiMCP::TextWrapped("%s", reason.c_str());
					return;
				}
			}
			if (model->status) {
				const std::string status = model->status();
				ImGuiMCP::TextWrapped("%s", status.c_str());
			}
			for (Row& row : model->rows) {
				// The SMF API has no PushID, so each control's label carries its own unique suffix.
				const std::string id = row.label + "##gsp_" + row.label;
				bool changed = false;
				const bool disabled = row.enabled && !row.enabled();
				if (disabled)
					ImGuiMCP::BeginDisabled(true);
				switch (row.kind) {
				case Row::Kind::Checkbox:
					changed = ImGuiMCP::Checkbox(id.c_str(), row.boolValue);
					Tip(row.tooltip);
					break;
				case Row::Kind::Slider:
					changed = ImGuiMCP::SliderFloat(id.c_str(), row.floatValue, row.minimum, row.maximum, row.format, ImGuiMCP::ImGuiSliderFlags_AlwaysClamp);
					Tip(row.tooltip);
					break;
				case Row::Kind::IntSlider:
					changed = ImGuiMCP::SliderInt(id.c_str(), row.intValue, static_cast<int>(row.minimum), static_cast<int>(row.maximum), row.format, ImGuiMCP::ImGuiSliderFlags_AlwaysClamp);
					Tip(row.tooltip);
					break;
				case Row::Kind::Header:
					ImGuiMCP::SeparatorText(row.label.c_str());
					break;
				case Row::Kind::Note:
					ImGuiMCP::TextDisabled("%s", (row.textFn ? row.textFn() : row.label).c_str());
					break;
				case Row::Kind::Button:
					if (row.sameLine)
						ImGuiMCP::SameLine(0.0f, -1.0f);
					{
						const std::string copy = row.clipboardFn ? row.clipboardFn() : std::string();
						const bool noText = row.clipboardFn && copy.empty();
						if (noText)
							ImGuiMCP::BeginDisabled(true);
						changed = ImGuiMCP::Button(id.c_str(), { 0, 0 });
						if (noText)
							ImGuiMCP::EndDisabled();
						if (changed && !copy.empty())
							ImGuiMCP::SetClipboardText(copy.c_str());
					}
					Tip(row.tooltip);
					break;
				case Row::Kind::Combo:
					{
						if (row.joinedItems.empty()) {
							if (row.items.empty() && row.itemsFn)
								row.items = row.itemsFn();
							for (const std::string& item : row.items) {
								row.joinedItems += item;
								row.joinedItems.push_back('\0');
							}
							if (row.joinedItems.empty())
								row.joinedItems.assign("(none)", sizeof("(none)"));
						}
						constexpr float kComboWidth = 300.0f;
						ImGuiMCP::SetNextItemWidth(kComboWidth);
						changed = ImGuiMCP::Combo(id.c_str(), row.intValue, row.joinedItems.c_str(), -1);
						Tip(row.tooltip);
					}
					break;
				case Row::Kind::List:
					changed = RenderList(row, id);
					break;
				}
				if (disabled)
					ImGuiMCP::EndDisabled();
				if (changed && row.onChanged)
					row.onChanged();
			}
			if (model->claims) {
				const std::vector<ClaimLine> lines = model->claims();
				if (lines.empty())
					ImGuiMCP::TextDisabled("Nothing is claimed.");
				for (const ClaimLine& line : lines) {
					if (line.enabled < 0) {
						ImGuiMCP::TextDisabled("%s", line.text.c_str());
						continue;
					}
					ImGuiMCP::PushID(static_cast<int>(line.id));
					bool enabled = line.enabled != 0;
					if (ImGuiMCP::Checkbox("##gpu", &enabled) && model->onEnabled)
						model->onEnabled(line.id, enabled);
					ImGuiMCP::SameLine(0.0f, -1.0f);
					ImGuiMCP::TextUnformatted(line.text.c_str());
					ImGuiMCP::PopID();
				}
			}
			if (model->reports) {
				const std::vector<ReportBlock> blocks = model->reports();
				if (blocks.empty())
					ImGuiMCP::TextDisabled("No physics file has needed a repair or been declined for its own fault yet.");
				std::string all;
				for (const ReportBlock& block : blocks)
					all += block.text + '\n';
				if (!blocks.empty() && ImGuiMCP::Button("Copy all##gsp_report_copy_all", { 0, 0 }))
					ImGuiMCP::SetClipboardText(all.c_str());
				for (std::size_t index = 0; index < blocks.size(); ++index) {
					ImGuiMCP::PushID(static_cast<int>(index));
					if (ImGuiMCP::CollapsingHeader((blocks[index].title + "##gsp_report_header").c_str(), 0)) {
						ImGuiMCP::TextWrapped("%s", blocks[index].text.c_str());
						if (ImGuiMCP::Button("Copy##gsp_report_copy", { 0, 0 }))
							ImGuiMCP::SetClipboardText(blocks[index].text.c_str());
					}
					ImGuiMCP::PopID();
				}
			}
			if (!model->showFooter)
				return;
			ImGuiMCP::Separator();
			if (model->onReevaluate && ImGuiMCP::Button("Re-evaluate actors##gsp_reevaluate", { 0, 0 }))
				model->onReevaluate();
			ImGuiMCP::SameLine(0.0f, -1.0f);
			if (model->onSave && ImGuiMCP::Button("Save settings##gsp_save", { 0, 0 }))
				model->onSave();
			Tip("Writes your settings so they apply the next time the game starts. Changes made here already apply to the running game.");
			if (model->onReset) {
				ImGuiMCP::SameLine(0.0f, -1.0f);
				if (ImGuiMCP::Button("Reset to defaults##gsp_reset", { 0, 0 }))
					model->onReset();
				Tip("Puts every setting on every page back to its default. Nothing is written until you press Save settings.");
			}
		}
	}

	namespace
	{
		template <std::size_t N>
		void RenderNth()
		{
			RenderPage(N < models.size() ? models[N] : nullptr);
		}
		constexpr void (*kRenderers[kMaxPages])() = { RenderNth<0>, RenderNth<1>, RenderNth<2>, RenderNth<3>, RenderNth<4>, RenderNth<5> };
	}

	void Register(const std::vector<Model*>& a_models)
	{
		if (!SKSEMenuFramework::IsInstalled()) {
			logger::info("SKSE Menu Framework not installed; in-game settings disabled.");
			return;
		}
		models = a_models;
		if (models.size() > kMaxPages)
			models.resize(kMaxPages);
		// The header caches the module handle in a static initialised when this DLL loads, which is null if SKSEMenuFramework.dll loads after us.
		menuFramework = GetModuleHandle(L"SKSEMenuFramework");
		SKSEMenuFramework::SetSection("GPU SMP Physics NG");
		for (std::size_t page = 0; page < models.size(); ++page)
			SKSEMenuFramework::AddSectionItem(models[page]->title, kRenderers[page]);
		logger::info("Registered SMF page (framework v{:.1f}).", SKSEMenuFramework::GetMenuFrameworkVersion());
	}
}
