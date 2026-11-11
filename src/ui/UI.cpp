// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

// Built without the plugin PCH: the SMF header's typedefs conflict with imgui.h.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#include <RE/Skyrim.h>
#include <REX/REX.h>
#include <SKSE/SKSE.h>

#include <string>

namespace logger = SKSE::log;

#include "SKSEMenuFramework.h"

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

		void RenderPage(Model* model)
		{
			if (!model)
				return;
			if (model->status) {
				const std::string status = model->status();
				ImGuiMCP::TextWrapped("%s", status.c_str());
			}
			for (Row& row : model->rows) {
				// The SMF API has no PushID, so each control's label carries its own unique suffix.
				const std::string id = row.label + "##gsp_" + row.label;
				bool changed = false;
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
					changed = ImGuiMCP::Button(id.c_str(), { 0, 0 });
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
								row.joinedItems.assign("(none)", 7);
						}
						ImGuiMCP::SetNextItemWidth(300.0f);
						changed = ImGuiMCP::Combo(id.c_str(), row.intValue, row.joinedItems.c_str(), -1);
						Tip(row.tooltip);
					}
					break;
				}
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
			if (!model->showFooter)
				return;
			ImGuiMCP::Separator();
			if (model->onReevaluate && ImGuiMCP::Button("Re-evaluate actors##gsp_reevaluate", { 0, 0 }))
				model->onReevaluate();
			ImGuiMCP::SameLine(0.0f, -1.0f);
			if (model->onSave && ImGuiMCP::Button("Save settings##gsp_save", { 0, 0 }))
				model->onSave();
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
