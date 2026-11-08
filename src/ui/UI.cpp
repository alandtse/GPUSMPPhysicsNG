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
		Model* model = nullptr;

		void Tip(const std::string& a_text)
		{
			if (!a_text.empty() && ImGuiMCP::IsItemHovered(ImGuiMCP::ImGuiHoveredFlags_AllowWhenDisabled))
				ImGuiMCP::SetTooltip("%s", a_text.c_str());
		}

		void RenderPage()
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
					ImGuiMCP::TextDisabled("%s", row.label.c_str());
					break;
				}
				if (changed && row.onChanged)
					row.onChanged();
			}
			ImGuiMCP::Separator();
			if (model->onReevaluate && ImGuiMCP::Button("Re-evaluate actors##gsp_reevaluate", { 0, 0 }))
				model->onReevaluate();
			ImGuiMCP::SameLine(0.0f, -1.0f);
			if (model->onSave && ImGuiMCP::Button("Save settings##gsp_save", { 0, 0 }))
				model->onSave();
		}

		HMODULE menuFramework = nullptr;
	}

	void Register(Model* a_model)
	{
		if (!SKSEMenuFramework::IsInstalled()) {
			logger::info("SKSE Menu Framework not installed; in-game settings disabled.");
			return;
		}
		model = a_model;
		// The header caches the module handle in a static initialised when this DLL loads, which is null if SKSEMenuFramework.dll loads after us.
		menuFramework = GetModuleHandle(L"SKSEMenuFramework");
		SKSEMenuFramework::SetSection("GPU SMP Physics NG");
		SKSEMenuFramework::AddSectionItem("Settings", RenderPage);
		logger::info("Registered SMF page (framework v{:.1f}).", SKSEMenuFramework::GetMenuFrameworkVersion());
	}
}
