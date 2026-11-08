#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "RE/Skyrim.h"

namespace TestBench
{
	/** @brief One idle animation the game defines: its editor ID, the animation event that plays it, and its form. */
	struct IdleChoice
	{
		std::string editorId;
		std::string event;
		RE::TESIdleForm* form = nullptr;
	};

	/**
	 * @brief The idle animations the game loaded, collected as their editor IDs are read.
	 * @details TESDataHandler's idle array is never filled, so the only way to list idles is to hook TESIdleForm::SetFormEditorID
	 *          before the game data loads. Paired animations ("pa_") need a second actor and are left out.
	 */
	namespace IdleCache
	{
		/** @brief Hooks the editor ID setter; call during plugin load, before game data is read. Safe to call more than once. */
		void Install();

		/** @brief Every cached idle, by editor ID. */
		[[nodiscard]] std::vector<IdleChoice> Snapshot();

		/** @brief The idle with this editor ID, or null. */
		[[nodiscard]] RE::TESIdleForm* Find(std::string_view a_editorId);
	}
}
