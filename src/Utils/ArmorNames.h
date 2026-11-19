#pragma once

#include "Utils/ItemNames.h"

namespace Util
{
	/** @brief The list names of a_armors in order, telling apart armors that share a name. */
	[[nodiscard]] inline std::vector<std::string> ItemNamesOf(std::span<RE::TESObjectARMO* const> a_armors)
	{
		std::vector<NamedItem> items;
		items.reserve(a_armors.size());
		for (const RE::TESObjectARMO* armor : a_armors)
			items.push_back({ armor->GetName(), armor->GetFormID() });
		return UniqueNames(items);
	}
}
