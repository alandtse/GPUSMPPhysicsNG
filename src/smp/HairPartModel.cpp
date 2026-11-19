// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "smp/HairPartModel.h"

#include <algorithm>
#include <cctype>

namespace SmpPhysics
{
	namespace
	{
		bool Contains(std::string a_text, std::string_view a_word)
		{
			std::ranges::transform(a_text, a_text.begin(), [](unsigned char a_c) { return static_cast<char>(std::tolower(a_c)); });
			return a_text.find(a_word) != std::string::npos;
		}
	}

	std::string HairPartModel(const std::string& a_own, std::span<const std::string> a_extras)
	{
		if (!a_own.empty())
			return a_own;
		for (const std::string& model : a_extras) {
			if (!model.empty() && !Contains(model, "extraparts") && !Contains(model, "collision"))
				return model;
		}
		return {};
	}
}
