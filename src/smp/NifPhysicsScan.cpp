// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "smp/NifPhysicsScan.h"

#include <algorithm>
#include <cstdint>

namespace SmpPhysics
{
	namespace
	{
		constexpr std::uint32_t kLongestString = 260;

		bool IsPrintable(char a_character)
		{
			return a_character >= 0x20 && a_character < 0x7F;
		}

		bool EndsWithXml(std::string_view a_text)
		{
			if (a_text.size() < 4)
				return false;
			const std::string_view tail = a_text.substr(a_text.size() - 4);
			return std::ranges::equal(tail, std::string_view(".xml"), [](char a_left, char a_right) {
				return (a_left >= 'A' && a_left <= 'Z' ? static_cast<char>(a_left + 32) : a_left) == a_right;
			});
		}
	}

	NifPhysicsHint ScanNifHeader(std::string_view a_bytes)
	{
		NifPhysicsHint hint;
		for (std::size_t offset = 0; offset + 5 <= a_bytes.size(); ++offset) {
			const auto length = static_cast<std::uint32_t>(static_cast<unsigned char>(a_bytes[offset])) |
			                    static_cast<std::uint32_t>(static_cast<unsigned char>(a_bytes[offset + 1])) << 8 |
			                    static_cast<std::uint32_t>(static_cast<unsigned char>(a_bytes[offset + 2])) << 16 |
			                    static_cast<std::uint32_t>(static_cast<unsigned char>(a_bytes[offset + 3])) << 24;
			if (length == 0 || length > kLongestString || offset + 4 + length > a_bytes.size())
				continue;
			const std::string_view text = a_bytes.substr(offset + 4, length);
			if (!std::ranges::all_of(text, IsPrintable))
				continue;
			if (text == kPhysicsExtraDataName)
				hint.hasExtraData = true;
			else if (EndsWithXml(text) && std::ranges::find(hint.xmlPaths, std::string(text)) == hint.xmlPaths.end())
				hint.xmlPaths.emplace_back(text);
		}
		return hint;
	}
}
