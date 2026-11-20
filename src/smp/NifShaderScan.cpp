// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "smp/NifShaderScan.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace SmpPhysics
{
	namespace
	{
		constexpr std::uint32_t kMaxBlocks = 1u << 20;
		constexpr std::uint32_t kMaxTypes = 4096;
		constexpr std::string_view kLightingShader = "BSLightingShaderProperty";

		/** @brief Reads little-endian values in order; any read past the end fails the whole parse. */
		class Reader
		{
		public:
			explicit Reader(std::string_view a_bytes) :
				bytes(a_bytes) {}

			bool Ok() const { return ok; }
			std::size_t Position() const { return position; }

			template <class T>
			T Read()
			{
				T value{};
				if (!Take(sizeof(T)))
					return value;
				std::memcpy(&value, bytes.data() + position - sizeof(T), sizeof(T));
				return value;
			}

			std::string_view Text(std::size_t a_length)
			{
				if (!Take(a_length))
					return {};
				return bytes.substr(position - a_length, a_length);
			}

			void Skip(std::size_t a_length) { Take(a_length); }

		private:
			bool Take(std::size_t a_length)
			{
				if (!ok || a_length > bytes.size() - position) {
					ok = false;
					return false;
				}
				position += a_length;
				return true;
			}

			std::string_view bytes;
			std::size_t position{};
			bool ok{ true };
		};
	}

	NifShaderHint ScanNifShaders(std::string_view a_bytes)
	{
		NifShaderHint hint;
		const std::size_t newline = a_bytes.find('\n');
		if (newline == std::string_view::npos)
			return hint;
		Reader reader(a_bytes);
		reader.Skip(newline + 1);
		reader.Skip(sizeof(std::uint32_t) + sizeof(std::uint8_t) + sizeof(std::uint32_t));  // version, endian, user version
		const auto blockCount = reader.Read<std::uint32_t>();
		reader.Skip(sizeof(std::uint32_t));      // user version 2
		for (int field = 0; field < 3; ++field)  // creator, export info 1 and 2, each a one-byte length and text
			reader.Skip(reader.Read<std::uint8_t>());
		const auto typeCount = reader.Read<std::uint16_t>();
		if (!reader.Ok() || blockCount > kMaxBlocks || typeCount > kMaxTypes)
			return hint;

		std::vector<bool> isLighting(typeCount);
		for (std::uint16_t type = 0; type < typeCount; ++type)
			isLighting[type] = reader.Text(reader.Read<std::uint32_t>()) == kLightingShader;
		std::vector<std::uint16_t> typeOf(blockCount);
		for (std::uint32_t block = 0; block < blockCount; ++block)
			typeOf[block] = reader.Read<std::uint16_t>();
		std::vector<std::uint32_t> sizeOf(blockCount);
		for (std::uint32_t block = 0; block < blockCount; ++block)
			sizeOf[block] = reader.Read<std::uint32_t>();
		const auto stringCount = reader.Read<std::uint32_t>();
		reader.Skip(sizeof(std::uint32_t));  // longest string
		for (std::uint32_t index = 0; index < stringCount && reader.Ok(); ++index)
			reader.Skip(reader.Read<std::uint32_t>());
		const auto groupCount = reader.Read<std::uint32_t>();
		if (!reader.Ok() || groupCount > a_bytes.size() / sizeof(std::uint32_t))
			return NifShaderHint{};
		reader.Skip(static_cast<std::size_t>(groupCount) * sizeof(std::uint32_t));

		for (std::uint32_t block = 0; block < blockCount && reader.Ok(); ++block) {
			const std::size_t start = reader.Position();
			if (typeOf[block] < typeCount && isLighting[typeOf[block]]) {
				const auto shaderType = reader.Read<std::uint32_t>();
				if (!reader.Ok())
					break;
				++hint.lightingShaders;
				if (shaderType == kHairTintShaderType)
					++hint.hairTintShaders;
			}
			reader.Skip(start + sizeOf[block] - reader.Position());
		}
		return hint;
	}
}
