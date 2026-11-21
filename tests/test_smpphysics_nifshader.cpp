// Unit tests for the NIF shader scan (src/smp/NifShaderScan.cpp). Pure bytes; no game dependency.

#include "smp/NifShaderScan.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

using namespace SmpPhysics;

namespace
{
	template <class T>
	void Put(std::string& a_bytes, T a_value)
	{
		a_bytes.append(reinterpret_cast<const char*>(&a_value), sizeof(T));
	}

	struct Block
	{
		std::string type;
		std::uint32_t shaderType;  ///< Written as the first field of a lighting shader block.
		std::uint32_t size;
	};

	/** @brief A Skyrim NIF header and block bodies with the given blocks, laid out as the file format has them. */
	std::string MakeNif(const std::vector<Block>& a_blocks)
	{
		std::vector<std::string> types;
		std::vector<std::uint16_t> typeOf;
		for (const Block& block : a_blocks) {
			std::size_t index = 0;
			while (index < types.size() && types[index] != block.type)
				++index;
			if (index == types.size())
				types.push_back(block.type);
			typeOf.push_back(static_cast<std::uint16_t>(index));
		}
		std::string bytes = "Gamebryo File Format, Version 20.2.0.7\n";
		Put<std::uint32_t>(bytes, 0x14020007);
		Put<std::uint8_t>(bytes, 1);
		Put<std::uint32_t>(bytes, 12);
		Put<std::uint32_t>(bytes, static_cast<std::uint32_t>(a_blocks.size()));
		Put<std::uint32_t>(bytes, 100);
		for (const char* text : { "creator", "export one", "" }) {
			bytes.push_back(static_cast<char>(std::strlen(text)));
			bytes += text;
		}
		Put<std::uint16_t>(bytes, static_cast<std::uint16_t>(types.size()));
		for (const std::string& type : types) {
			Put<std::uint32_t>(bytes, static_cast<std::uint32_t>(type.size()));
			bytes += type;
		}
		for (const std::uint16_t index : typeOf)
			Put(bytes, index);
		for (const Block& block : a_blocks)
			Put(bytes, block.size);
		Put<std::uint32_t>(bytes, 1);
		Put<std::uint32_t>(bytes, 4);
		Put<std::uint32_t>(bytes, 4);
		bytes += "Name";
		Put<std::uint32_t>(bytes, 0);
		for (const Block& block : a_blocks) {
			std::string body(block.size, '\x7f');
			if (block.type == "BSLightingShaderProperty" && block.size >= 4)
				std::memcpy(body.data(), &block.shaderType, sizeof(std::uint32_t));
			bytes += body;
		}
		return bytes;
	}
}

TEST_CASE("Lighting shaders are counted and hair tint ones told apart", "[smpphysics][nifshader]")
{
	const NifShaderHint hint = ScanNifShaders(MakeNif({ { "NiNode", 0, 40 }, { "BSTriShape", 0, 500 }, { "BSLightingShaderProperty", 6, 90 }, { "BSLightingShaderProperty", 0, 90 }, { "BSShaderTextureSet", 0, 30 } }));

	CHECK(hint.lightingShaders == 2);
	CHECK(hint.hairTintShaders == 1);
}

TEST_CASE("A model with no hair tint shader reports none", "[smpphysics][nifshader]")
{
	const NifShaderHint hint = ScanNifShaders(MakeNif({ { "BSLightingShaderProperty", 1, 60 }, { "BSLightingShaderProperty", 0, 60 } }));

	CHECK(hint.lightingShaders == 2);
	CHECK(hint.hairTintShaders == 0);
}

TEST_CASE("A file that is not a NIF or is cut short reports nothing", "[smpphysics][nifshader]")
{
	const std::string nif = MakeNif({ { "BSLightingShaderProperty", 6, 90 } });

	CHECK(ScanNifShaders("").lightingShaders == 0);
	CHECK(ScanNifShaders("no newline here").lightingShaders == 0);
	CHECK(ScanNifShaders(std::string_view(nif).substr(0, 60)).lightingShaders == 0);
	CHECK(ScanNifShaders(std::string_view(nif).substr(0, nif.size() - 88)).hairTintShaders == 0);
}

TEST_CASE("A block size smaller than a shader type does not run past the file", "[smpphysics][nifshader]")
{
	const NifShaderHint hint = ScanNifShaders(MakeNif({ { "BSLightingShaderProperty", 6, 2 } }));

	CHECK(hint.hairTintShaders == 0);
}
