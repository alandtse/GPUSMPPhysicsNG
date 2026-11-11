// Unit tests for the NIF header physics scan (src/smp/NifPhysicsScan.cpp). Pure bytes; no game dependency.

#include "smp/NifPhysicsScan.h"

#include <catch2/catch_test_macros.hpp>

using namespace SmpPhysics;

namespace
{
	std::string Entry(const std::string& a_text)
	{
		std::string bytes(4, '\0');
		bytes[0] = static_cast<char>(a_text.size() & 0xFF);
		bytes[1] = static_cast<char>((a_text.size() >> 8) & 0xFF);
		return bytes + a_text;
	}
}

TEST_CASE("A header naming the physics extra data and a path is read as having physics", "[smpphysics][nifscan]")
{
	const std::string bytes = std::string("Gamebryo File Format\n\x01\x02", 22) + Entry("Skirt") + Entry(std::string(kPhysicsExtraDataName)) +
	                          Entry("SKSE/Plugins/hdtSkinnedMeshConfigs/Dress.XML") + Entry("Skirt:0");

	const NifPhysicsHint hint = ScanNifHeader(bytes);

	CHECK(hint.hasExtraData);
	REQUIRE(hint.xmlPaths.size() == 1);
	CHECK(hint.xmlPaths[0] == "SKSE/Plugins/hdtSkinnedMeshConfigs/Dress.XML");
}

TEST_CASE("A header without the extra data name has no physics even with an xml string", "[smpphysics][nifscan]")
{
	const NifPhysicsHint hint = ScanNifHeader(Entry("Mesh") + Entry("something/else.xml"));

	CHECK_FALSE(hint.hasExtraData);
	CHECK(hint.xmlPaths.size() == 1);
}

TEST_CASE("Bytes that merely look like a string do not count and short input is safe", "[smpphysics][nifscan]")
{
	std::string bad = Entry(std::string(kPhysicsExtraDataName));
	bad[5] = '\x01';

	CHECK_FALSE(ScanNifHeader(bad).hasExtraData);
	CHECK_FALSE(ScanNifHeader("").hasExtraData);
	CHECK_FALSE(ScanNifHeader(std::string_view("\x05\0\0\0ab", 6)).hasExtraData);
}
