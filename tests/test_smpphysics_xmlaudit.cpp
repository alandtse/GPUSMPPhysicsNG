// Unit tests for the physics xml audit (src/smp/XmlAudit.cpp).

#include "smp/XmlAudit.h"

#include <catch2/catch_test_macros.hpp>

using namespace SmpPhysics;

TEST_CASE("Paths compare without regard to case, slash direction or a leading Data folder", "[smpphysics][xmlaudit]")
{
	CHECK(NormalizeXmlPath("Data/Meshes/A/B.XML") == "meshes\\a\\b.xml");
	CHECK(NormalizeXmlPath("meshes\\a\\b.xml") == "meshes\\a\\b.xml");
}

TEST_CASE("The default map's file attributes are read from each map entry", "[smpphysics][xmlaudit]")
{
	const auto files = ParseDefaultBbpFiles(
		"<default-bbps>\n<map shape=\"Cape\" file=\"SKSE/Plugins/hdtSkinnedMeshConfigs/cape.xml\"/>\n"
		"<map file='meshes/x/y.xml' shape=\"Y\"/>\n<mapping file=\"no.xml\"/>\n<map shape=\"Z\"/>\n</default-bbps>");

	REQUIRE(files.size() == 2);
	CHECK(files[0] == "SKSE/Plugins/hdtSkinnedMeshConfigs/cape.xml");
	CHECK(files[1] == "meshes/x/y.xml");
}

TEST_CASE("A file nothing names is an orphan and a name with no file is dangling", "[smpphysics][xmlaudit]")
{
	const XmlAudit audit = AuditXmlFiles({ "meshes/A/used.xml", "meshes/A/Lonely.xml" }, { "Meshes\\a\\USED.xml", "meshes\\gone.xml" });

	REQUIRE(audit.orphans.size() == 1);
	CHECK(audit.orphans[0] == "meshes/A/Lonely.xml");
	REQUIRE(audit.dangling.size() == 1);
	CHECK(audit.dangling[0] == "meshes\\gone.xml");
}

TEST_CASE("Empty input gives an empty audit and unterminated tags are safe", "[smpphysics][xmlaudit]")
{
	CHECK(AuditXmlFiles({}, {}).orphans.empty());
	CHECK(ParseDefaultBbpFiles("<map file=\"a").empty());
	CHECK(ParseDefaultBbpFiles("<map ").empty());
}

TEST_CASE("A search text matches a physics file path in any case with either slash", "[smpphysics][xmlaudit]")
{
	CHECK(XmlPathMatches("meshes\\armor\\[d] BDOr\\Hair\\COM\\PHW205andPKOW25\\PHW205andPKOW25.xml", "phw205andpkow25"));
	CHECK(XmlPathMatches("meshes\\armor\\[d] BDOr\\Hair\\COM\\PHW205andPKOW25\\PHW205andPKOW25.xml", "COM/phw205"));
	CHECK_FALSE(XmlPathMatches("meshes\\armor\\a.xml", "phw205"));
	CHECK_FALSE(XmlPathMatches("meshes\\armor\\a.xml", ""));
}
