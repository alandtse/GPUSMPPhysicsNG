// Unit tests for the list filter's physics file matching (src/ui/ListSearch.h).

#include "ui/ListSearch.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace SettingsUi;

TEST_CASE("A list item is searched by its name and its physics file", "[ui][listsearch]")
{
	CHECK(SearchText("[BDOR Hair] - Stacy", "meshes\\armor\\[d] bdor\\hair\\com\\phw205andpkow25\\phw205andpkow25.xml").find("stacy") != std::string::npos);
	CHECK(SearchText("[BDOR Hair] - Stacy", "meshes\\armor\\[d] bdor\\hair\\com\\phw205andpkow25\\phw205andpkow25.xml").find("phw205") != std::string::npos);
	CHECK(SearchText("[BDOR Hair] - Stacy", "meshes\\armor\\a.xml").find("phw205") == std::string::npos);
	CHECK(SearchText("[BDOR Hair] - Stacy", "meshes\\armor\\[d] bdor\\hair\\com\\phw205andpkow25\\phw205andpkow25.xml").find(Util::SearchKey("COM/PHW205")) != std::string::npos);
}

TEST_CASE("A list item is searched by its plugin name", "[ui][listsearch]")
{
	CHECK(SearchText("Dress", "", "DD - Elanor Timeless Elegance.esp").find(Util::SearchKey("elanor")) != std::string::npos);
	CHECK(SearchText("Dress", "", "DD - Elanor Timeless Elegance.esp").find(Util::SearchKey(".esp")) != std::string::npos);
	CHECK(SearchText("Dress", "", "DD - Elanor Timeless Elegance.esp").find(Util::SearchKey("Heels Sound")) == std::string::npos);
	CHECK(SearchText("Dress", "").find(Util::SearchKey("elanor")) == std::string::npos);
	CHECK(SearchText("Dress", "meshes\\dress.xml", "").find(Util::SearchKey("elanor")) == std::string::npos);
	CHECK(SearchText("Dress", "meshes\\dress.xml") == SearchText("Dress", "meshes\\dress.xml", ""));
	CHECK(SearchText("Dress", "meshes\\dress.xml").find(Util::SearchKey("dress")) != std::string::npos);
	CHECK(SearchText("Dress", "meshes\\dress.xml").find(Util::SearchKey("meshes\\dress.xml")) != std::string::npos);
}

TEST_CASE("A scan's progress is clamped and zero without a size", "[ui][listsearch]")
{
	using Catch::Matchers::WithinAbs;
	CHECK_THAT(ScanFraction(50, 200), WithinAbs(0.25, 1e-6));
	CHECK_THAT(ScanFraction(300, 200), WithinAbs(1.0, 1e-6));
	CHECK_THAT(ScanFraction(5, 0), WithinAbs(0.0, 1e-6));
}
