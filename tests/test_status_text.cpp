// Unit tests for the wording of the settings page's status block (src/ui/StatusText.h).

#include "Utils/ShaderFailure.h"
#include "ui/StatusText.h"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("A link the plugin accepted is shown capitalised", "[status_text]")
{
	CHECK(SettingsUi::LinkStatusText(true, "linked to hdtSMP64 plugin interface 2.2.0") == "Linked to hdtSMP64 plugin interface 2.2.0");
}

TEST_CASE("A refused link keeps the plugin's reason and says what to do", "[status_text]")
{
	const std::string text = SettingsUi::LinkStatusText(false, "hdtSMP64 plugin interface 2.1.0 is older than the required 2.2.0");
	CHECK(text.starts_with("hdtSMP64 plugin interface 2.1.0 is older than the required 2.2.0."));
	CHECK(text.find("restart the game") != std::string::npos);
}

TEST_CASE("No link message at all means hdtSMP64 was never seen", "[status_text]")
{
	CHECK(SettingsUi::LinkStatusText(false, "") == "hdtSMP64 was not found. Install it and load it before this plugin.");
}

TEST_CASE("The status block adds shader progress only while the startup compile runs", "[status_text]")
{
	const std::string idle = SettingsUi::StatusText(true, "linked to hdtSMP64 plugin interface 2.2.0", 12, 0.5, false, 0, 0);
	CHECK(idle.find("Managed physics files: 12") != std::string::npos);
	CHECK(idle.find("0.500 ms") != std::string::npos);
	CHECK(idle.find("Preparing shaders") == std::string::npos);

	const std::string busy = SettingsUi::StatusText(true, "linked to hdtSMP64 plugin interface 2.2.0", 0, 0.0, true, 3, 17);
	CHECK(busy.find("Preparing shaders: 3 of 17 done.") != std::string::npos);
}

TEST_CASE("Failed shaders become a heading with one line each, and none means no alert", "[status_text]")
{
	CHECK(Util::ShaderAlertLines({}).empty());
	const auto lines = Util::ShaderAlertLines({ "A.hlsl (Main): error X1", "B.hlsl (Main): error X2" });
	REQUIRE(lines.size() == 3);
	CHECK(lines[0].starts_with("Shader compilation failed."));
	CHECK(lines[0].find("Shaders folder") != std::string::npos);
	CHECK(lines[1] == "  - A.hlsl (Main): error X1");
	CHECK(lines[2] == "  - B.hlsl (Main): error X2");
}
