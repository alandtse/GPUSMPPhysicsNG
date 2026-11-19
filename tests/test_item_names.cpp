// Unit tests for the item list names (src/Utils/ItemNames.h). Pure; no game dependency.

#include "Utils/ItemNames.h"

#include <catch2/catch_test_macros.hpp>

#include <array>

TEST_CASE("A name only one item has is shown as it is", "[items][names]")
{
	const std::array items{ Util::NamedItem{ "Iron Armor", 0x12 }, Util::NamedItem{ "Steel Armor", 0x13 } };
	CHECK(Util::UniqueNames(items) == std::vector<std::string>{ "Iron Armor", "Steel Armor" });
}

TEST_CASE("Items sharing a name get their form ID, in list order", "[items][names]")
{
	const std::array items{ Util::NamedItem{ "Hood", 0x1A }, Util::NamedItem{ "Cloak", 0x2 }, Util::NamedItem{ "Hood", 0x1B } };
	CHECK(Util::UniqueNames(items) == std::vector<std::string>{ "Hood [0000001A]", "Cloak", "Hood [0000001B]" });
}

TEST_CASE("No items gives no names", "[items][names]")
{
	CHECK(Util::UniqueNames({}).empty());
}
