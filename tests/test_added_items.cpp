#include "Utils/TestBench/AddedItems.h"

#include <catch2/catch_test_macros.hpp>

using TestBench::AddedItems;

TEST_CASE("An item the tool added is taken back once", "[testbench][addeditems]")
{
	AddedItems added;
	added.Note(0x14, 0x100);
	CHECK(added.Take(0x14, 0x100));
	CHECK_FALSE(added.Take(0x14, 0x100));
}

TEST_CASE("An item that was never noted is not taken back", "[testbench][addeditems]")
{
	AddedItems added;
	added.Note(0x14, 0x100);
	CHECK_FALSE(added.Take(0x14, 0x200));
	CHECK_FALSE(added.Take(0x15, 0x100));
}

TEST_CASE("Noting an item twice takes it back once", "[testbench][addeditems]")
{
	AddedItems added;
	added.Note(0x14, 0x100);
	added.Note(0x14, 0x100);
	CHECK(added.Take(0x14, 0x100));
	CHECK_FALSE(added.Take(0x14, 0x100));
}

TEST_CASE("Taking everything for one owner leaves the others", "[testbench][addeditems]")
{
	AddedItems added;
	added.Note(0x14, 0x100);
	added.Note(0x14, 0x200);
	added.Note(0x15, 0x300);
	CHECK(added.TakeAll(0x14) == std::vector<std::uint32_t>{ 0x100, 0x200 });
	CHECK(added.TakeAll(0x14).empty());
	CHECK(added.Take(0x15, 0x300));
}
