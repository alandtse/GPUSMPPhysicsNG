#include "ui/ListView.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

using SettingsUi::RowVisible;
using SettingsUi::ShouldScroll;
using SettingsUi::Sort;
using SettingsUi::SortedOrder;
using SettingsUi::StepVisible;

namespace
{
	const std::vector<std::string> kNames{ "alpha braid", "beta", "gamma braid", "delta" };
}

TEST_CASE("A row is visible under the filter and the Show mode", "[ui][listview]")
{
	CHECK(RowVisible("alpha braid", "braid", 0, 0));
	CHECK_FALSE(RowVisible("beta", "braid", 0, 0));
	CHECK_FALSE(RowVisible("beta", "", 1, 0));
	CHECK(RowVisible("beta", "", 1, 1));
	CHECK_FALSE(RowVisible("beta", "", 2, 1));
	CHECK(RowVisible("beta", "", 2, 3));
}

TEST_CASE("Stepping walks the visible rows and wraps at both ends", "[ui][listview]")
{
	const std::vector<int> states;
	CHECK(StepVisible(kNames, "", 0, states, 0, 1) == 1);
	CHECK(StepVisible(kNames, "", 0, states, 3, 1) == 0);
	CHECK(StepVisible(kNames, "", 0, states, 0, -1) == 3);
}

TEST_CASE("Stepping skips rows the filter or the Show mode hides", "[ui][listview]")
{
	const std::vector<int> states;
	CHECK(StepVisible(kNames, "braid", 0, states, 0, 1) == 2);
	CHECK(StepVisible(kNames, "braid", 0, states, 2, 1) == 0);
	const std::vector<int> attention{ 0, 3, 0, 2 };
	CHECK(StepVisible(kNames, "", 2, attention, 1, 1) == 3);
	CHECK(StepVisible(kNames, "", 2, attention, 3, -1) == 1);
}

TEST_CASE("Stepping from a hidden or unselected row lands on the first or last visible one", "[ui][listview]")
{
	const std::vector<int> states;
	CHECK(StepVisible(kNames, "braid", 0, states, 1, 1) == 2);
	CHECK(StepVisible(kNames, "braid", 0, states, -1, 1) == 0);
	CHECK(StepVisible(kNames, "braid", 0, states, -1, -1) == 2);
}

TEST_CASE("Stepping stays put when no row is visible", "[ui][listview]")
{
	const std::vector<int> states;
	CHECK(StepVisible(kNames, "zzz", 0, states, 1, 1) == 1);
	CHECK(StepVisible({}, "", 0, states, -1, 1) == -1);
}

TEST_CASE("Stepping follows the order the list shows its rows in", "[ui][listview]")
{
	const std::vector<int> states;
	const std::vector<int> order{ 2, 0, 3, 1 };
	CHECK(StepVisible(kNames, "", 0, states, 2, 1, order) == 0);
	CHECK(StepVisible(kNames, "", 0, states, 0, 1, order) == 3);
	CHECK(StepVisible(kNames, "", 0, states, 1, 1, order) == 2);
	CHECK(StepVisible(kNames, "", 0, states, 2, -1, order) == 1);
	CHECK(StepVisible(kNames, "braid", 0, states, 2, 1, order) == 0);
	CHECK(StepVisible(kNames, "", 0, states, -1, 1, order) == 2);
	CHECK(StepVisible(kNames, "", 0, states, 0, 1, std::vector<int>{ 1, 0 }) == 1);
}

TEST_CASE("A list scrolls to a changed selection except one the user just clicked", "[ui][listview]")
{
	CHECK(ShouldScroll(5, 2, -1));
	CHECK_FALSE(ShouldScroll(5, 2, 5));
	CHECK_FALSE(ShouldScroll(2, 2, -1));
	CHECK(ShouldScroll(7, 2, 5));
}

TEST_CASE("The natural sort leaves a list as it is", "[ui][listview]")
{
	CHECK(SortedOrder(Sort::Natural, kNames, {}).empty());
}

TEST_CASE("Sorting by name ignores case and runs either way", "[ui][listview]")
{
	const std::vector<std::string> names{ "beta", "Alpha", "delta", "Gamma" };

	CHECK(SortedOrder(Sort::NameAscending, names, {}) == std::vector<int>{ 1, 0, 2, 3 });
	CHECK(SortedOrder(Sort::NameDescending, names, {}) == std::vector<int>{ 3, 2, 0, 1 });
}

TEST_CASE("Sorting by physics health puts the worst or the ones with physics first and keeps ties in order", "[ui][listview]")
{
	const std::vector<int> states{ 0, 1, 3, 2, 0, 3 };
	const std::vector<std::string> names{ "a", "b", "c", "d", "e", "f" };

	CHECK(SortedOrder(Sort::WorstFirst, names, states) == std::vector<int>{ 2, 5, 3, 1, 0, 4 });
	CHECK(SortedOrder(Sort::HasPhysicsFirst, names, states) == std::vector<int>{ 1, 2, 3, 5, 0, 4 });
	CHECK(SortedOrder(Sort::WorstFirst, names, std::vector<int>{ 0, 2 }) == std::vector<int>{ 1, 0, 2, 3, 4, 5 });
}
