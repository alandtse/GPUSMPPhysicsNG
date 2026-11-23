#include "smp/RollingWindow.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using SmpPhysics::RollingWindow;

TEST_CASE("A rolling window reports the mean and largest of its newest samples only", "[smpphysics]")
{
	RollingWindow<3> window;
	CHECK(window.Count() == 0);
	CHECK(window.Average() == 0.0f);
	CHECK(window.Max() == 0.0f);

	window.Add(2.0f);
	window.Add(4.0f);
	CHECK(window.Count() == 2);
	CHECK(window.Average() == Catch::Approx(3.0f));
	CHECK(window.Max() == Catch::Approx(4.0f));

	window.Add(9.0f);
	window.Add(1.0f);
	window.Add(1.0f);
	CHECK(window.Count() == 3);
	CHECK(window.Average() == Catch::Approx(11.0f / 3.0f));
	CHECK(window.Max() == Catch::Approx(9.0f));

	window.Add(1.0f);
	CHECK(window.Max() == Catch::Approx(1.0f));
}
