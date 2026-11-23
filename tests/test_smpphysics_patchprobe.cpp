#include "smp/PatchProbe.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>

using SmpPhysics::MeasureSkinOutsideCloth;
using SmpPhysics::Vec3;

namespace
{
	const std::array<Vec3, 3> kCloth{ { { 0, 0, 0 }, { 10, 0, 0 }, { 0, 10, 0 } } };
	const std::array<std::uint32_t, 3> kFace{ 0, 1, 2 };
}

TEST_CASE("A skin point behind the cloth is not outside it", "[patchprobe]")
{
	const std::array<Vec3, 1> skin{ { { 2, 2, -1.0f } } };
	const auto sample = MeasureSkinOutsideCloth(skin, kCloth, kFace, 1.0f, 5.0f, 0.05f);
	CHECK(sample.measured == 1);
	CHECK(sample.outside == 0);
}

TEST_CASE("A skin point in front of the cloth is counted with its distance", "[patchprobe]")
{
	const std::array<Vec3, 2> skin{ { { 2, 2, 0.5f }, { 3, 3, -2.0f } } };
	const auto sample = MeasureSkinOutsideCloth(skin, kCloth, kFace, 1.0f, 5.0f, 0.05f);
	CHECK(sample.measured == 2);
	CHECK(sample.outside == 1);
	CHECK_THAT(sample.maxOutside, Catch::Matchers::WithinAbs(0.5f, 1e-5f));
	CHECK(sample.worst == 0);
	CHECK(sample.worstFace == 0);
}

TEST_CASE("Flipping the orientation swaps which side is outside", "[patchprobe]")
{
	const std::array<Vec3, 1> skin{ { { 2, 2, 0.5f } } };
	CHECK(MeasureSkinOutsideCloth(skin, kCloth, kFace, -1.0f, 5.0f, 0.05f).outside == 0);
}

TEST_CASE("A point farther than the reach from every face is not measured", "[patchprobe]")
{
	const std::array<Vec3, 1> skin{ { { 2, 2, 20.0f } } };
	const auto sample = MeasureSkinOutsideCloth(skin, kCloth, kFace, 1.0f, 5.0f, 0.05f);
	CHECK(sample.measured == 0);
	CHECK(sample.outside == 0);
}

TEST_CASE("A point past a face edge is measured to the nearest edge point", "[patchprobe]")
{
	const std::array<Vec3, 1> skin{ { { 12, 0, 1.0f } } };
	const auto sample = MeasureSkinOutsideCloth(skin, kCloth, kFace, 1.0f, 5.0f, 0.05f);
	CHECK(sample.measured == 1);
}

TEST_CASE("The nearest of several faces decides, across cell borders and negative space", "[patchprobe]")
{
	const std::array<Vec3, 6> cloth{ { { -10, -10, 0 }, { 0, -10, 0 }, { -10, 0, 0 }, { -10, -10, -3 }, { 0, -10, -3 }, { -10, 0, -3 } } };
	const std::array<std::uint32_t, 6> faces{ 0, 1, 2, 3, 4, 5 };
	const std::array<Vec3, 2> skin{ { { -6.0f, -6.0f, 0.4f }, { -6.0f, -6.0f, -1.0f } } };
	const auto sample = MeasureSkinOutsideCloth(skin, cloth, faces, 1.0f, 6.0f, 0.05f);
	CHECK(sample.measured == 2);
	CHECK(sample.outside >= 1);
	CHECK(sample.worst == 0);
}
