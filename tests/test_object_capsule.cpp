// Unit tests for turning a loose object's bound box into the capsule cloth collides with.

#include "Utils/ObjectCapsule.h"

#include <catch2/catch_test_macros.hpp>
#include <cmath>

using Util::ObjectCapsule::Fit;
using Util::ObjectCapsule::IsLoose;
using Util::ObjectCapsule::Motion;

TEST_CASE("Only free rigid bodies are cloth objects", "[havokobjects][capsule]")
{
	REQUIRE(IsLoose(Motion::kDynamic));
	REQUIRE(IsLoose(Motion::kSphereInertia));
	REQUIRE(IsLoose(Motion::kBoxInertia));
	REQUIRE(IsLoose(Motion::kThinBoxInertia));

	REQUIRE_FALSE(IsLoose(Motion::kInvalid));
	REQUIRE_FALSE(IsLoose(Motion::kFixed));
	REQUIRE_FALSE(IsLoose(Motion::kCharacter));
	// A door or a lever is keyframed, not fixed: the engine still moves it on a script's behalf.
	REQUIRE_FALSE(IsLoose(Motion::kKeyframed));
}

TEST_CASE("A door is a thin panel, not a blob", "[havokobjects][capsule]")
{
	// WRCastleDoor01.nif at 0x16707 in Dragonsreach: half extents 131 by 16 by 166.5, longest on z.
	const auto shape = Fit({ 131.0f, 16.0f, 166.5f }, 0.0f);
	REQUIRE(shape.axis == 2);
	REQUIRE(shape.radius == 16.0f);
	REQUIRE(shape.halfLength == 150.5f);
	// Taking the widest of the other axes instead gave a 131 radius, which reached through a cape 43 units away.
	REQUIRE(shape.radius < 131.0f);
}

TEST_CASE("A bound sphere caps the radius", "[havokobjects][capsule]")
{
	// The radius can never exceed the bounding sphere, so a box is not widened past what the engine already bounds. A bound
	// sphere larger than the box's thinnest axis does not widen it, because the radius is a cap and not a target.
	REQUIRE(Fit({ 131.0f, 16.0f, 166.5f }, 20.0f).radius == 16.0f);
	REQUIRE(Fit({ 131.0f, 16.0f, 166.5f }, 8.0f).radius == 8.0f);
	REQUIRE(Fit({ 131.0f, 16.0f, 166.5f }, 0.0f).radius == 16.0f);
}

TEST_CASE("The radius never falls below a floor", "[havokobjects][capsule]")
{
	// A coin is thinner than the solver's smallest useful proxy; a zero radius would divide it away.
	REQUIRE(Fit({ 0.4f, 0.01f, 0.05f }, 0.0f).radius == Util::ObjectCapsule::kMinRadius);
}

TEST_CASE("The capsule never overshoots the box it came from", "[havokobjects][capsule]")
{
	// Whatever the shape, the capsule stays inside the box's own extent along every axis: that is the invariant the door broke.
	const std::array<std::array<float, 3>, 6> boxes{ { { 1.0f, 1.0f, 1.0f }, { 131.0f, 16.0f, 166.5f }, { 176.0f, 103.0f, 1.0f },
		{ 8.0f, 300.0f, 8.0f }, { 0.4f, 0.01f, 0.05f }, { 70.0f, 35.0f, 12.0f } } };
	for (const auto& half : boxes) {
		const auto shape = Fit(half, 0.0f);
		// An object thinner than the floor becomes a capsule of the floor's size, which is deliberate; the box still bounds
		// every other case, and that is the invariant a door with a 131 radius broke.
		const float reach = std::max(half[static_cast<std::size_t>(shape.axis)], Util::ObjectCapsule::kMinRadius);
		REQUIRE(shape.halfLength + shape.radius <= reach + 1e-4f);
		for (std::size_t axis = 0; axis < 3; ++axis) {
			if (static_cast<int>(axis) == shape.axis)
				continue;
			REQUIRE(shape.radius <= std::max(half[axis], Util::ObjectCapsule::kMinRadius) + 1e-4f);
		}
	}
}
