// Unit tests for the dependency-free helpers in src/Utils and the solver's step clock and parameter sanitising.

#include "Utils/FrameLatch.h"
#include "Utils/MathUtils.h"
#include "Utils/SkinVertexBlock.h"
#include "Utils/StringUtils.h"
#include "smp/SolverCommon.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <limits>

using Catch::Approx;

TEST_CASE("FrameLatch advances once per frame", "[utils]")
{
	Util::FrameLatch latch;
	CHECK(latch.TryAdvance(7));
	CHECK_FALSE(latch.TryAdvance(7));
	CHECK(latch.TryAdvance(8));
	latch.Reset();
	CHECK(latch.TryAdvance(8));
}

TEST_CASE("String helpers ignore ASCII case", "[utils]")
{
	CHECK(Util::ToLowerAscii("MiXed/Path.NIF") == "mixed/path.nif");
	CHECK(Util::IEquals("Cloak", "cLOAK"));
	CHECK_FALSE(Util::IEquals("Cloak", "Cloaks"));
	CHECK(Util::IEquals("", ""));
}

TEST_CASE("ClampFinite replaces non-finite values", "[utils]")
{
	CHECK(Util::ClampFinite(5.0f, 0.0f, 3.0f, -1.0f) == 3.0f);
	CHECK(Util::ClampFinite(-5.0f, 0.0f, 3.0f, -1.0f) == 0.0f);
	CHECK(Util::ClampFinite(2.0f, 0.0f, 3.0f, -1.0f) == 2.0f);
	CHECK(Util::ClampFinite(std::numeric_limits<float>::quiet_NaN(), 0.0f, 3.0f, -1.0f) == -1.0f);
	CHECK(Util::ClampFinite(std::numeric_limits<float>::infinity(), 0.0f, 3.0f, -1.0f) == -1.0f);
}

TEST_CASE("Fnv1a64 matches the reference vectors and chains", "[utils]")
{
	CHECK(Util::Fnv1a64("") == Util::kFnv1a64Basis);
	CHECK(Util::Fnv1a64("a") == 0xaf63dc4c8601ec8cull);
	CHECK(Util::Fnv1a64("foobar") == 0x85944171f73967e8ull);
	CHECK(Util::Fnv1a64("bar", Util::Fnv1a64("foo")) == Util::Fnv1a64("foobar"));
}

TEST_CASE("HalfToFloat decodes halves and rejects infinity and NaN", "[utils]")
{
	CHECK(Util::HalfToFloat(0x0000) == 0.0f);
	CHECK(Util::HalfToFloat(0x3C00) == 1.0f);
	CHECK(Util::HalfToFloat(0xBC00) == -1.0f);
	CHECK(Util::HalfToFloat(0x3800) == 0.5f);
	CHECK(Util::HalfToFloat(0x0001) == Approx(std::ldexp(1.0f, -24)));
	CHECK(Util::HalfToFloat(0x7C00) == 0.0f);
	CHECK(Util::HalfToFloat(0x7E00) == 0.0f);
}

TEST_CASE("DominantSkinBone picks the strongest valid influence", "[utils]")
{
	using Bones = std::array<int, 4>;
	using Weights = std::array<float, 4>;
	const float nan = std::numeric_limits<float>::quiet_NaN();

	CHECK(Util::DominantSkinBone(Bones{ 3, 1, 0, 0 }, Weights{ 0.6f, 0.4f, 0.0f, 0.0f }, 8) == 3u);
	CHECK_FALSE(Util::DominantSkinBone(Bones{ 3, 1, 0, 0 }, Weights{ 0.4f, 0.4f, 0.1f, 0.1f }, 8).has_value());
	CHECK(Util::DominantSkinBone(Bones{ 5, 2, 0, 0 }, Weights{ 0.5f, 0.5f, 0.0f, 0.0f }, 8) == 2u);
	CHECK(Util::DominantSkinBone(Bones{ -1, 9, 4, 0 }, Weights{ 0.9f, 0.9f, 0.5f, 0.0f }, 8) == 4u);
	CHECK_FALSE(Util::DominantSkinBone(Bones{ 1, 0, 0, 0 }, Weights{ nan, 0.0f, 0.0f, 0.0f }, 8).has_value());
}

TEST_CASE("ReadSkinVertexBlock reads weights then palette slots", "[utils]")
{
	const std::uint8_t block[Util::kSkinVertexBlockBytes] = { 0x00, 0x3C, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 9, 8, 7, 6 };
	const Util::SkinVertexBlock vertex = Util::ReadSkinVertexBlock(block);
	CHECK(vertex.weight[0] == 1.0f);
	CHECK(vertex.weight[1] == 0.5f);
	CHECK(vertex.weight[2] == 0.0f);
	CHECK(vertex.paletteSlot == std::array<std::uint8_t, 4>{ 9, 8, 7, 6 });
}

TEST_CASE("SmpStepClock skips short frames and caps long ones", "[solver]")
{
	SmpPhysics::SmpStepClock clock;
	const float tick = 1.0f / 60.0f;

	CHECK(clock.Advance(0.0f).count == 0);
	CHECK(clock.Advance(std::numeric_limits<float>::quiet_NaN()).count == 0);
	CHECK(clock.Advance(tick * 0.4f).count == 0);

	const auto accumulated = clock.Advance(tick * 0.4f);
	CHECK(accumulated.count >= 1);
	CHECK(accumulated.total == Approx(tick * 0.8f));

	clock.Reset();
	const auto stall = clock.Advance(5.0f);
	CHECK(stall.total <= tick * 4.0f + 1e-6f);
	CHECK(stall.count <= 5);
	float sum = 0.0f;
	for (int index = 0; index < stall.count; ++index)
		sum += stall.steps[static_cast<std::size_t>(index)];
	CHECK(sum == Approx(stall.total).margin(1.0f / 300.0f));
}

TEST_CASE("SanitizeParams repairs every non-finite or out-of-range value", "[solver]")
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	SmpPhysics::SolverParams params;
	params.gravity = nan;
	params.gravityScale = std::numeric_limits<float>::infinity();
	params.bodyScale = -2.0f;
	params.fixedStep = 0.0f;
	params.maxFrameTime = -1.0f;
	params.maxAngularSpeed = nan;
	params.maxSubSteps = 1000;
	params.windFactor = nan;
	params.wind = { 1.0f, 2.0f, 3.0f };

	const auto clean = SmpPhysics::SanitizeParams(params);
	CHECK(clean.gravity == 0.0f);
	CHECK(clean.gravityScale == 0.0f);
	CHECK(clean.bodyScale == 1.0f);
	CHECK(clean.fixedStep == Approx(1.0f / 60.0f));
	CHECK(clean.maxFrameTime == 0.0f);
	CHECK(clean.maxAngularSpeed == 0.0f);
	CHECK(clean.maxSubSteps == 64);
	CHECK(clean.windFactor == 0.0f);
	CHECK(clean.wind.x == 0.0f);

	params = {};
	params.maxSubSteps = 0;
	CHECK(SmpPhysics::SanitizeParams(params).maxSubSteps == 1);
	CHECK(SmpPhysics::SanitizeParams(SmpPhysics::SolverParams{}).gravity == SmpPhysics::SolverParams{}.gravity);
}
