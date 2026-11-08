// Unit tests for the strand grab's grip tracking and strand pick (src/smp/StrandGrab.cpp).

#include "smp/StrandGrab.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <vector>

using namespace SmpPhysics;
using Catch::Approx;

TEST_CASE("The pick takes each strand in reach, nearest first, with the nearest point of it", "[SmpPhysics]")
{
	const Vec3 palm{ 0.0f, 0.0f, 0.0f };
	const Vec3 tip{ 10.0f, 0.0f, 0.0f };
	const std::vector<GrabCandidate> candidates = {
		{ 7, 1, 3, { 4.0f, 4.0f, -10.0f }, { 4.0f, 4.0f, 10.0f } },    // 4 from the line
		{ 7, 2, 4, { 5.0f, 2.0f, -10.0f }, { 5.0f, 2.0f, 10.0f } },    // 2 from the line, crossing at its middle
		{ 9, 1, 1, { -3.0f, 0.0f, -2.0f }, { -3.0f, 0.0f, 2.0f } },    // 3 behind the palm
		{ 9, 2, 2, { 5.0f, 30.0f, -10.0f }, { 5.0f, 30.0f, 10.0f } },  // far
	};

	const auto picks = PickGrabCandidates(candidates, MakeGrabLine(palm, tip), kGrabReach, kMaxGrabPinsPerHand);
	REQUIRE(picks.size() == 3);
	REQUIRE(picks[0].index == 1);
	REQUIRE(picks[0].distance == Approx(2.0f));
	REQUIRE(picks[0].point.x == Approx(5.0f));
	REQUIRE(picks[0].point.y == Approx(2.0f));
	REQUIRE(picks[0].point.z == Approx(0.0f).margin(1e-4f));
	REQUIRE(picks[1].index == 2);
	REQUIRE(picks[2].index == 0);

	REQUIRE(PickGrabCandidates(candidates, MakeGrabLine(palm, tip), 1.0f, kMaxGrabPinsPerHand).empty());
	REQUIRE(PickGrabCandidates({}, MakeGrabLine(palm, tip), kGrabReach, kMaxGrabPinsPerHand).empty());
	REQUIRE(PickGrabCandidates(candidates, MakeGrabLine(palm, tip), kGrabReach, 0).empty());
}

TEST_CASE("The pick holds each strand once, by its nearest stretch", "[SmpPhysics]")
{
	const Vec3 palm{ 0.0f, 0.0f, 0.0f };
	const Vec3 tip{ 10.0f, 0.0f, 0.0f };
	const std::vector<GrabCandidate> candidates = {
		{ 5, 1, 10, { 5.0f, 3.0f, 0.0f }, { 5.0f, 3.0f, 20.0f } },    // strand 1 of claim 5, 3 away
		{ 5, 1, 11, { 5.0f, 1.0f, 0.0f }, { 5.0f, 1.0f, -20.0f } },   // the next stretch of it, 1 away
		{ 5, 2, 12, { 6.0f, -2.0f, 0.0f }, { 6.0f, -2.0f, 20.0f } },  // another strand of the claim
		{ 6, 1, 13, { 4.0f, 4.0f, 0.0f }, { 4.0f, 4.0f, 20.0f } },    // the same group number in another claim
	};
	const auto picks = PickGrabCandidates(candidates, MakeGrabLine(palm, tip), kGrabReach, kMaxGrabPinsPerHand);
	REQUIRE(picks.size() == 3);
	REQUIRE(picks[0].index == 1);
	REQUIRE(picks[1].index == 2);
	REQUIRE(picks[2].index == 3);
}

TEST_CASE("The pick catches a strand that the line only passes beside, and stops at the hand's limit", "[SmpPhysics]")
{
	std::vector<GrabCandidate> candidates = { { 1, 1, 0, { 5.0f, 3.0f, 0.0f }, { 5.0f, 3.0f, 40.0f } } };
	const auto beside = PickGrabCandidates(candidates, MakeGrabLine({ 0.0f, 0.0f, 0.0f }, { 10.0f, 0.0f, 0.0f }), kGrabReach, kMaxGrabPinsPerHand);
	REQUIRE(beside.size() == 1);
	REQUIRE(beside[0].point.z == Approx(0.0f).margin(1e-4f));
	REQUIRE(beside[0].distance == Approx(3.0f));

	candidates.clear();
	for (std::uint32_t strand = 0; strand < 12; ++strand)
		candidates.push_back({ 1, strand, static_cast<int>(strand), { 5.0f, 0.1f * static_cast<float>(strand), 0.0f }, { 5.0f, 0.1f * static_cast<float>(strand), 20.0f } });
	const auto capped = PickGrabCandidates(candidates, MakeGrabLine({ 0.0f, 0.0f, 0.0f }, { 10.0f, 0.0f, 0.0f }), kGrabReach, kMaxGrabPinsPerHand);
	REQUIRE(capped.size() == kMaxGrabPinsPerHand);
	REQUIRE(capped[0].index == 0);
	REQUIRE(capped[1].index == 1);
}

TEST_CASE("The pick breaks a tie by claim then bone, whatever the listing order", "[SmpPhysics]")
{
	const Vec3 palm{ 0.0f, 0.0f, 0.0f };
	const Vec3 tip{ 10.0f, 0.0f, 0.0f };
	const GrabCandidate a{ 5, 1, 2, { 5.0f, 2.0f, -5.0f }, { 5.0f, 2.0f, 5.0f } };
	const GrabCandidate b{ 5, 2, 1, { 5.0f, -2.0f, -5.0f }, { 5.0f, -2.0f, 5.0f } };
	const GrabCandidate c{ 4, 3, 9, { 5.0f, 0.0f, 2.0f }, { 5.0f, 0.0f, 12.0f } };

	const std::vector<GrabCandidate> forward = { a, b, c };
	const std::vector<GrabCandidate> backward = { c, b, a };
	const auto first = PickGrabCandidates(forward, MakeGrabLine(palm, tip), kGrabReach, 3);
	const auto second = PickGrabCandidates(backward, MakeGrabLine(palm, tip), kGrabReach, 3);
	REQUIRE(first.size() == 3);
	REQUIRE(second.size() == 3);
	REQUIRE(forward[first[0].index].claim == 4);
	REQUIRE(backward[second[0].index].claim == 4);
	REQUIRE(forward[first[1].index].bone == 1);
	REQUIRE(backward[second[1].index].bone == 1);
	REQUIRE(forward[first[2].index].bone == 2);
	REQUIRE(backward[second[2].index].bone == 2);
}

TEST_CASE("The pick ignores non-finite points and unusable bones", "[SmpPhysics]")
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	const std::vector<GrabCandidate> candidates = {
		{ 1, 1, 0, { nan, 0.0f, 0.0f }, { 5.0f, 0.0f, 5.0f } },
		{ 1, 2, -1, { 5.0f, 0.0f, -5.0f }, { 5.0f, 0.0f, 5.0f } },
	};
	REQUIRE(PickGrabCandidates(candidates, MakeGrabLine({ 0.0f, 0.0f, 0.0f }, { 10.0f, 0.0f, 0.0f }), kGrabReach, kMaxGrabPinsPerHand).empty());
	REQUIRE(PickGrabCandidates(candidates, MakeGrabLine({ nan, 0.0f, 0.0f }, { 10.0f, 0.0f, 0.0f }), kGrabReach, kMaxGrabPinsPerHand).empty());
}

TEST_CASE("HIGGS is disabled on a hand over hair and given back when it leaves", "[SmpPhysics]")
{
	REQUIRE(DecideHiggsHand(true, false, false) == HiggsHandAction::Disable);
	REQUIRE(DecideHiggsHand(true, false, true) == HiggsHandAction::None);
	REQUIRE(DecideHiggsHand(false, false, true) == HiggsHandAction::Enable);
	REQUIRE(DecideHiggsHand(false, false, false) == HiggsHandAction::None);
}

TEST_CASE("A hand HIGGS holds an object in is left alone, and handed back if it had been disabled", "[SmpPhysics]")
{
	REQUIRE(DecideHiggsHand(true, true, false) == HiggsHandAction::None);
	REQUIRE(DecideHiggsHand(true, true, true) == HiggsHandAction::Enable);
}

TEST_CASE("A grab pulses at once and a taut strand pulses at a throttled, tension-scaled rate", "[SmpPhysics]")
{
	REQUIRE(HapticPulseDuration(true, 0.0f, 0.0f) == Approx(kGrabHapticDuration));
	REQUIRE(HapticPulseDuration(false, 0.5f, 10.0f) == Approx(0.0f));
	REQUIRE(HapticPulseDuration(false, 4.0f, kHapticIntervalSeconds * 0.5f) == Approx(0.0f));
	REQUIRE(HapticPulseDuration(false, 4.0f, kHapticIntervalSeconds) == Approx(4.0f * kHapticDurationPerUnit));
	REQUIRE(HapticPulseDuration(false, 1.0f, 1.0f) == Approx(1.0f));
	REQUIRE(HapticPulseDuration(false, 1000.0f, 1.0f) == Approx(kHapticMaxDuration));
	REQUIRE(HapticPulseDuration(false, std::numeric_limits<float>::quiet_NaN(), 1.0f) == Approx(0.0f));
}

TEST_CASE("A held strand is dropped once it has stayed too far from the hand for long enough", "[SmpPhysics]")
{
	float seconds = 0.0f;
	REQUIRE_FALSE(ShouldDropGrab(kGrabDropDistance - 1.0f, 1.0f, seconds));
	REQUIRE(seconds == Approx(0.0f));
	REQUIRE_FALSE(ShouldDropGrab(kGrabDropDistance + 1.0f, kGrabDropSeconds * 0.4f, seconds));
	REQUIRE_FALSE(ShouldDropGrab(kGrabDropDistance + 1.0f, kGrabDropSeconds * 0.4f, seconds));
	REQUIRE(ShouldDropGrab(kGrabDropDistance + 1.0f, kGrabDropSeconds * 0.4f, seconds));
}

TEST_CASE("A strand that comes back in range starts its drop timer again", "[SmpPhysics]")
{
	float seconds = 0.0f;
	REQUIRE_FALSE(ShouldDropGrab(kGrabDropDistance + 5.0f, kGrabDropSeconds * 0.9f, seconds));
	REQUIRE_FALSE(ShouldDropGrab(kGrabDropDistance - 5.0f, 1.0f, seconds));
	REQUIRE(seconds == Approx(0.0f));
	REQUIRE_FALSE(ShouldDropGrab(kGrabDropDistance + 5.0f, kGrabDropSeconds * 0.9f, seconds));
	REQUIRE_FALSE(ShouldDropGrab(std::numeric_limits<float>::quiet_NaN(), 1.0f, seconds));
	REQUIRE(seconds == Approx(0.0f));
}

TEST_CASE("A strand across the palm cast is hit at the distance along the normal where it crosses", "[SmpPhysics]")
{
	const Vec3 palm{ 0.0f, 0.0f, 0.0f };
	const Vec3 normal{ 0.0f, 1.0f, 0.0f };
	const auto hit = PalmCastHit(palm, normal, { -5.0f, 4.0f, 0.0f }, { 5.0f, 4.0f, 0.0f });
	REQUIRE(hit.has_value());
	REQUIRE(*hit == Approx(4.0f));
}

TEST_CASE("A strand beside or beyond the palm cast is not hit", "[SmpPhysics]")
{
	const Vec3 palm{ 0.0f, 0.0f, 0.0f };
	const Vec3 normal{ 0.0f, 1.0f, 0.0f };
	REQUIRE_FALSE(PalmCastHit(palm, normal, { kHiggsSelectRadius + 1.0f, 0.0f, -5.0f }, { kHiggsSelectRadius + 1.0f, 0.0f, 5.0f }).has_value());
	REQUIRE_FALSE(PalmCastHit(palm, normal, { -5.0f, kHiggsSelectDistance + kHiggsSelectRadius + 1.0f, 0.0f }, { 5.0f, kHiggsSelectDistance + kHiggsSelectRadius + 1.0f, 0.0f }).has_value());
	REQUIRE_FALSE(PalmCastHit(palm, normal, { 0.0f, -10.0f, kHiggsSelectRadius + 2.0f }, { 0.0f, -5.0f, kHiggsSelectRadius + 2.0f }).has_value());
}

TEST_CASE("A thick target is hit from farther to the side", "[SmpPhysics]")
{
	const Vec3 palm{ 0.0f, 0.0f, 0.0f };
	const Vec3 normal{ 0.0f, 1.0f, 0.0f };
	const Vec3 start{ kHiggsSelectRadius + 3.0f, 3.0f, -5.0f };
	const Vec3 end{ kHiggsSelectRadius + 3.0f, 3.0f, 5.0f };
	REQUIRE_FALSE(PalmCastHit(palm, normal, start, end).has_value());
	REQUIRE(PalmCastHit(palm, normal, start, end, 3.5f).has_value());
}

TEST_CASE("The palm is pointed at hair unless a body part is clearly nearer", "[SmpPhysics]")
{
	REQUIRE(DecidePalmTarget(std::nullopt, std::nullopt) == PalmTarget::None);
	REQUIRE(DecidePalmTarget(std::nullopt, 3.0f) == PalmTarget::None);
	REQUIRE(DecidePalmTarget(4.0f, std::nullopt) == PalmTarget::Hair);
	REQUIRE(DecidePalmTarget(4.0f, 2.0f) == PalmTarget::Hair);
	REQUIRE(DecidePalmTarget(4.0f, 4.0f + kHairOverBodyMargin) == PalmTarget::Hair);
	REQUIRE(DecidePalmTarget(5.0f, 5.0f - kHairOverBodyMargin - 0.5f) == PalmTarget::Other);
	REQUIRE(DecidePalmTarget(6.0f, 0.0f) == PalmTarget::Other);
}

TEST_CASE("A hand catches hair in the curl of its fingers and along them, and not hair behind the palm", "[SmpPhysics]")
{
	// Palm at the origin facing +Y, fingers along +Z.
	const GrabHandShape hand = MakeHandGrabShape({ 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f });
	const std::vector<GrabCandidate> candidates = {
		{ 1, 1, 0, { -10.0f, 2.0f, 3.0f }, { 10.0f, 2.0f, 3.0f } },    // across the fingers, inside the curl
		{ 1, 2, 1, { -10.0f, 1.0f, 6.0f }, { 10.0f, 1.0f, 6.0f } },    // across the fingertips
		{ 1, 3, 2, { -10.0f, -4.0f, 3.0f }, { 10.0f, -4.0f, 3.0f } },  // behind the palm, 4 away
		{ 1, 4, 3, { -10.0f, 4.0f, 3.0f }, { 10.0f, 4.0f, 3.0f } },    // in front of the palm, 4 away
	};
	const auto picks = PickGrabCandidates(candidates, hand, kGrabReach, kMaxGrabPinsPerHand);
	REQUIRE(picks.size() == 3);
	REQUIRE(picks[0].index == 1);
	REQUIRE(picks[1].index == 0);
	REQUIRE(picks[2].index == 3);
	for (const GrabPick& pick : picks)
		REQUIRE(pick.index != 2);
}

TEST_CASE("Hair that clips the fingers counts, and hair behind the hand does not", "[SmpPhysics]")
{
	const GrabHandShape hand = MakeHandGrabShape({ 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f });
	const auto picked = [&](float a_y, float a_z) {
		const std::vector<GrabCandidate> candidates = { { 1, 1, 0, { -10.0f, a_y, a_z }, { 10.0f, a_y, a_z } } };
		return PickGrabCandidates(candidates, hand, kGrabReach, kMaxGrabPinsPerHand).size();
	};
	REQUIRE(picked(0.0f, 3.0f) == 1);
	REQUIRE(picked(-kGrabFingerThickness + 0.1f, 3.0f) == 1);
	REQUIRE(picked(-kGrabFingerThickness - 0.1f, 3.0f) == 0);
	REQUIRE(picked(-2.0f, 3.0f) == 0);
}

TEST_CASE("Hair past the fingertips or behind the wrist end does not count", "[SmpPhysics]")
{
	const GrabHandShape hand = MakeHandGrabShape({ 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f });
	const auto picked = [&](float a_z) {
		const std::vector<GrabCandidate> candidates = { { 1, 1, 0, { -10.0f, 1.0f, a_z }, { 10.0f, 1.0f, a_z } } };
		return PickGrabCandidates(candidates, hand, kGrabReach, kMaxGrabPinsPerHand).size();
	};
	REQUIRE(picked(kGrabFingerLength) == 1);
	REQUIRE(picked(kGrabFingerLength + kGrabTipSlack - 0.1f) == 1);
	REQUIRE(picked(kGrabFingerLength + kGrabTipSlack + 0.1f) == 0);
	REQUIRE(picked(-kGrabWristSlack + 0.1f) == 1);
	REQUIRE(picked(-kGrabWristSlack - 0.1f) == 0);
}

TEST_CASE("A strand that runs from behind the hand to its palm side is caught by the part on the palm side", "[SmpPhysics]")
{
	const GrabHandShape hand = MakeHandGrabShape({ 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f });
	const std::vector<GrabCandidate> candidates = { { 1, 1, 0, { 0.0f, -8.0f, 3.0f }, { 0.0f, 3.0f, 3.0f } } };
	const auto picks = PickGrabCandidates(candidates, hand, kGrabReach, kMaxGrabPinsPerHand);
	REQUIRE(picks.size() == 1);
	REQUIRE(picks[0].point.y >= -kGrabFingerThickness - 1e-4f);
}

TEST_CASE("A hand shape with a bad point or no segments picks nothing", "[SmpPhysics]")
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	const std::vector<GrabCandidate> candidates = { { 1, 1, 0, { -10.0f, 1.0f, 3.0f }, { 10.0f, 1.0f, 3.0f } } };
	REQUIRE(PickGrabCandidates(candidates, MakeHandGrabShape({ nan, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }), kGrabReach, kMaxGrabPinsPerHand).empty());
	REQUIRE(PickGrabCandidates(candidates, GrabHandShape{}, kGrabReach, kMaxGrabPinsPerHand).empty());
}

TEST_CASE("A strand the hand pulls past its own length is dropped after a short time", "[SmpPhysics]")
{
	float seconds = 0.0f;
	REQUIRE_FALSE(ShouldDropOverreach(kGrabOverreachSlack - 1.0f, 1.0f, seconds));
	REQUIRE_FALSE(ShouldDropOverreach(kGrabOverreachSlack + 1.0f, kGrabOverreachSeconds * 0.6f, seconds));
	REQUIRE(ShouldDropOverreach(kGrabOverreachSlack + 1.0f, kGrabOverreachSeconds * 0.6f, seconds));

	seconds = 0.0f;
	REQUIRE_FALSE(ShouldDropOverreach(kGrabOverreachSlack + 5.0f, kGrabOverreachSeconds * 0.9f, seconds));
	REQUIRE_FALSE(ShouldDropOverreach(0.0f, 1.0f, seconds));
	REQUIRE_FALSE(ShouldDropOverreach(kGrabOverreachSlack + 5.0f, kGrabOverreachSeconds * 0.9f, seconds));
	REQUIRE_FALSE(ShouldDropOverreach(std::numeric_limits<float>::quiet_NaN(), 1.0f, seconds));
}
