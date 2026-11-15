#include "StrandGrab.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace SmpPhysics
{
	HiggsHandAction DecideHiggsHand(bool a_onHair, bool a_holdingObject, bool a_disabled) noexcept
	{
		const bool wantDisabled = a_onHair && !a_holdingObject;
		if (wantDisabled == a_disabled)
			return HiggsHandAction::None;
		return wantDisabled ? HiggsHandAction::Disable : HiggsHandAction::Enable;
	}

	bool ShouldDropGrab(float a_tension, float a_frameSeconds, float& a_seconds) noexcept
	{
		if (!(a_tension > kGrabDropDistance)) {
			a_seconds = 0.0f;
			return false;
		}
		a_seconds += a_frameSeconds;
		return a_seconds >= kGrabDropSeconds;
	}

	bool ShouldDropOverreach(float a_overreach, float a_frameSeconds, float& a_seconds) noexcept
	{
		if (!(a_overreach > kGrabOverreachSlack)) {
			a_seconds = 0.0f;
			return false;
		}
		a_seconds += a_frameSeconds;
		return a_seconds >= kGrabOverreachSeconds;
	}

	float HapticPulseDuration(bool a_justGrabbed, float a_tension, float a_secondsSinceLastPulse) noexcept
	{
		if (a_justGrabbed)
			return kGrabHapticDuration;
		if (!(a_tension >= kHapticMinimumTension) || a_secondsSinceLastPulse < kHapticIntervalSeconds)
			return 0.0f;
		return std::clamp(a_tension * kHapticDurationPerUnit, 1.0f, kHapticMaxDuration);
	}

	namespace
	{
		constexpr float kParallelEpsilon = 1.0e-8f;

		struct SegmentPair
		{
			Vec3 onFirst{};
			Vec3 onSecond{};
			float distance = 0.0f;
		};

		/** @brief The closest points of two segments, after Ericson's Real-Time Collision Detection, 5.1.9. */
		SegmentPair ClosestPoints(const Vec3& a_p1, const Vec3& a_q1, const Vec3& a_p2, const Vec3& a_q2) noexcept
		{
			const Vec3 d1 = a_q1 - a_p1;
			const Vec3 d2 = a_q2 - a_p2;
			const Vec3 r = a_p1 - a_p2;
			const float a = Dot(d1, d1);
			const float e = Dot(d2, d2);
			const float f = Dot(d2, r);
			float s = 0.0f;
			float t = 0.0f;
			if (a <= kParallelEpsilon && e <= kParallelEpsilon) {
				s = t = 0.0f;
			} else if (a <= kParallelEpsilon) {
				t = std::clamp(f / e, 0.0f, 1.0f);
			} else {
				const float c = Dot(d1, r);
				if (e <= kParallelEpsilon) {
					s = std::clamp(-c / a, 0.0f, 1.0f);
				} else {
					const float b = Dot(d1, d2);
					const float denominator = a * e - b * b;
					s = denominator > kParallelEpsilon ? std::clamp((b * f - c * e) / denominator, 0.0f, 1.0f) : 0.0f;
					t = (b * s + f) / e;
					if (t < 0.0f) {
						t = 0.0f;
						s = std::clamp(-c / a, 0.0f, 1.0f);
					} else if (t > 1.0f) {
						t = 1.0f;
						s = std::clamp((b - c) / a, 0.0f, 1.0f);
					}
				}
			}
			SegmentPair result;
			result.onFirst = a_p1 + d1 * s;
			result.onSecond = a_p2 + d2 * t;
			result.distance = Length(result.onFirst - result.onSecond);
			return result;
		}
	}

	std::optional<float> PalmCastHit(const Vec3& a_palm, const Vec3& a_normal, const Vec3& a_start, const Vec3& a_end, float a_extraRadius) noexcept
	{
		const Vec3 axisEnd = a_palm + a_normal * kHiggsSelectDistance;
		const SegmentPair closest = ClosestPoints(a_start, a_end, a_palm, axisEnd);
		if (!std::isfinite(closest.distance) || closest.distance > kHiggsSelectRadius + a_extraRadius)
			return std::nullopt;
		return Length(closest.onSecond - a_palm);
	}

	PalmTarget DecidePalmTarget(std::optional<float> a_hairAlong, std::optional<float> a_otherAlong) noexcept
	{
		if (!a_hairAlong)
			return PalmTarget::None;
		if (a_otherAlong && *a_hairAlong > *a_otherAlong + kHairOverBodyMargin)
			return PalmTarget::Other;
		return PalmTarget::Hair;
	}

	GrabHandShape MakeHandGrabShape(const Vec3& a_palm, const Vec3& a_normal, const Vec3& a_fingerDirection) noexcept
	{
		GrabHandShape shape;
		shape.segments[0] = { a_palm, a_palm + a_normal * kGrabCurlDepth };
		const Vec3 fingerStart = a_palm + a_normal * kGrabFingerLift;
		shape.segments[1] = { fingerStart, fingerStart + a_fingerDirection * kGrabFingerLength };
		shape.segmentCount = 2;
		shape.palm = a_palm;
		shape.normal = a_normal;
		shape.fingerDirection = a_fingerDirection;
		shape.palmSideOnly = true;
		return shape;
	}

	namespace
	{
		/** @brief Cuts the parameter range [a_low, a_high] of a_start to a_end to where Dot(point - a_palm, a_axis) is at least a_bound; false when none is left. */
		bool ClipToHalfSpace(const Vec3& a_start, const Vec3& a_end, const Vec3& a_palm, const Vec3& a_axis, float a_bound, float& a_low, float& a_high) noexcept
		{
			const float atStart = Dot(a_start - a_palm, a_axis) - a_bound;
			const float atEnd = Dot(a_end - a_palm, a_axis) - a_bound;
			if (atStart < 0.0f && atEnd < 0.0f)
				return false;
			if (atStart < 0.0f)
				a_low = std::max(a_low, atStart / (atStart - atEnd));
			else if (atEnd < 0.0f)
				a_high = std::min(a_high, atStart / (atStart - atEnd));
			return a_low <= a_high;
		}

		/** @brief The nearest point of the part of a strand the hand can reach to the hand's shape, with its distance; infinite when none of it can be reached. */
		SegmentPair NearestToHand(const GrabHandShape& a_hand, const Vec3& a_start, const Vec3& a_end) noexcept
		{
			SegmentPair best;
			best.distance = std::numeric_limits<float>::infinity();
			Vec3 start = a_start;
			Vec3 end = a_end;
			if (a_hand.palmSideOnly) {
				float low = 0.0f;
				float high = 1.0f;
				const bool reachable = ClipToHalfSpace(a_start, a_end, a_hand.palm, a_hand.normal, -kGrabFingerThickness, low, high) &&
				                       ClipToHalfSpace(a_start, a_end, a_hand.palm, a_hand.fingerDirection, -kGrabWristSlack, low, high) &&
				                       ClipToHalfSpace(a_start, a_end, a_hand.palm, a_hand.fingerDirection * -1.0f, -(kGrabFingerLength + kGrabTipSlack), low, high);
				if (!reachable)
					return best;
				start = a_start + (a_end - a_start) * low;
				end = a_start + (a_end - a_start) * high;
			}
			for (std::size_t index = 0; index < a_hand.segmentCount; ++index) {
				const SegmentPair closest = ClosestPoints(start, end, a_hand.segments[index][0], a_hand.segments[index][1]);
				if (closest.distance < best.distance)
					best = closest;
			}
			return best;
		}
	}

	std::vector<GrabPick> PickGrabCandidates(std::span<const GrabCandidate> a_candidates, const GrabHandShape& a_hand, float a_reach, std::size_t a_maxCount)
	{
		std::vector<GrabPick> picks;
		bool finite = a_hand.segmentCount > 0 && a_hand.segmentCount <= a_hand.segments.size();
		for (std::size_t index = 0; finite && index < a_hand.segmentCount; ++index)
			finite = IsFiniteVector(a_hand.segments[index][0]) && IsFiniteVector(a_hand.segments[index][1]);
		if (!finite || a_maxCount == 0)
			return picks;
		const auto before = [&a_candidates](const GrabPick& a_left, const GrabPick& a_right) {
			if (a_left.distance != a_right.distance)
				return a_left.distance < a_right.distance;
			const GrabCandidate& left = a_candidates[a_left.index];
			const GrabCandidate& right = a_candidates[a_right.index];
			if (left.claim != right.claim)
				return left.claim < right.claim;
			return left.bone < right.bone;
		};
		for (std::size_t index = 0; index < a_candidates.size(); ++index) {
			const GrabCandidate& candidate = a_candidates[index];
			if (candidate.bone < 0 || !IsFiniteVector(candidate.start) || !IsFiniteVector(candidate.end))
				continue;
			const SegmentPair closest = NearestToHand(a_hand, candidate.start, candidate.end);
			if (!(closest.distance <= a_reach))
				continue;
			const GrabPick pick{ index, closest.onFirst, closest.distance };
			const auto sameStrand = std::ranges::find_if(picks, [&](const GrabPick& a_other) {
				const GrabCandidate& other = a_candidates[a_other.index];
				return other.claim == candidate.claim && other.group == candidate.group;
			});
			if (sameStrand == picks.end())
				picks.push_back(pick);
			else if (before(pick, *sameStrand))
				*sameStrand = pick;
		}
		std::ranges::sort(picks, before);
		if (picks.size() > a_maxCount)
			picks.resize(a_maxCount);
		return picks;
	}
}
