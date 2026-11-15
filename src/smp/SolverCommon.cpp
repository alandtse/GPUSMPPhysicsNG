#include "SolverCommon.h"

#include "RestRetarget.h"
#include "TriangleGrid.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace SmpPhysics
{
	namespace
	{
		constexpr float kDefaultFixedStep = 1.0f / 60.0f;
		constexpr int kMaxSubStepsLimit = 64;
		constexpr float kIntervalSmoothing = 0.125f;  ///< Share of each new frame interval folded into the running average.
	}

	SolverParams SanitizeParams(SolverParams a_params) noexcept
	{
		if (!std::isfinite(a_params.gravity))
			a_params.gravity = 0.0f;
		if (!std::isfinite(a_params.gravityScale))
			a_params.gravityScale = 0.0f;
		if (!std::isfinite(a_params.stiffnessScale))
			a_params.stiffnessScale = 0.0f;
		if (!std::isfinite(a_params.dampingScale))
			a_params.dampingScale = 0.0f;
		if (!std::isfinite(a_params.bodyScale) || a_params.bodyScale <= 0.0f)
			a_params.bodyScale = 1.0f;
		if (!std::isfinite(a_params.fixedStep) || a_params.fixedStep <= 0.0f)
			a_params.fixedStep = kDefaultFixedStep;
		if (!std::isfinite(a_params.maxFrameTime) || a_params.maxFrameTime < 0.0f)
			a_params.maxFrameTime = 0.0f;
		if (!std::isfinite(a_params.maxAngularSpeed) || a_params.maxAngularSpeed < 0.0f)
			a_params.maxAngularSpeed = 0.0f;
		if (!IsFiniteVector(a_params.wind) || !std::isfinite(a_params.windTime) || !std::isfinite(a_params.windFactor)) {
			a_params.wind = {};
			a_params.windTime = 0.0f;
			a_params.windFactor = 0.0f;
		}
		if (a_params.maxSubSteps < 1)
			a_params.maxSubSteps = 1;
		if (a_params.maxSubSteps > kMaxSubStepsLimit)
			a_params.maxSubSteps = kMaxSubStepsLimit;
		return a_params;
	}

	void SmpStepClock::Reset() noexcept
	{
		averageInterval = 1.0f / 60.0f;
		accumulated = 0.0f;
	}

	SmpStepClock::Frame SmpStepClock::Advance(float a_interval) noexcept
	{
		constexpr float kTimeTick = 1.0f / 60.0f;
		constexpr int kMaxTicks = 4;
		constexpr float kMinStep = 1.0f / 300.0f;
		Frame frame;
		if (!std::isfinite(a_interval) || a_interval <= 0.0f)
			return frame;
		accumulated += a_interval;
		averageInterval += (a_interval - averageInterval) * kIntervalSmoothing;
		const float tick = std::min(averageInterval, kTimeTick);
		if (!(accumulated * 2.0f > tick))
			return frame;
		float remaining = std::min(accumulated, tick * static_cast<float>(kMaxTicks));
		frame.total = remaining;
		while (remaining > tick && frame.count < static_cast<int>(frame.steps.size())) {
			frame.steps[static_cast<std::size_t>(frame.count++)] = tick;
			remaining -= tick;
		}
		if (remaining > kMinStep && frame.count < static_cast<int>(frame.steps.size()))
			frame.steps[static_cast<std::size_t>(frame.count++)] = remaining;
		accumulated = 0.0f;
		return frame;
	}

	SubStepPlan PlanSubSteps(float a_dt, const SolverParams& a_params) noexcept
	{
		SubStepPlan plan;
		if (!std::isfinite(a_dt) || a_dt <= 0.0f)
			return plan;
		const float frameTime = a_dt < a_params.maxFrameTime ? a_dt : a_params.maxFrameTime;
		if (!(frameTime > 0.0f))
			return plan;
		float requestedSubSteps = std::ceil(frameTime / a_params.fixedStep);
		if (!(requestedSubSteps > 1.0f))
			requestedSubSteps = 1.0f;
		if (requestedSubSteps > static_cast<float>(a_params.maxSubSteps))
			requestedSubSteps = static_cast<float>(a_params.maxSubSteps);
		plan.count = static_cast<int>(requestedSubSteps);
		plan.step = frameTime / static_cast<float>(plan.count);
		return plan;
	}

	Vec3 WindForce(const SolverParams& a_params, float a_boneFactor, float a_mass, float a_area, const Vec3& a_axis,
		const Vec3& a_origin, const Vec3& a_velocity) noexcept
	{
		constexpr float kCrosswindTimeScale = 0.004f;
		constexpr float kVerticalPhaseScale = 0.006f;
		constexpr float kEpsilon = 1.1920929e-7f;
		constexpr float kLongGustRate = 0.55f;
		constexpr float kLongGustVertical = 0.35f;
		constexpr float kMidGustRate = 1.35f;
		constexpr float kMidGustFeedback = 0.5f;
		constexpr float kFlutterRate = 4.15f;
		constexpr float kFlutterVertical = 2.2f;
		constexpr float kPulseRate = 0.85f;
		constexpr float kPulseVertical = 0.6f;
		constexpr float kSidewaysMid = 0.13f;
		constexpr float kSidewaysFlutter = 0.06f;
		constexpr float kSidewaysBurst = 0.04f;
		constexpr float kLiftRate = 1.9f;
		constexpr float kLiftVertical = 1.4f;
		constexpr float kLiftGust = 0.018f;
		constexpr float kLiftFlutter = 0.01f;

		const float magnitude = Length(a_params.wind);
		if (!(magnitude > kEpsilon) || !(a_boneFactor > kEpsilon) || !(a_area > kEpsilon))
			return {};
		const Vec3 relativeWind = a_params.wind - a_velocity;
		const Vec3 across = relativeWind - a_axis * Dot(a_axis, relativeWind);
		const float speed = Length(across);
		if (!(speed > kEpsilon))
			return {};
		Vec3 force = across * (kWindDrag * a_area * a_boneFactor * speed);
		float forceLength = Length(force);
		const float maximumForce = a_mass * kMaxWindAcceleration;
		if (forceLength > maximumForce) {
			force = force * (maximumForce / forceLength);
			forceLength = maximumForce;
		}

		const float time = std::fmod(a_params.windTime, kWindClockWrap);
		const Vec3 direction = a_params.wind / magnitude;
		const Vec3 up{ 0.0f, 0.0f, 1.0f };
		Vec3 side = Cross(direction, up);
		side = Dot(side, side) < kEpsilon ? Vec3{ 1.0f, 0.0f, 0.0f } : Normalized(side);

		const float gustSpeed = std::clamp(magnitude * kGustSpeedPerAcceleration, kMinGustSpeed, kMaxGustSpeed);
		const Vec3 origin = a_origin * kBulletPerSkyrimUnit;
		const float advected = time - Dot(origin, direction) / gustSpeed - Dot(origin, side) * kCrosswindTimeScale;
		const float vertical = origin.z * kVerticalPhaseScale;

		const float longGust = std::sin(advected * kLongGustRate + vertical * kLongGustVertical);
		const float midGust = std::sin(advected * kMidGustRate + vertical + longGust * kMidGustFeedback);
		const float flutter = std::sin(advected * kFlutterRate + vertical * kFlutterVertical + midGust);
		const float pulse = 0.5f + 0.5f * std::sin(advected * kPulseRate + vertical * kPulseVertical);
		const float burst = pulse * pulse;

		const float sideways = (midGust * kSidewaysMid + flutter * kSidewaysFlutter + burst * kSidewaysBurst) * forceLength;
		const float lift = (std::sin(advected * kLiftRate + vertical * kLiftVertical) * kLiftGust + flutter * kLiftFlutter) * forceLength;
		return force + side * sideways + up * lift;
	}

	void ProbeSpheres(const std::vector<std::vector<BoneSphere>>& a_spheres, const std::vector<Collider>& a_colliders, const std::vector<SkinnedSurface>& a_surfaces,
		const std::vector<Transform>& a_boneWorld, const std::vector<Transform>& a_driverWorld, std::vector<SphereProbe>& a_out)
	{
		a_out.clear();
		if (a_spheres.empty() || a_boneWorld.size() != a_spheres.size() || a_driverWorld.empty())
			return;
		std::vector<TriangleGrid> grids(a_surfaces.size());
		for (std::size_t index = 0; index < a_surfaces.size(); ++index)
			grids[index].Build(PoseSurface(a_surfaces[index], a_driverWorld), a_surfaces[index].indices, kSurfaceCellSize);
		for (std::size_t bone = 0; bone < a_spheres.size(); ++bone) {
			for (const BoneSphere& sphere : a_spheres[bone]) {
				const Vec3 center = a_boneWorld[bone].TransformPoint(sphere.centerLocal);
				Contact best = QueryColliders(a_colliders, a_driverWorld, center, sphere.radius, sphere.exemptMask);
				float behind = 0.0f;
				float insideShell = 0.0f;
				Vec3 behindNormal{};
				int behindSurface = -1;
				const Vec3 axisPoint = a_driverWorld.front().translation;
				for (std::size_t index = 0; index < a_surfaces.size(); ++index) {
					const Contact contact = grids[index].Query(center, sphere.radius + a_surfaces[index].margin, a_surfaces[index].contactPenetration,
						a_surfaces[index].deepPush ? kDeepPushReach : 0.0f);
					if (contact.depth > best.depth)
						best = { contact.depth, contact.normal, static_cast<int>(a_colliders.size() + index) };
					if (!(a_surfaces[index].contactPenetration > 0.0f))
						continue;
					if (!a_surfaces[index].deepPush && !a_surfaces[index].skin) {
						const float depth = grids[index].RadialDepthInside(center, kRetargetReach, a_surfaces[index].contactPenetration, axisPoint);
						if (std::isfinite(depth))
							insideShell = std::max(insideShell, -depth);
					}
					Vec3 normal{};
					const float distance = grids[index].NearestFaceDistance(center, kProbeBehindReach, &normal);
					if (-distance > behind) {
						behind = -distance;
						behindNormal = normal;
						behindSurface = static_cast<int>(index);
					}
				}
				const Vec3 fromAxis = center - a_driverWorld.front().translation;
				const bool outward = Dot(behindNormal, Vec3{ fromAxis.x, fromAxis.y, 0.0f }) > 0.0f;
				a_out.push_back({ static_cast<int>(bone), center, sphere.radius, best.depth, best.collider, behind, outward, behindSurface, insideShell });
			}
		}
	}

	std::size_t ExemptSurfacesBehind(const std::vector<SphereProbe>& a_probes, std::size_t a_surfaceLimit, std::size_t a_boneCount,
		std::vector<std::uint32_t>& a_boneSurfaceMask)
	{
		std::size_t exempted = 0;
		for (const SphereProbe& probe : a_probes) {
			if (probe.behind <= kProbeLostBehind || probe.behindSurface < 0 || probe.bone < 0)
				continue;
			const auto surface = static_cast<std::size_t>(probe.behindSurface);
			const auto bone = static_cast<std::size_t>(probe.bone);
			if (surface >= a_surfaceLimit || surface >= 32 || bone >= a_boneCount)
				continue;
			if (a_boneSurfaceMask.size() < a_boneCount)
				a_boneSurfaceMask.resize(a_boneCount, 0xFFFFFFFFu);
			const std::uint32_t bit = 1u << surface;
			if ((a_boneSurfaceMask[bone] & bit) == 0u)
				continue;
			a_boneSurfaceMask[bone] &= ~bit;
			++exempted;
		}
		return exempted;
	}
}
