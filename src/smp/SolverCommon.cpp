#include "SolverCommon.h"

#include "RestRetarget.h"
#include "TriangleGrid.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <execution>
#include <vector>

namespace SmpPhysics
{
	namespace
	{
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
		if (!IsFiniteVector(a_params.wind) || !std::isfinite(a_params.windTime) || !std::isfinite(a_params.windFactor)) {
			a_params.wind = {};
			a_params.windTime = 0.0f;
			a_params.windFactor = 0.0f;
		}
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

	void ProbeSpheres(const std::vector<std::vector<BoneSphere>>& a_spheres, const std::vector<Collider>& a_colliders, const std::vector<SkinnedSurface>& a_surfaces,
		const std::vector<Transform>& a_boneWorld, const std::vector<Transform>& a_driverWorld, std::vector<SphereProbe>& a_out)
	{
		a_out.clear();
		if (a_spheres.empty() || a_boneWorld.size() != a_spheres.size() || a_driverWorld.empty())
			return;
		std::vector<TriangleGrid> grids(a_surfaces.size());
		for (std::size_t index = 0; index < a_surfaces.size(); ++index)
			grids[index].Build(PoseSurface(a_surfaces[index], a_driverWorld), a_surfaces[index].indices, kSurfaceCellSize);
		struct Job
		{
			std::size_t bone;
			const BoneSphere* sphere;
		};
		std::vector<Job> jobs;
		for (std::size_t bone = 0; bone < a_spheres.size(); ++bone) {
			for (const BoneSphere& sphere : a_spheres[bone])
				jobs.push_back({ bone, &sphere });
		}
		a_out.resize(jobs.size());
		const auto probe = [&](const Job& a_job) {
			const std::size_t bone = a_job.bone;
			const BoneSphere& sphere = *a_job.sphere;
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
			a_out[static_cast<std::size_t>(&a_job - jobs.data())] = { static_cast<int>(bone), center, sphere.radius, best.depth, best.collider, behind, outward, behindSurface, insideShell };
		};
		if (jobs.size() >= kParallelProbeSpheres)
			std::for_each(std::execution::par, jobs.begin(), jobs.end(), probe);
		else
			std::for_each(jobs.begin(), jobs.end(), probe);
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
			if (surface >= a_surfaceLimit || surface >= kMaskedSurfaceLimit || bone >= a_boneCount)
				continue;
			if (a_boneSurfaceMask.size() < a_boneCount)
				a_boneSurfaceMask.resize(a_boneCount, kAllMaskBits);
			const std::uint32_t bit = 1u << surface;
			if ((a_boneSurfaceMask[bone] & bit) == 0u)
				continue;
			a_boneSurfaceMask[bone] &= ~bit;
			++exempted;
		}
		return exempted;
	}
}
