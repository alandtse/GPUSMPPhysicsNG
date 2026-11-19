#pragma once

#include "Collision.h"
#include "Math.h"
#include "SkinnedSurface.h"
#include "SmpRig.h"
#include "WindDrag.h"

#include <array>
#include <cstddef>
#include <vector>

namespace SmpPhysics
{
	/** @brief Seconds the gust clock runs before it wraps, as in hdtSMP64. */
	inline constexpr float kWindClockWrap = 4096.0f;

	/** @brief The scales, wind and substep policy the compute solver runs a rig with. */
	struct SolverParams
	{
		float gravity{ 687.7193f };  ///< Skyrim units per second squared (9.8 / 0.01425)
		float gravityScale{ 1.0f };
		float stiffnessScale{ 1.0f };
		float dampingScale{ 1.0f };
		float bodyScale{ 1.0f };   ///< The rig nodes' world scale: hdtSMP64 scales a body's mass by its cube and its inertia and angular springs by its fifth power
		Vec3 wind{};               ///< World wind velocity in Skyrim units per second; zero switches wind off
		float windTime{};          ///< Gust clock in seconds, advanced once per frame by the caller
		float windFactor{ 1.0f };  ///< The actor's wind exposure, 0 fully sheltered to 1 fully exposed
	};

	/** @brief Wind speed, in Skyrim units per second, that one unit of wind strength gives at full sky wind speed. */
	inline constexpr float kWindSpeedPerStrength = 300.0f;
	/** @brief Fastest wind a bone is exposed to, so a transient blast cannot overpower the joints. */
	inline constexpr float kMaxWindSpeed = 900.0f;
	/**
	 * @brief How far, in Skyrim units, in front of a surface a hair sphere's contact already engages.
	 * @details About one substep's fall under gravity, so a bone resting on a surface keeps its contact between
	 *          substeps instead of losing it, falling back and chattering. Passed to the shader in TreeInput.
	 */
	inline constexpr float kContactSkin = 0.1f;
	/** @brief Stiffness of the pin that holds a grabbed strand to a hand, in force units per unit of distance. */
	inline constexpr float kGrabStiffness = 1500.0f;

	/** @brief A strand held by a hand: a point on the bone is pulled toward the target like a contact that pushes it there. */
	struct Grab
	{
		int bone{ -1 };  ///< The rig bone the hand holds, or -1 for none.
		Vec3 local{};    ///< The held point in the bone's own frame. A bone's origin is its joint, so a point along the strand is what a pull can move.
		Vec3 target{};   ///< World position the hand wants that point at.
		float stiffness{ kGrabStiffness };
	};

	/** @brief How far from a one-sided face a diagnostic probe still measures which side a sphere is on. */
	inline constexpr float kProbeBehindReach = 8.0f;
	/** @brief Distance behind a one-sided face past which a probe counts the sphere as behind it, a little over the armor allowance and skin. */
	inline constexpr float kProbeLostBehind = 0.5f;

	/**
	 * @brief hdtSMP64's frame-to-step policy: SkyrimPhysicsWorld::doUpdate and SkinnedMeshWorld::stepSimulation.
	 * @details Frames accumulate until more than half a tick has passed, then run whole ticks plus a variable
	 *          remainder, so the step sizes follow the frame times rather than splitting them evenly.
	 */
	class SmpStepClock
	{
	public:
		/** @brief The steps one frame runs and the time they cover; an empty list means the frame is skipped. */
		struct Frame
		{
			std::array<float, 64> steps{};
			int count{};
			float total{};
		};

		/** @brief Adds a_interval seconds and returns the steps to run now. */
		[[nodiscard]] Frame Advance(float a_interval) noexcept;
		/** @brief Forgets the accumulated time and restarts the interval average at one tick. */
		void Reset() noexcept;

	private:
		float averageInterval{ 1.0f / 60.0f };
		float accumulated{};
	};

	/** @brief a_params with every non-finite or out-of-range value replaced by one the solver can run with. */
	[[nodiscard]] SolverParams SanitizeParams(SolverParams a_params) noexcept;

	/** @brief Spheres at which ProbeSpheres spreads its queries over the cores; fewer are quicker on one. */
	inline constexpr std::size_t kParallelProbeSpheres = 512;

	/** @brief One collision sphere as placed by a set of bone poses, and the deepest collider it penetrates. */
	struct SphereProbe
	{
		int bone{};
		Vec3 center{};
		float radius{};
		float depth{};
		int collider{ -1 };
		float behind{};           ///< how far the centre sits behind the nearest one-sided face it projects onto, zero when it is in front of every one
		bool behindOutward{};     ///< that face points away from the body's axis under the head, so it is an outer shell rather than an inner lining
		int behindSurface{ -1 };  ///< index of the surface that face belongs to
		float insideShell{};      ///< how deep inside a worn shell the centre sits, measured out from the body axis (TriangleGrid::RadialDepthInside), zero when it is not
	};

	/**
	 * @brief Places every collision sphere with the bone poses the caller supplies, such as those the GPU solver returned, and reports its penetration, for diagnostics.
	 * @param a_spheres One sphere list per rig bone, in the bone's frame.
	 * @param a_colliders The proxies, each driven by an entry of a_driverWorld; a surface's contact index is the collider count plus its position in a_surfaces.
	 * @details Builds its own surface grids from a_driverWorld. Leaves a_out empty when there are no spheres or the bone count does not match.
	 */
	void ProbeSpheres(const std::vector<std::vector<BoneSphere>>& a_spheres, const std::vector<Collider>& a_colliders, const std::vector<SkinnedSurface>& a_surfaces,
		const std::vector<Transform>& a_boneWorld, const std::vector<Transform>& a_driverWorld, std::vector<SphereProbe>& a_out);

	/**
	 * @brief Stops a bone touching the surfaces its spheres were authored behind.
	 * @param a_probes The spheres placed at rest, as ProbeSpheres reports them.
	 * @param a_surfaceLimit Only surfaces below this index are exempted, so the body and worn armor keep colliding.
	 * @param a_boneCount The number of rig bones; a_boneSurfaceMask is created with every surface allowed when empty.
	 * @details One-sided cloth layers that lie one inside the other would otherwise push the inner layer through the outer one
	 *          on a hard move and trap it there, because the face then holds it on the wrong side.
	 * @return How many bone and surface pairs were exempted.
	 */
	std::size_t ExemptSurfacesBehind(const std::vector<SphereProbe>& a_probes, std::size_t a_surfaceLimit, std::size_t a_boneCount,
		std::vector<std::uint32_t>& a_boneSurfaceMask);
}
