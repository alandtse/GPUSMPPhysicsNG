#pragma once

#include "Math.h"
#include "SkinnedSurface.h"
#include "SmpRig.h"
#include "SolverCommon.h"
#include "TriangleBvh.h"

#include <array>
#include <cstdint>
#include <vector>

/**
 * @brief The immutable records the chain solver compute shader reads, packed once per claim.
 *
 * Pure data and pure functions: no D3D11, no RE/SKSE, so the packing can be unit tested against
 * the rig it came from. The record layout is mirrored by ChainSolverCS.hlsl -- a field added
 * here needs the matching field there, in the same order.
 */
namespace SmpPhysics::GpuRig
{
	struct Float4
	{
		float x{};
		float y{};
		float z{};
		float w{};
	};

	struct Uint4
	{
		std::uint32_t x{};
		std::uint32_t y{};
		std::uint32_t z{};
		std::uint32_t w{};
	};

	/** @brief One rig bone's static state: its joint frames in both bodies, its body, and its joint's springs. */
	struct BoneStatic
	{
		Float4 rotationA{};
		Float4 originA{};
		Float4 rotationB{};
		Float4 originB{};
		Float4 body{};         ///< mass, mean inertia, angular damping, gravity factor
		Float4 stiffness{};    ///< xyz joint stiffness, w non-zero when the springs are enabled
		Float4 damping{};      ///< xyz joint damping
		Float4 equilibrium{};  ///< xyz joint equilibrium angles
		Float4 lower{};        ///< xyz joint lower limits
		Float4 upper{};        ///< xyz joint upper limits
		Float4 wind{};         ///< xyz the strand's direction in the bone's frame, w the wind-facing area
		Uint4 topology{};      ///< x parent bone, y parent is kinematic, z the bone's wind factor as float bits
		Uint4 tieSlots{};      ///< x and y: a tie's other bone slot plus one shifted left by one, with bit 0 set on the parent side; 0 for no tie
		Float4 tieOffset0{};   ///< xyz the first tie's rest offset of its child bone from its parent bone, in the parent's frame
		Float4 tieOffset1{};   ///< xyz the second tie's rest offset
		Float4 tieLower{};     ///< xyz lower limit of that offset's deviation from rest, shared by the bone's ties
		Float4 tieUpper{};     ///< xyz upper limit
	};
	static_assert(sizeof(BoneStatic) == 272);

	/** @brief One dynamic tree: its slice of the parent-first order, the kinematic bone it hangs from, and the collision range it shares with its claim. */
	struct TreeStatic
	{
		Uint4 range{};    ///< x first order slot, y order count, z kinematic parent bone, w on a claim's first tree the claim's tree count when its bones carry ties, else 0
		Uint4 contact{};  ///< x first collider slot, y collider count, z first driver slot, w driver count
	};
	static_assert(sizeof(TreeStatic) == 32);

	/** @brief One dynamic bone's slice of the claim's sphere list, and a sphere enclosing all of it. */
	struct BoneSpheres
	{
		Uint4 range{};   ///< x first sphere in the sphere list, y how many; 0 for a bone whose cluster is empty
		Float4 bound{};  ///< xyz centre in the bone's rest frame, w radius
	};
	static_assert(sizeof(BoneSpheres) == 32);

	/** @brief One per-vertex collision sphere, in its bone's own rest frame. */
	struct SphereRecord
	{
		Float4 centreRadius{};  ///< xyz centre, w radius
		Uint4 exempt{};         ///< x one bit per claim-local collider this sphere was authored inside
	};
	static_assert(sizeof(SphereRecord) == 32);

	/** @brief One sphere's contact at the start of a substep, as ChainSolverCS.hlsl's HairSphereHit holds it. */
	struct SphereHit
	{
		Float4 normalDepth{};
		Float4 centreCollider{};
	};
	static_assert(sizeof(SphereHit) == 32);

	/** @brief A collider kind, mirroring Collider::Type in Collision.h and the shader's own check. */
	enum class ColliderKind : std::uint32_t
	{
		Ellipsoid = 0,
		Capsule = 1,
		Plane = 2
	};

	/** @brief The driver slot of a collider that follows no node, which the shader skips as QueryColliders does. */
	inline constexpr std::uint32_t kInvalidDriverSlot = 0xFFFFFFFFu;

	/** @brief Bones in one tree the compute solver serves; the shader's wind and contact ancestry stop here, so a longer chain is declined. */
	inline constexpr std::uint32_t kMaxTreeBones = 64;

	/** @brief One analytic collision proxy, expressed in the frame of the driver it follows. */
	struct ColliderPacked
	{
		Float4 orientation{};  ///< ellipsoid: its axes in the driver's frame
		Float4 center{};       ///< ellipsoid centre, or a point on the plane
		Float4 halfExtents{};  ///< xyz ellipsoid half extents, w the capsule radius
		Float4 segmentA{};     ///< capsule end in the driver's frame
		Float4 segmentB{};     ///< capsule end in the driver's frame
		Float4 normal{};       ///< xyz plane normal, w the penetration allowance
		Uint4 kind{};          ///< x a ColliderKind, y the driver slot or kInvalidDriverSlot
	};
	static_assert(sizeof(ColliderPacked) == 112);

	/** @brief One driver's world transform, refreshed every submitted frame. */
	struct DriverWorld
	{
		Float4 rotation{};
		Float4 translation{};
	};
	static_assert(sizeof(DriverWorld) == 32);

	/** @brief One posed collision surface's hierarchy, as a frame's AppendSurfaces writes it. */
	struct SurfaceMeta
	{
		Float4 gridOrigin{};  ///< zero; the hierarchy has no origin and these lanes keep the record layout
		Uint4 dimensions{};   ///< xyz zero, w the triangle count
		Uint4 base{};         ///< x first triangle, y first node, z node count, in the frame's shared arrays
		Float4 contact{};     ///< x the surface margin, y its contact penetration allowance, z how far behind a one-sided face it holds a sphere
	};
	static_assert(sizeof(SurfaceMeta) == 64);

	/** @brief One posed triangle, in leaf order: its corners, its unit face normal with twice its area in w, and its source index in a.w. */
	struct SurfaceTriangle
	{
		Float4 a{};  ///< corner; w the source triangle index bit-cast to float
		Float4 b{};
		Float4 c{};
		Float4 normalArea{};  ///< xyz the unit face normal, w twice the area; zero when the triangle is unusable
	};
	static_assert(sizeof(SurfaceTriangle) == 64);

	/** @brief Every claim's posed surfaces for one frame, concatenated; a surface's nodes index its own triangles. */
	struct SurfaceFrame
	{
		std::vector<SurfaceMeta> metas;
		std::vector<SurfaceTriangle> triangles;
		std::vector<BvhNode> nodes;

		/** @brief Empties every array but keeps their capacity, so a frame that reuses it does not reallocate. */
		void Clear() noexcept
		{
			metas.clear();
			triangles.clear();
			nodes.clear();
		}
	};

	/**
	 * @brief Poses one surface, builds or refits its hierarchy, and appends it to a_frame; a surface with no usable
	 *        triangle adds nothing.
	 * @param a_bvh The surface's own hierarchy, built on first use and refitted on every later call.
	 */
	void AppendSurface(SurfaceFrame& a_frame, const SkinnedSurface& a_surface,
		const std::vector<Transform>& a_driverWorld, TriangleBvh& a_bvh);

	/**
	 * @brief Poses a claim's surfaces from the drivers' world transforms, builds a hierarchy per surface, and appends the result.
	 * @details A surface with no usable triangle is left out. A caller that rebuilds every frame keeps a TriangleBvh
	 *          per surface and calls AppendSurface, so the topology is built once.
	 * @param a_firstMeta Receives the index of the claim's first record in a_frame.metas.
	 * @param a_metaCount Receives how many records the claim added.
	 */
	void AppendSurfaces(SurfaceFrame& a_frame, const std::vector<SkinnedSurface>& a_surfaces,
		const std::vector<Transform>& a_driverWorld, std::uint32_t& a_firstMeta, std::uint32_t& a_metaCount);

	/**
	 * @brief Appends a frame AppendSurfaces built on its own, moving each record's buffer bases to where it now sits.
	 * @param a_firstMeta Receives the index of the first appended record in a_frame.metas.
	 */
	void MergeSurfaceFrame(SurfaceFrame& a_frame, const SurfaceFrame& a_part, std::uint32_t& a_firstMeta);

	/** @brief One claim's collision proxies, in the claim's own bone and collider index space. */
	struct CollisionData
	{
		std::vector<BoneSpheres> spheres;      ///< one per rig bone, indexed by bone
		std::vector<SphereRecord> sphereList;  ///< every bone's spheres, bone after bone; BoneSpheres::range.x indexes it
		std::vector<ColliderPacked> colliders;
		std::uint32_t driverCount{};  ///< how many drivers the colliders index
	};

	/** @brief One tree's inputs for a single frame. */
	struct TreeInput
	{
		Float4 previousRotation{};
		Float4 previousTranslation{};
		Float4 currentRotation{};
		Float4 currentTranslation{};
		Float4 solver{};                      ///< x substep length, y gravity, z gravity scale, w maximum angular speed
		Float4 scales{};                      ///< x stiffness scale, y damping scale, z the actor's wind exposure, w 1 when joints carry the wind load below them
		Float4 contact{};                     ///< x contact spring stiffness, y contact damping, z contact skin, w tie spring stiffness, so SolverCommon.h stays their only definition
		Float4 wind{};                        ///< xyz world wind vector, w gust clock
		Uint4 control{};                      ///< x substep count, y a previous input exists, z a reset was requested, w 1 when joints carry the load of the bones below them
		Uint4 surface{};                      ///< x first surface record this frame's AppendSurfaces gave the tree's claim, y how many, z 1 when the rigid solver owns the tree and the contact passes run for it every substep, w the claim's bone span
		Uint4 grabSlots{};                    ///< per pin, the held bone's slot plus one (0 for no pin)
		std::array<Float4, 4> grabTargets{};  ///< per pin, xyz where the hand holds the point and w the pin's stiffness
		std::array<Float4, 4> grabLocals{};   ///< per pin, xyz the held point in the bone's own frame
	};
	static_assert(sizeof(TreeInput) == 304);

	/** @brief One bone's world transform while a tree is being stepped. */
	struct BoneWorld
	{
		Float4 rotation{};
		Float4 translation{};
	};
	static_assert(sizeof(BoneWorld) == 32);

	/** @brief One bone's contacts merged into one at the start of a substep, as the shader's contact pass writes it. */
	struct BoneContact
	{
		float depth{};
		float deepest{};
		std::int32_t collider{ -1 };
		float normal[3]{};
		float position[3]{};
	};
	static_assert(sizeof(BoneContact) == 36);

	/** @brief One bone's root-relative result, which the host turns back into a world transform. */
	struct BoneOutput
	{
		Float4 rotation{};
		Float4 translation{};
		Uint4 status{};  ///< x the tree stopped being finite and was reset
	};
	static_assert(sizeof(BoneOutput) == 48);

	/** @brief Everything immutable about one claim that the shader needs. */
	struct RigData
	{
		std::vector<BoneStatic> bones;     ///< one per rig bone, indexed by bone
		std::vector<TreeStatic> trees;     ///< one per dynamic tree
		std::vector<std::uint32_t> order;  ///< every dynamic bone, parent first inside each tree
		CollisionData collision;           ///< empty until SetCollision packs the claim's proxies
	};

	/**
	 * @brief Derives the shader records from a rig and its rest pose.
	 * @details Joint frames are resolved from the rest pose with ResolveJointFrames, and the order is
	 *          BuildParentFirstOrder restricted to each tree in turn.
	 * @return Empty records when the rig and the rest pose disagree in size.
	 */
	[[nodiscard]] RigData BuildRigData(const SmpRig& a_rig, const std::vector<Transform>& a_restWorld, const std::vector<float>& a_windAreas = {});

	/**
	 * @brief Packs a claim's collision proxies.
	 * @param a_spheres One sphere list per rig bone; a list longer than kMaxContactSpheresPerBone is truncated
	 *                  the same way SetCollision truncates it, so both solvers see the same spheres.
	 * @param a_colliders The proxies, each naming its driver by that driver's index in the frame's driver list.
	 * @param a_driverCount The size of that list, which is what a collider's driver index is checked against.
	 */
	[[nodiscard]] CollisionData PackCollision(const std::vector<std::vector<BoneSphere>>& a_spheres,
		const std::vector<Collider>& a_colliders, std::uint32_t a_driverCount);

	/** @brief One collider packed for the compute shader; a collider with no driver packs as one the shader skips. */
	[[nodiscard]] ColliderPacked PackCollider(const Collider& a_collider);

	/**
	 * @brief Installs a claim's collision proxies, giving every tree the range of them it shares with the claim.
	 * @details A rig with no proxies keeps a zero count on every tree, which is what turns the shader's contact
	 *          path off exactly as a solver with no colliders leaves it off.
	 */
	void SetCollision(RigData& a_rig, CollisionData a_collision) noexcept;

	/**
	 * @brief The bone, collider and driver slots one claim's records occupy, in that order from the base
	 *        ShiftToSlotBase is handed, so two claims never overlap.
	 */
	[[nodiscard]] std::uint32_t SlotSpan(const RigData& a_rig) noexcept;

	constexpr std::uint32_t kNoTree = 0xFFFFFFFFu;  ///< The tree slot of a kinematic bone, which no tree carries.

	/** @brief Per rig bone, the slot of the tree that carries it (a_boneOffset plus the tree index), or kNoTree for a kinematic bone. */
	[[nodiscard]] std::vector<std::uint32_t> BoneTreeSlots(const RigData& a_rig, std::uint32_t a_boneOffset, std::uint32_t a_boneCount);

	/** @brief One claim's static records in the shared slot space, each ready to be written at the claim's own slots. */
	struct ClaimStatics
	{
		std::vector<BoneStatic> bones;          ///< at the bone slots
		std::vector<TreeStatic> trees;          ///< at the bone slots, one per tree
		std::vector<std::uint32_t> order;       ///< at the bone slots
		std::vector<BoneSpheres> spheres;       ///< at the bone slots, each range moved past the sphere base; empty for a rig without collision
		std::vector<SphereRecord> sphereList;   ///< the claim's records, each naming its bone slot, to append at the sphere base
		std::vector<ColliderPacked> colliders;  ///< at the collider slots
		std::vector<std::uint32_t> boneTrees;   ///< at the bone slots
	};

	/**
	 * @brief Packs one claim's statics for the shared buffers, from a rig already shifted to its slot base.
	 * @details A rebuild of every claim and the append of one claim both write exactly this, so an appended claim
	 *          lands as a rebuild would lay it out.
	 * @param a_sphereBase Where the claim's sphere records start in the shared sphere list.
	 */
	[[nodiscard]] ClaimStatics PackClaimStatics(const RigData& a_rig, std::uint32_t a_boneOffset, std::uint32_t a_boneCount, std::uint32_t a_sphereBase);

	/**
	 * @brief Moves a claim's records out of their own index space and into the shared slot space the shaders
	 *        index: every stored bone index and order offset gains a_base, a collider's driver index gains the
	 *        driver base, and each tree's collision range gains the collider and driver bases.
	 */
	void ShiftToSlotBase(RigData& a_rig, std::uint32_t a_base) noexcept;

	/**
	 * @brief For every bone, the kinematic bone its dynamic tree hangs from, which is the frame the
	 *        shader's output is expressed in; a kinematic bone maps to itself.
	 */
	[[nodiscard]] std::vector<int> BuildTreeKinematicParents(const SmpRig& a_rig);

	/**
	 * @brief For every dynamic bone, the farthest its kinematic root may place it: the rest length of the joint
	 *        chain from the root to the bone, times a_slack, plus a_margin. A kinematic bone gets zero.
	 * @details A limited linear axis adds its window times a_linearScale to its joint's length; a free one has no reach limit.
	 */
	[[nodiscard]] std::vector<float> BuildBoneReach(const SmpRig& a_rig, const std::vector<Transform>& a_restPose, float a_slack, float a_margin,
		float a_linearScale = 1.0f);

	/**
	 * @brief Whether a solver pose, each bone in its kinematic root's frame, has a bone that is not finite or sits
	 *        farther from that root than a_reach allows, which no pose of a connected strand can.
	 */
	[[nodiscard]] bool PoseExceedsReach(const SmpRig& a_rig, const std::vector<Transform>& a_bones, const std::vector<float>& a_reach);

	/** @brief Packs a frame's kinematic input for one tree, for the compute solver to interpolate from. */
	/** @brief Most pins one tree's input carries. */
	inline constexpr std::size_t kMaxGrabPinsPerTree = 4;

	/**
	 * @brief Adds a pin holding the point a_local of the bone at a_slot toward a_target to a tree's input.
	 * @return False when the tree already carries kMaxGrabPinsPerTree pins.
	 */
	[[nodiscard]] bool AddGrab(TreeInput& a_input, std::uint32_t a_slot, const Vec3& a_local, const Vec3& a_target, float a_stiffness) noexcept;

	[[nodiscard]] TreeInput MakeTreeInput(const SolverParams& a_params, const SubStepPlan& a_plan,
		const Transform& a_previousKinematic, const Transform& a_currentKinematic,
		bool a_hasPreviousInput, bool a_resetRequested);

	/** @brief One surface vertex's bind position and the global skin-bone slot of each of its four bone weights. */
	struct SurfacePrepVertex
	{
		Float4 position{};  ///< xyz bind position
		Uint4 bones{};      ///< one global skin-bone index per slot, or kInvalidDriverSlot when the slot carries no bone
		Float4 weights{};
	};
	static_assert(sizeof(SurfacePrepVertex) == 48);

	/** @brief One skin bone's bind transform and the global driver slot it follows. */
	struct SurfacePrepBone
	{
		Float4 rotation{};     ///< skinToBone rotation in w, x, y, z lane order, matching DriverWorld
		Float4 translation{};  ///< xyz skinToBone translation
		Uint4 driver{};        ///< x the global driver slot, or kInvalidDriverSlot when the bone is undriven
	};
	static_assert(sizeof(SurfacePrepBone) == 48);

	/** @brief One BVH leaf slot's triangle as global vertex indices, so the same record serves every surface. */
	struct SurfacePrepTriangle
	{
		Uint4 corners{};  ///< xyz global vertex indices, or kInvalidDriverSlot in all three; w the source triangle index
	};
	static_assert(sizeof(SurfacePrepTriangle) == 16);

	/** @brief One BVH node's topology: skip stays surface-relative for the traversal, the children index the shared node array. */
	struct SurfacePrepNode
	{
		Uint4 topology{};  ///< x surface-relative skip as the shader traverses it, y leaf field, z first child or kInvalidDriverSlot, w second child or kInvalidDriverSlot (both global)
	};
	static_assert(sizeof(SurfacePrepNode) == 16);

	/** @brief One BVH leaf node's slice of the shared triangle array. */
	struct SurfacePrepLeaf
	{
		Uint4 range{};  ///< x global node index, y global first triangle slot, z triangle count, w zero
	};
	static_assert(sizeof(SurfacePrepLeaf) == 16);

	/** @brief One internal BVH node's children and depth, which orders the refit dispatches. */
	struct SurfacePrepRefit
	{
		Uint4 link{};  ///< x global node index, y first child global, z second child global, w depth
	};
	static_assert(sizeof(SurfacePrepRefit) == 16);

	/** @brief Most refit dispatch levels a surface may need; a deeper hierarchy is refused rather than mis-dispatched. */
	inline constexpr std::uint32_t kMaxRefitLevels = 32;

	/** @brief Every claim's collision surfaces packed once, in the shared slot space the surface shaders index. */
	struct SurfacePrepData
	{
		std::vector<SurfacePrepVertex> vertices;
		std::vector<SurfacePrepBone> bones;
		std::vector<SurfacePrepTriangle> triangles;  ///< one per BVH leaf slot, in leaf order, from a meta's base.x
		std::vector<SurfacePrepNode> nodes;
		std::vector<SurfacePrepLeaf> leaves;
		std::vector<SurfacePrepRefit> refit;  ///< every internal node, deepest dispatch first after FinalizeSurfacePrep
		std::vector<SurfaceMeta> metas;
		std::vector<std::uint32_t> refitLevelStart;  ///< first refit index of each dispatch, then refit.size()
		std::uint32_t refitLevels{};

		/** @brief Empties every array but keeps their capacity. */
		void Clear() noexcept;
	};

	/**
	 * @brief Appends one surface's vertices, skin bones, leaf-order triangles, hierarchy topology and refit work list,
	 *        moving every index into the shared arrays.
	 * @param a_driverOffset The global slot a_surface's driver 0 occupies.
	 * @param a_driverCount How many drivers the surface owns, which validates each bone's driver index.
	 * @return false, leaving a_data untouched, when the hierarchy is not ready, does not cover the surface's triangles,
	 *         or has an internal node deeper than kMaxRefitLevels.
	 */
	[[nodiscard]] bool AppendSurfacePrep(SurfacePrepData& a_data, const SkinnedSurface& a_surface, const TriangleBvh& a_bvh,
		std::uint32_t a_driverOffset, std::uint32_t a_driverCount);

	/**
	 * @brief Orders the refit work list deepest dispatch first and recomputes the per-dispatch ranges.
	 * @details Call again after further surfaces were appended; the levels are rebuilt from scratch.
	 */
	void FinalizeSurfacePrep(SurfacePrepData& a_data);

	/**
	 * @brief The shader's posing, triangle build and hierarchy refit, run on the CPU as their reference.
	 * @details Expects FinalizeSurfacePrep to have ordered a_data.refit. Produces exactly what PoseSurface,
	 *          AppendSurface and TriangleBvh::Refit produce from the same pose.
	 * @param a_driverWorldBySlot One world transform per global driver slot.
	 * @param a_triangles Receives one record per global triangle slot.
	 * @param a_nodes Receives one record per node: skip and leaf from the topology, low and high from the pose.
	 */
	void ReferenceSurfacePrep(const SurfacePrepData& a_data, const std::vector<Transform>& a_driverWorldBySlot,
		std::vector<SurfaceTriangle>& a_triangles, std::vector<BvhNode>& a_nodes);

	/** @brief The absolute floor of the difference still reported as a match, in the record's own units. */
	inline constexpr float kSurfacePrepTolerance = 1e-4f;

	/** @brief What the tolerance adds per unit of coordinate magnitude, since world positions carry float rounding that grows with distance from the origin. */
	inline constexpr float kSurfacePrepRelativeTolerance = 1e-5f;

	/** @brief The floor of how much a unit-normal component and, as a fraction, a triangle's twice-area may differ across rounding. */
	inline constexpr float kSurfacePrepNormalTolerance = 1e-3f;

	/** @brief The rounding in a corner position per unit of its magnitude (a few dozen ulps), which a triangle's normal and area inherit divided by its edge length. */
	inline constexpr float kSurfacePrepRoundingPerMagnitude = 6.4e-6f;

	/** @brief A triangle of twice this area or less is too ill-conditioned for its normal and area to be compared across rounding. */
	inline constexpr float kSurfacePrepConditionedArea = 1.0f;

	/** @brief A box component above this magnitude is an empty-box sentinel rather than a bound. */
	inline constexpr float kSurfacePrepEmptySentinel = 1e29f;

	/** @brief How a GPU surface prep pass differed from ReferenceSurfacePrep. */
	struct SurfacePrepComparison
	{
		std::uint32_t triangles{};           ///< triangle records compared
		std::uint32_t nodes{};               ///< node records compared
		float maxTriangleError{};            ///< largest absolute difference of any triangle corner component
		float maxNormalError{};              ///< largest absolute difference of any unit-normal component of a well-conditioned triangle
		std::uint32_t triangleMismatches{};  ///< records whose corners, source index, usability, or normal and area differ beyond tolerance
		float maxNodeError{};                ///< largest absolute difference of any box component compared
		std::uint32_t nodeMismatches{};      ///< nodes whose box differs beyond tolerance
		std::uint32_t topologyMismatches{};  ///< nodes whose skip or leaf differs
	};

	/**
	 * @brief Compares actual surface prep output against the reference for the same pose.
	 * @details A box component is compared only when both sides are bounds; two empty-box sentinels carry no
	 *          information. A record present on only one side counts as a mismatch.
	 */
	[[nodiscard]] SurfacePrepComparison CompareSurfacePrep(const std::vector<SurfaceTriangle>& a_expectedTriangles,
		const std::vector<BvhNode>& a_expectedNodes, const SurfaceTriangle* a_actualTriangles, const BvhNode* a_actualNodes,
		std::uint32_t a_actualTriangleCount, std::uint32_t a_actualNodeCount) noexcept;
}
