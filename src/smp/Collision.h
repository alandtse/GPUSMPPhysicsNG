#pragma once

#include "Math.h"
#include "Utils/SkinVertexBlock.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace SmpPhysics
{
	/** @brief Most per-vertex spheres one dynamic bone keeps; a bone with more hair vertices is thinned evenly. */
	inline constexpr int kMaxContactSpheresPerBone = 64;
	/** @brief Skin weight a vertex needs before DominantBone counts it as a member of a bone's cluster. */
	inline constexpr float kMinimumBoneWeight = Util::kMinDominantBoneWeight;
	/** @brief Half-extents a default head proxy starts from, in Skyrim units. */
	inline constexpr float kDefaultHeadRadius = 9.0f;
	/** @brief Radius of a neck capsule fitted between two skeleton nodes when no mesh is available, in Skyrim units. */
	inline constexpr float kDefaultNeckRadius = 3.5f;
	/** @brief The same for a shoulder capsule, which covers the trapezius and deltoid the neck capsule misses. */
	inline constexpr float kDefaultShoulderRadius = 6.0f;
	/** @brief The same for a torso capsule, which covers the chest and back long hair falls across. */
	inline constexpr float kDefaultTorsoRadius = 8.0f;
	/** @brief The same for a forearm capsule. */
	inline constexpr float kDefaultArmRadius = 5.0f;
	/** @brief How far past either end, as a fraction of a capsule's length, a skinned body vertex may project and still be measured. */
	inline constexpr float kSegmentSlack = 0.25f;
	/** @brief Radius a fitted capsule may not exceed, so a misread mesh cannot swallow the head. */
	inline constexpr float kMaximumCapsuleRadius = 40.0f;
	/** @brief The skeleton node hair and wind tests treat as the head. */
	inline constexpr const char* kHeadBoneName = "NPC Head [Head]";
	/** @brief World-space capsule colliders a claim reserves for other bodies, hands and held items that come near its hair. */
	inline constexpr std::size_t kExternalColliderSlots = 8;
	/** @brief Armour sits outside the naked body, so every fitted body radius is widened by this, in Skyrim units. */
	inline constexpr float kBodyArmorMargin = 0.5f;
	/** @brief Body vertices a segment needs before its measured radius is trusted; fewer than this keeps the default. */
	inline constexpr std::size_t kMinimumSegmentVertices = 12;
	/** @brief Distance percentile a segment's radius is taken from, so a stray armour vertex cannot widen the capsule. */
	inline constexpr float kSegmentRadiusPercentile = 0.90f;
	/** @brief Depth a collider tolerates before the solver's safety clamp starts pulling the bone back out, in Skyrim units. */
	inline constexpr float kDefaultPenetration = 0.1f;
	/**
	 * @brief Smallest half-extent an ellipsoid keeps, below which its scaled-space projection would divide by zero.
	 * @note Mirrored by kMinimumHalfExtent in package/Shaders/Common/HairChainSolver.hlsli.
	 */
	inline constexpr float kMinimumHalfExtent = 1e-3f;
	/**
	 * @brief Colliders a sphere's rest-exemption mask can name.
	 * @details A claim with more colliders than this can only exempt the first few, the rest staying live.
	 * @note Mirrored by kMaximumExemptColliders in package/Shaders/Common/HairChainSolver.hlsli.
	 */
	inline constexpr std::size_t kMaximumExemptColliders = 32;

	/** @brief One collision sphere of a dynamic bone, in that bone's rest frame. */
	struct BoneSphere
	{
		Vec3 centerLocal;
		float radius{};
		/**
		 * @brief The colliders this sphere was authored inside of, by a bit per claim-local collider index.
		 * @details A mod authored for another body can leave a bone's sphere resting well inside a fitted proxy;
		 *          such a collider must not shove it, or the safety clamp fights the spring every frame.
		 */
		std::uint32_t exemptMask{};
	};

	/**
	 * @brief One analytic collision proxy, expressed in the frame of the node that drives it.
	 * @details The driver is a skeleton node, or the actor root for a ground plane, whose world transform the
	 *          caller hands to QueryColliders every frame, so a proxy follows the body it belongs to.
	 */
	struct Collider
	{
		enum class Type
		{
			Ellipsoid,
			Capsule,
			Plane
		};

		Type type{ Type::Ellipsoid };
		int driverIndex{ -1 };
		Quat orientation{};               ///< ellipsoid: its axes in the driver's frame
		Vec3 center{};                    ///< ellipsoid centre, or a point on the plane
		Vec3 halfExtents{};               ///< ellipsoid
		Vec3 segmentA{};                  ///< capsule end in the driver's frame
		Vec3 segmentB{};                  ///< capsule end in the driver's frame
		float radius{};                   ///< capsule
		Vec3 normal{ 0.0f, 0.0f, 1.0f };  ///< plane
		float penetration{ kDefaultPenetration };
	};

	/** @brief The deepest contact a query found. Depth is how far the queried sphere reaches past the surface. */
	struct Contact
	{
		float depth{};       ///< positive only while the sphere penetrates
		Vec3 normal{};       ///< unit vector pointing out of the collider, in world space
		int collider{ -1 };  ///< index of the collider that produced it
	};

	/** @brief A sphere in a bone's rest frame that encloses every sphere of its list, so a bone far from a surface can skip them all. */
	struct SphereBound
	{
		Vec3 centre{};
		float radius{};
	};

	/** @brief The mean of the sphere centres, reaching the far side of the farthest sphere; a zero bound for an empty list. */
	[[nodiscard]] SphereBound BoundSpheres(const std::vector<BoneSphere>& a_spheres) noexcept;

	/**
	 * @brief hdtSMP64's contact merge: every contact a bone makes in one step becomes a single contact.
	 * @details Each contact weighs depth squared; the merged normal is the sum of the normals weighted by depth cubed,
	 *          and the merged depth the length of that sum over the weight total, a mean biased toward the deepest
	 *          contacts. The merged position is the weighted mean of the contact positions.
	 */
	struct ContactMerge
	{
		float weight{};      ///< sum of depth squared
		Vec3 normal{};       ///< sum of normal * depth cubed
		Vec3 position{};     ///< sum of position * depth squared
		float deepest{};     ///< deepest single contact
		int collider{ -1 };  ///< collider of the deepest single contact

		void Add(const Contact& a_contact, const Vec3& a_position) noexcept;
		/** @return The merged depth, zero when nothing was added; a_normal and a_position receive the rest. */
		[[nodiscard]] float Result(Vec3& a_normal, Vec3& a_position) const noexcept;
	};

	/** @brief One mesh vertex and the bones it is skinned to, as the mesh reader finds them. */
	struct SkinnedVertex
	{
		Vec3 position;
		int bone[4]{ -1, -1, -1, -1 };
		float weight[4]{};
	};

	/**
	 * @brief A mesh vertex skinned by its bones' current world transforms, as the renderer would draw it.
	 * @param a_vertex The vertex, with its position in mesh space and its skinning slots filled in.
	 * @param a_boneToWorld One world transform per skin bone.
	 * @param a_skinToBone One mesh-space to bone-space transform per skin bone, as NiSkinData stores it.
	 * @return The blended world position, or the mesh-space position when no slot has a usable weight.
	 */
	[[nodiscard]] Vec3 SkinVertexToWorld(const SkinnedVertex& a_vertex, const std::vector<Transform>& a_boneToWorld,
		const std::vector<Transform>& a_skinToBone) noexcept;

	/**
	 * @brief The bone a vertex's skinning is dominated by.
	 * @param a_boneCount How many bones the vertex's indices are valid within.
	 * @return The bone Util::DominantSkinBone picks, or -1 when none reaches kMinimumBoneWeight.
	 */
	[[nodiscard]] int DominantBone(const SkinnedVertex& a_vertex, int a_boneCount) noexcept;

	/**
	 * @brief Groups a hair mesh's vertices into per-vertex spheres, one list per dynamic bone, as hdtSMP64 collides them: a vertex joins every dynamic bone it has any weight on.
	 * @param a_vertices The mesh's vertices, in mesh space, with their skin weights.
	 * @param a_meshToBone One mesh-space to bone-space transform per rig bone.
	 * @param a_dynamicBone One flag per rig bone; only dynamic bones get spheres.
	 * @param a_margin The shape's collision margin, which is every sphere's radius as it is for hdtSMP64's per-vertex spheres.
	 * @param a_marginMultiplier Optional per-rig-bone scale on a_margin (hdtSMP64's <margin-multiplier>); a missing entry means 1.
	 * @return One vector per rig bone, empty for a bone no vertex reaches; a list past kMaxContactSpheresPerBone is thinned evenly.
	 */
	[[nodiscard]] std::vector<std::vector<BoneSphere>> ReduceHairSpheres(const std::vector<SkinnedVertex>& a_vertices,
		const std::vector<Transform>& a_meshToBone, const std::vector<bool>& a_dynamicBone, float a_margin,
		const std::vector<float>& a_marginMultiplier = {});

	/**
	 * @brief Fits an ellipsoid to a head mesh's vertices, already expressed in the head driver's frame.
	 * @details The axes are the vertex cloud's principal axes, each half-extent the 95th percentile of the
	 *          absolute projection onto it, capped at the cloud's own extent along that axis.
	 * @return A default sphere when there are too few vertices to describe a cloud.
	 */
	[[nodiscard]] Collider FitHeadEllipsoid(const std::vector<Vec3>& a_verticesInDriver, int a_driverIndex);

	/** @brief The head proxy used when no head mesh can be read: a sphere at the head node's origin. */
	[[nodiscard]] Collider MakeDefaultHeadCollider(int a_driverIndex);

	/** @brief One body vertex offered to a segment, in the frame of the node that drives it. */
	struct SegmentVertex
	{
		Vec3 position;
		bool skinnedToSegment{};  ///< whether the dominant skin bone is the segment's own bone
	};

	/** @brief What a segment's radius measurement found. */
	struct SegmentFit
	{
		float radius{};     ///< the measured radius, or the default when too few vertices landed on the segment
		int vertexCount{};  ///< vertices the measurement selected
		bool fitted{};      ///< false when the default was kept
	};

	/**
	 * @brief Measures a capsule's radius from the body vertices around it.
	 * @details A vertex is selected when it is skinned to the segment and projects onto it within kSegmentSlack of
	 *          either end, so the neighbouring body parts the same bone also weights stay out. The radius is then the kSegmentRadiusPercentile distance plus kBodyArmorMargin,
	 *          so an armour shell outside the naked body widens the capsule and a stray vertex does not.
	 * @param a_vertices The body vertices, in the driver's frame; only their positions and flags are read.
	 * @param a_endA One capsule end, in the driver's frame.
	 * @param a_endB The other capsule end, in the driver's frame.
	 * @param a_defaultRadius The radius kept when too few vertices were selected, and the smallest a fit may give.
	 * @return fitted is false, and the radius a_defaultRadius, when fewer than kMinimumSegmentVertices vertices
	 *         were selected; otherwise the radius is held within [a_defaultRadius, kMaximumCapsuleRadius].
	 */
	[[nodiscard]] SegmentFit FitSegmentRadius(const std::vector<SegmentVertex>& a_vertices, const Vec3& a_endA,
		const Vec3& a_endB, float a_defaultRadius) noexcept;

	/** @brief A capsule of a fixed radius, for a chain no mesh could be measured on. */
	[[nodiscard]] Collider MakeDefaultCapsule(const Vec3& a_endA, const Vec3& a_endB, float a_radius, int a_driverIndex);

	/** @brief A ground plane through a_point with a unit a_normal, driven by the actor root. */
	[[nodiscard]] Collider MakeGroundPlane(const Vec3& a_point, const Vec3& a_normal, int a_driverIndex);

	/**
	 * @brief The collider a sphere penetrates most deeply.
	 * @param a_driverWorld One world transform per driver the colliders name; a collider with no entry is skipped.
	 * @param a_exemptMask The colliders to ignore, one bit per collider index as ComputeRestExemptions set them.
	 * @return The deepest contact, or a zero depth and a zero normal when nothing is touched.
	 */
	[[nodiscard]] Contact QueryColliders(const std::vector<Collider>& a_colliders,
		const std::vector<Transform>& a_driverWorld, const Vec3& a_pointWorld, float a_radius,
		std::uint32_t a_exemptMask = 0u) noexcept;

	/**
	 * @brief Marks, per sphere, the colliders it already rests inside deeply enough to be exempt from them.
	 * @param a_spheres One sphere list per rig bone, in the bone's rest frame; each sphere's mask is overwritten.
	 * @param a_boneRestWorld One rest world transform per rig bone, the frame its spheres were built in.
	 * @param a_colliders The claim's proxies, indexed by the mask bit each one owns.
	 * @param a_driverWorld The drivers' world transforms at claim time, which are the rest ones.
	 * @details A sphere is exempt from a collider it reaches further into than that collider's own allowance, so a
	 *          proxy that merely grazes it keeps working while one it was authored inside cannot push it out.
	 */
	void ComputeRestExemptions(std::vector<std::vector<BoneSphere>>& a_spheres,
		const std::vector<Transform>& a_boneRestWorld, const std::vector<Collider>& a_colliders,
		const std::vector<Transform>& a_driverWorld);
}
