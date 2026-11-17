#pragma once

#include "Math.h"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace SmpPhysics
{
	/** @brief Which Bullet constraint frame the file's frame element defines. */
	enum class FrameKind
	{
		FrameInA,
		FrameInB,
		FrameInLerp  ///< built from a blend of both bodies' rest frames rather than from either one alone
	};

	struct RigBone
	{
		std::string name;
		bool kinematic{};  ///< mass == 0
		float mass{};
		Vec3 inertia{};
		float angularDamping{};
		float linearDamping{};
		float gravityFactor{};
		float windFactor{ 1.0f };        ///< SMP's <wind-factor>: scales the weather wind force on this bone
		float friction{ 0.5f };          ///< SMP's <friction>: this body's share of a contact's combined friction; Bullet's default when unset
		float marginMultiplier{ 1.0f };  ///< SMP's <margin-multiplier>: scales the contact radius of this bone's hair spheres
		Transform comTransform{};        ///< SMP's <centerOfMassTransform>: the body frame relative to the node; identity for a kinematic bone
	};

	struct RigJoint
	{
		int child{};   ///< bodyA, the simulated bone
		int parent{};  ///< bodyB
		FrameKind frameKind{};
		Transform frame{};        ///< the frameInA or frameInB transform
		float translationLerp{};  ///< frameInLerp only: how far the frame slides from body A towards body B
		float rotationLerp{};     ///< frameInLerp only: how far the frame's rotation turns from body A's towards body B's
		Vec3 linearLower{};       ///< per axis: lower above upper is free, equal is locked, below is limited, as Bullet reads them
		Vec3 linearUpper{};
		Vec3 linearStiffness{};
		Vec3 linearDamping{};
		Vec3 linearEquilibrium{};
		Vec3 linearBounce{};
		Vec3 linearNonHookeanStiffness{};  ///< hdtSMP64's non-Hookean spring factors: how much the spring weakens or damps further from equilibrium
		Vec3 linearNonHookeanDamping{};
		bool linearSpringsEnabled{ true };
		bool linearStiffnessLimited{ true };
		Vec3 angularLower{};
		Vec3 angularUpper{};
		Vec3 angularStiffness{};
		Vec3 angularDamping{};
		Vec3 angularEquilibrium{};
		Vec3 angularBounce{};
		Vec3 angularNonHookeanStiffness{};
		Vec3 angularNonHookeanDamping{};
		bool angularSpringsEnabled{};
		bool angularStiffnessLimited{ true };  ///< SMP's default: Bullet caps the spring so it stays stable at the physics tick
		bool springDampingLimited{ true };
		bool reversed{};                  ///< NormalizeRigTopology turned child and parent around; the rigid builder swaps the bodies back
		bool useLinearReferenceFrameA{};  ///< SMP swaps the constraint's bodies, so the parent becomes body A
		bool linearMotors{};              ///< SMP's linear/angular motor and servo switches; a servo's target is the axis's equilibrium
		bool angularMotors{};
		bool linearServoMotors{};
		bool angularServoMotors{};
		Vec3 linearTargetVelocity{};
		Vec3 angularTargetVelocity{};
		Vec3 linearMaxMotorForce{};
		Vec3 angularMaxMotorForce{};
		float stopErp{ 0.2f };
		float motorErp{ 0.9f };
		float motorCfm{};
		std::optional<float> stopCfm;  ///< unset keeps the solver's own default, which differs between linear and angular axes
	};

	/** @brief One weight-threshold entry: a bone the shape's mesh is skinned to, and the weight a vertex needs to count as part of the shape. */
	struct RigShapeWeight
	{
		std::string bone;
		float threshold{};
	};

	/** @brief One collision shape as the file declares it, before any mesh is read for it. */
	struct RigShape
	{
		std::string name;     ///< the mesh the shape is built from
		float margin{};       ///< SMP adds this to every collider built from the mesh
		float penetration{};  ///< per-triangle shapes only; 0 for a per-vertex shape
		std::string shared;   ///< SMP's sharing scope, e.g. "private"
		std::string tag;      ///< the tag other shapes match against
		int priority{};
		std::vector<std::string> collidesWith;         ///< can-collide-with-tag
		std::vector<std::string> noCollidesWith;       ///< no-collide-with-tag
		std::vector<std::string> noCollidesWithBones;  ///< no-collide-with-bone
		std::vector<RigShapeWeight> weights;           ///< weight-threshold
	};

	/**
	 * @brief Whether a shape's own lists let it collide with a shape carrying a_tag, as SMP's canCollideWith reads them.
	 * @details An empty can-collide list collides with every tag the no-collide list does not name; a non-empty one
	 *          collides only with the tags it names.
	 */
	[[nodiscard]] bool AllowsCollisionWithTag(const RigShape& a_shape, const std::string& a_tag);

	/**
	 * @brief Which of a claim's first 32 surfaces a bone built from a_boneShape may touch, one bit each.
	 * @param a_surfaceShapes One entry per surface: the shape it was built from, or null for a surface that is not one of the rig's
	 *        (worn armor, the body), which every bone may touch. A shape never touches the surface it was built from.
	 * @return All bits set when a_boneShape is null.
	 */
	[[nodiscard]] std::uint32_t AllowedSurfaceMask(const RigShape* a_boneShape, const std::vector<const RigShape*>& a_surfaceShapes);

	/** @brief Whether two shapes collide: each one's lists must allow the other's tag. */
	[[nodiscard]] bool ShapesCanCollide(const RigShape& a_first, const RigShape& a_second);

	/** @brief Every collision shape a file declares, split by the kind of mesh it is built from. */
	struct RigCollision
	{
		std::vector<RigShape> vertexShapes;    ///< per-vertex-shape: a sphere at each vertex of the named mesh
		std::vector<RigShape> triangleShapes;  ///< per-triangle-shape: the named mesh's triangles
	};

	struct SmpRig
	{
		std::vector<RigBone> bones;
		std::vector<RigJoint> joints;  ///< at most one per bone
		/**
		 * @brief Joints naming a bone that joints already drives.
		 * @details SMP pins such a bone to two parents at once, which a single pivot per bone cannot express, so
		 *          the solver drives the bone from its first joint and this holds the rest for a later stage.
		 */
		std::vector<RigJoint> additionalJoints;
		RigCollision collision;
	};

	enum class ParseStatus
	{
		Ok,
		Unsupported,  ///< a valid SMP file describing something this module does not simulate
		Malformed     ///< not parseable at all
	};

	struct ParseResult
	{
		ParseStatus status{ ParseStatus::Malformed };
		std::string reason;
		SmpRig rig;
		std::vector<std::string> warnings;  ///< values the reader repaired, such as an empty number that kept its default
	};

	/** @brief Parses the bone-pendulum subset of an hdtSMP64 XML file. The rig is filled only on Ok; every other status carries a short reason. */
	[[nodiscard]] ParseResult ParseSmpXml(std::string_view a_xml);

	/** @brief The physics tick hdtSMP64 steps Bullet at, which sets how far it lets a spring's stiffness rise. */
	inline constexpr float kSmpPhysicsTick = 1.0f / 60.0f;

	/**
	 * @brief Damping ratio every joint spring is held at least to when a rig is claimed.
	 * @details The solver moves each bone under its parent's motion without the bone loading the parent back, so a
	 *          chain of lightly damped joints multiplies its resonance at every link and the tip whips at idle;
	 *          Bullet's coupled solve does not. 0.7 matches hdtSMP64's peak on a heavy rig's turn.
	 */
	inline constexpr float kMinimumSpringDampingRatio = 0.7f;

	/**
	 * @brief Caps every limited joint spring the way Bullet's generic 6DOF spring constraint does.
	 * @details Stiffness is held to m / (16 dt^2) and damping to m / dt, where m is the reduced inertia of the two
	 *          bodies (the child's alone when the parent is kinematic). A joint whose file clears the limited
	 *          flags keeps its raw values. Without this a stiff rig rings far above the frequency SMP allows.
	 */
	void ApplySpringLimits(SmpRig& a_rig, float a_step, float a_minimumDampingRatio = 0.0f) noexcept;

	/**
	 * @brief A copy of a_rig without the bones flagged in a_drop and every joint that touches one of them.
	 * @details SMP skips a bone whose node is missing and the constraints that name it; this does the same.
	 */
	[[nodiscard]] SmpRig DropBones(const SmpRig& a_rig, const std::vector<bool>& a_drop);

	/**
	 * @brief Makes a rig's joints a forest rooted at kinematic bones, as ValidateRig requires, without changing what SMP simulates.
	 * @details SMP builds symmetric Bullet constraints, so a dynamic bone that only ever appears as a joint's parent is
	 *          still held by it. A group of dynamic bones no joint connects to a kinematic bone falls freely in SMP and
	 *          is dropped here. A group that is connected is re-rooted: the joints between its root and the connecting
	 *          joint are turned around and flagged `reversed`, so the rigid builder still steps them as authored.
	 * @param a_reason Why a rig could not be normalized, when this returns false.
	 */
	[[nodiscard]] bool NormalizeRigTopology(SmpRig& a_rig, std::string& a_reason);

	/** @brief A joint's constraint frame expressed in each of its two bodies' local frames. */
	struct JointFrames
	{
		Transform frameA{};
		Transform frameB{};
	};

	/** @brief Resolves a joint's frame from the rest transforms of its child and parent, as SMP's calcFrame does for each FrameKind. */
	[[nodiscard]] JointFrames ResolveJointFrames(const RigJoint& a_joint, const Transform& a_childRest, const Transform& a_parentRest);

	/**
	 * @brief Checks the invariants the solver relies on: joint counts and, unless a_requireForest is false, a joint forest
	 *        over dynamic bones rooted at kinematic bones.
	 * @details The parser passes false, because the bones a skeleton lacks are not known until a mesh is claimed;
	 *          NormalizeRigTopology then makes the claim's rig a forest.
	 */
	[[nodiscard]] bool ValidateRig(const SmpRig& a_rig, std::string& a_reason, bool a_requireForest = true);

	/**
	 * @brief The rig's dynamic bones sorted so a parent always precedes its children, which ValidateRig
	 *        has already proved is a forest.
	 * @param a_jointOfBone The joint index of each bone, or -1 for a bone that has none.
	 */
	[[nodiscard]] std::vector<int> BuildParentFirstOrder(const SmpRig& a_rig, const std::vector<int>& a_jointOfBone);
}
