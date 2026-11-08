#include "Common/HairChainSolver.hlsli"

// Per substep ContactCull, ContactSpheres and ContactReduce build each bone's contact, then Step walks each tree.
// The record layouts mirror src/Features/SmpPhysics/GpuRigPacking.h; keep them in step.

struct HairBoneStatic
{
	float4 rotationA;
	float4 originA;
	float4 rotationB;
	float4 originB;
	float4 body;         // mass, inertiaMean, angularDamping, gravityFactor
	float4 stiffness;    // joint xyz stiffness, w = springs enabled
	float4 damping;      // joint xyz damping
	float4 equilibrium;  // joint xyz equilibrium angles
	float4 lower;        // joint xyz lower limits
	float4 upper;        // joint xyz upper limits
	float4 wind;         // xyz = the strand's direction in the bone's frame, w = the wind-facing area
	uint4 topology;      // x = parent bone slot, y = 1 when the parent is kinematic
	uint4 tieSlots;      // x, y = a tie's other bone slot plus one, shifted left by one, with bit 0 set on the parent side; 0 for no tie
	float4 tieOffset0;   // xyz = the first tie's rest offset of its child bone from its parent bone, in the parent's frame
	float4 tieOffset1;   // xyz = the second tie's rest offset
	float4 tieLower;     // xyz = lower limit of that offset's deviation from rest, shared by the bone's ties
	float4 tieUpper;     // xyz = upper limit
};

struct HairTreeStatic
{
	uint4 range;    // x = first order slot, y = order count, z = kinematic parent bone slot
	uint4 contact;  // x = first collider slot, y = collider count, z = first driver slot, w = driver count
};

struct HairBoneSphere
{
	uint4 range;   // x = first sphere in the sphere list, y = how many
	float4 bound;  // xyz = centre in the bone's rest frame, w = radius that encloses every sphere
};

struct HairSphereRecord
{
	float4 centreRadius;  // xyz = centre in the bone's rest frame, w = radius
	uint4 exempt;         // x = rest-exemption mask, y = the slot of the bone the sphere belongs to
};

struct HairCollider
{
	float4 orientation;  // ellipsoid: its axes in the driver's frame
	float4 center;       // xyz = ellipsoid centre, or a point on the plane
	float4 halfExtents;  // xyz = ellipsoid half extents, w = capsule radius
	float4 segmentA;     // capsule end in the driver's frame
	float4 segmentB;     // capsule end in the driver's frame
	float4 normal;       // xyz = plane normal, w = penetration allowance
	uint4 kind;          // x = ColliderKind, y = driver slot
};

struct HairDriverWorld
{
	float4 rotation;
	float4 translation;
};

struct HairTreeInput
{
	float4 previousRotation;
	float4 previousTranslation;
	float4 currentRotation;
	float4 currentTranslation;
	float4 solver;          // x = substep length, y = gravity, z = gravity scale, w = maximum angular speed
	float4 scales;          // x = stiffness scale, y = damping scale, z = the actor's wind exposure, w = 1 when joints carry the wind load below them
	float4 contact;         // x = contact spring stiffness, y = contact damping, z = contact skin, w = tie spring stiffness
	float4 wind;            // xyz = world wind vector, w = gust clock
	uint4 control;          // x = substep count, y = 1 when a previous input exists, z = 1 when a reset is requested
	uint4 surface;          // x = first surface record of the tree's claim this frame, y = how many
	uint4 grabSlots;        // per pin, the held bone's slot plus one (0 for no pin)
	float4 grabTargets[4];  // per pin, xyz = where the hand holds the point, w = the pin's stiffness
	float4 grabLocals[4];   // per pin, xyz = the held point in the bone's own frame
};

struct HairSurfaceMeta
{
	float4 gridOrigin;  // unused, kept for the packed layout
	uint4 dimensions;   // w = face count
	uint4 base;         // x = first face, y = first node, z = node count
	float4 contact;     // x = the surface margin, y = its contact penetration allowance, z = how far behind a one-sided face it holds a sphere
};

struct HairSurfaceTriangle
{
	float4 a;  // w = the source face index
	float4 b;
	float4 c;
	float4 normalArea;  // xyz = unit face normal, w = twice the area, 0 or less when the face is degenerate
};

struct HairBvhNode
{
	float3 low;
	uint skip;  // surface-relative index of the first node after this one's subtree
	float3 high;
	uint leaf;  // 0 for an internal node, else (first slot << 3) | count
};

struct HairBoneState
{
	float4 rotation;
	float4 angularVelocity;
	float4 previousPivot;
	float4 previousPivotVelocity;
	float4 previousParentRotation;
	uint4 history;  // x = 1 when the bone has a previous pivot, y = 1 when it has a previous pivot velocity
};

struct HairBoneWorld
{
	float4 rotation;
	float4 translation;
};

struct HairBoneOutput
{
	float4 rotation;     // root relative
	float4 translation;  // root relative
	uint4 status;        // x = 1 when the tree stopped being finite and was reset
};

StructuredBuffer<HairBoneStatic> HairBoneStatics : register(t0);
StructuredBuffer<HairTreeStatic> HairTrees : register(t1);
StructuredBuffer<HairTreeInput> HairTreeInputs : register(t2);
StructuredBuffer<uint> HairOrder : register(t3);
StructuredBuffer<uint> HairResetList : register(t4);
StructuredBuffer<HairBoneSphere> HairBoneSpheres : register(t5);
StructuredBuffer<HairCollider> HairColliders : register(t6);
StructuredBuffer<HairDriverWorld> HairDrivers : register(t7);
StructuredBuffer<HairSurfaceMeta> HairSurfaces : register(t8);
StructuredBuffer<HairSurfaceTriangle> HairSurfaceTriangles : register(t9);
StructuredBuffer<HairBvhNode> HairSurfaceNodes : register(t10);
StructuredBuffer<HairSphereRecord> HairSphereList : register(t12);
StructuredBuffer<uint> HairBoneTrees : register(t13);  // per bone slot: its tree slot, or 0xFFFFFFFF for a kinematic bone

RWStructuredBuffer<HairBoneState> HairBoneStates : register(u0);
RWStructuredBuffer<HairBoneWorld> HairBoneWorlds : register(u1);
RWStructuredBuffer<HairBoneOutput> HairBoneOutputs : register(u2);
RWStructuredBuffer<uint> HairTreeFlags : register(u4);  // per tree slot: 1 once the tree stopped being finite this frame

cbuffer HairChainSolverParams : register(b0)
{
	uint HairTreeCount;
	uint HairResetCount;
	uint HairSubStep;       // the substep this dispatch advances, counted from 1
	uint HairBoneCount;     // one past the highest bone slot any claim uses
	uint HairSphereCount;   // entries in the sphere list
	uint HairContactOwner;  // 0 = the chain pass, which serves chain trees; 1 = a rigid substep's pass, which serves rigid trees
};

HairChainSolver::Quat LoadQuat(float4 a_value)
{
	HairChainSolver::Quat result;
	result.w = a_value.x;
	result.x = a_value.y;
	result.y = a_value.z;
	result.z = a_value.w;
	return result;
}

float4 StoreQuat(HairChainSolver::Quat a_value)
{
	return float4(a_value.w, a_value.x, a_value.y, a_value.z);
}

HairChainSolver::Transform LoadTransform(float4 a_rotation, float4 a_translation)
{
	HairChainSolver::Transform result;
	result.rotation = LoadQuat(a_rotation);
	result.translation = a_translation.xyz;
	return result;
}

HairChainSolver::JointFrame LoadJointFrame(HairBoneStatic a_bone)
{
	HairChainSolver::JointFrame result;
	result.rotationA = LoadQuat(a_bone.rotationA);
	result.originA = a_bone.originA.xyz;
	result.rotationB = LoadQuat(a_bone.rotationB);
	result.originB = a_bone.originB.xyz;
	return result;
}

HairChainSolver::BoneParameters LoadBoneParameters(HairBoneStatic a_bone)
{
	HairChainSolver::BoneParameters result;
	result.mass = a_bone.body.x;
	result.inertiaMean = a_bone.body.y;
	result.angularDamping = a_bone.body.z;
	result.gravityFactor = a_bone.body.w;
	result.windFactor = asfloat(a_bone.topology.z);
	result.windAxis = a_bone.wind.xyz;
	result.windArea = a_bone.wind.w;
	return result;
}

HairChainSolver::JointSprings LoadJointSprings(HairBoneStatic a_bone)
{
	HairChainSolver::JointSprings result;
	result.stiffness = a_bone.stiffness.xyz;
	result.damping = a_bone.damping.xyz;
	result.equilibrium = a_bone.equilibrium.xyz;
	result.lower = a_bone.lower.xyz;
	result.upper = a_bone.upper.xyz;
	result.enabled = a_bone.stiffness.w != 0.0f;
	return result;
}

HairChainSolver::SolverScales LoadSolverScales(HairTreeInput a_input)
{
	HairChainSolver::SolverScales result;
	result.gravity = a_input.solver.y;
	result.gravityScale = a_input.solver.z;
	result.stiffnessScale = a_input.scales.x;
	result.dampingScale = a_input.scales.y;
	result.maxAngularSpeed = a_input.solver.w;
	result.contactStiffness = a_input.contact.x;
	result.contactDamping = a_input.contact.y;
	result.contactSkin = a_input.contact.z;
	result.tieStiffness = a_input.contact.w;
	result.wind = a_input.wind.xyz;
	result.windTime = a_input.wind.w;
	result.windFactor = a_input.scales.z;
	result.windChainLoad = a_input.scales.w != 0.0f;
	return result;
}

/** @brief Unpacks one collider, whose radius and penetration ride in the spare lanes of two of its vectors. */
HairChainSolver::Collider LoadCollider(HairCollider a_collider)
{
	HairChainSolver::Collider result;
	result.kind = (int)a_collider.kind.x;
	result.orientation = LoadQuat(a_collider.orientation);
	result.center = a_collider.center.xyz;
	result.halfExtents = a_collider.halfExtents.xyz;
	result.segmentA = a_collider.segmentA.xyz;
	result.segmentB = a_collider.segmentB.xyz;
	result.radius = a_collider.halfExtents.w;
	result.normal = a_collider.normal.xyz;
	result.penetration = a_collider.normal.w;
	return result;
}

/**
 * @brief The deepest contact a sphere makes with a claim's colliders, the port of QueryColliders in Collision.cpp.
 * @details A collider whose driver is outside the claim's own driver range is skipped there exactly as the CPU
 *          skips one whose driver the frame did not supply, so a stale slot is never read as a live transform.
 * @param a_exemptMask The colliders to ignore, one bit per collider index as ComputeRestExemptions set them.
 * @param a_candidateMask The colliders worth testing, one bit per index below kMaximumExemptColliders; a collider past it is always tested.
 */
HairChainSolver::Contact QueryColliders(HairTreeStatic a_tree, float3 a_point, float a_radius, uint a_exemptMask, uint a_candidateMask)
{
	HairChainSolver::Contact best = HairChainSolver::NoContact();
	if (!HairChainSolver::IsFiniteVector(a_point) || !isfinite(a_radius))
		return best;
	uint colliderCount = a_tree.contact.y;
	uint driverFirst = a_tree.contact.z;
	uint driverCount = a_tree.contact.w;
	for (uint index = 0; index < colliderCount; ++index) {
		// A collider the sphere was authored inside is not a contact at all, or its spring would fight the clamp.
		if (index < HairChainSolver::kMaximumExemptColliders && ((a_exemptMask & (1u << index)) != 0u || (a_candidateMask & (1u << index)) == 0u))
			continue;
		HairCollider raw = HairColliders[a_tree.contact.x + index];
		if (raw.kind.y < driverFirst || raw.kind.y >= driverFirst + driverCount)
			continue;
		HairDriverWorld rawWorld = HairDrivers[raw.kind.y];
		HairChainSolver::Transform world = LoadTransform(rawWorld.rotation, rawWorld.translation);
		if (!HairChainSolver::IsFiniteTransform(world))
			continue;
		HairChainSolver::MergeContact(best, LoadCollider(raw), world, a_point, a_radius, (int)index);
	}
	return best;
}

/** @brief Whether a query box intersects a node's box on all three axes; touching counts. */
bool BoxOverlapsNode(HairBvhNode a_node, float3 a_low, float3 a_high)
{
	return all(a_high >= a_node.low) && all(a_low <= a_node.high);
}

/**
 * @brief Folds one face's contact with the sphere into the deepest one a surface query has found.
 * @param a_id The face's source index; of two faces at the same depth the lower index wins, so the pick is stable.
 */
void OfferSurfaceFace(HairSurfaceTriangle a_face, uint a_id, float3 a_point, float a_reach, bool a_flipped,
	float a_allowance, float a_behind, inout HairChainSolver::Contact a_best, inout uint a_bestId)
{
	if (a_face.normalArea.w <= 0.0f)
		return;
	float3 normal = a_flipped ? -a_face.normalArea.xyz : a_face.normalArea.xyz;
	float distance = dot(a_point - a_face.a.xyz, normal);
	if (a_allowance > 0.0f) {
		if (distance >= a_reach || distance < -a_behind)
			return;
	} else {
		if (distance < 0.0f) {
			distance = -distance;
			normal = -normal;
		}
		if (distance >= a_reach)
			return;
	}
	float depth = a_reach - distance;
	if (depth < a_best.depth || (depth == a_best.depth && a_id >= a_bestId))
		return;
	float3 projected = a_point - normal * distance;
	float3 toA = a_face.a.xyz - projected;
	float3 toB = a_face.b.xyz - projected;
	float3 toC = a_face.c.xyz - projected;
	float areaA = length(cross(toB, toC));
	float areaB = length(cross(toC, toA));
	float areaC = length(cross(toA, toB));
	float twiceArea = a_face.normalArea.w;
	if (areaA + areaB > twiceArea || areaB + areaC > twiceArea || areaC + areaA > twiceArea)
		return;
	a_best.depth = depth;
	a_best.normal = normal;
	a_bestId = a_id;
}

/**
 * @brief The deepest contact a sphere makes with one posed surface, the port of TriangleGrid::Query in TriangleGrid.cpp, enumerating through a BVH.
 * @param a_reach The sphere's radius plus the surface margin.
 */
HairChainSolver::Contact QuerySurface(HairSurfaceMeta a_meta, float3 a_point, float a_reach)
{
	HairChainSolver::Contact best = HairChainSolver::NoContact();
	float penetration = a_meta.contact.y;
	bool flipped = penetration < 0.0f;
	float allowance = abs(penetration);
	// Only a one-sided face has an inside a sphere can be stuck in, so only it reaches past its allowance.
	float behind = penetration > 0.0f ? allowance + a_meta.contact.z : allowance;
	float extent = max(a_reach, behind);
	float3 low = a_point - float3(extent, extent, extent);
	float3 high = a_point + float3(extent, extent, extent);
	uint bestId = 0xFFFFFFFFu;
	uint node = 0u;
	while (node < a_meta.base.z) {
		HairBvhNode current = HairSurfaceNodes[a_meta.base.y + node];
		if (!BoxOverlapsNode(current, low, high)) {
			node = current.skip;
			continue;
		}
		if (current.leaf != 0u) {
			uint first = current.leaf >> 3;
			uint count = current.leaf & 7u;
			for (uint index = 0u; index < count; ++index) {
				HairSurfaceTriangle face = HairSurfaceTriangles[a_meta.base.x + first + index];
				OfferSurfaceFace(face, asuint(face.a.w), a_point, a_reach, flipped, allowance, behind, best, bestId);
			}
			node = current.skip;
		} else {
			node = node + 1u;
		}
	}
	return best;
}

/**
 * @brief The deepest contact a sphere makes with the claim's colliders and posed surfaces, the port of
 *        ChainSolver::QueryContacts. A surface's contact index is the collider count plus its record's position.
 */
HairChainSolver::Contact QueryContacts(HairTreeStatic a_tree, uint4 a_surface, uint a_surfaceMask, uint a_colliderMask, float3 a_point, float a_radius, uint a_exemptMask)
{
	HairChainSolver::Contact best = QueryColliders(a_tree, a_point, a_radius, a_exemptMask, a_colliderMask);
	if (!HairChainSolver::IsFiniteVector(a_point) || !isfinite(a_radius))
		return best;
	for (uint index = 0; index < a_surface.y; ++index) {
		if (index < 32u && (a_surfaceMask & (1u << index)) == 0u)
			continue;
		HairSurfaceMeta meta = HairSurfaces[a_surface.x + index];
		HairChainSolver::Contact contact = QuerySurface(meta, a_point, a_radius + meta.contact.x);
		if (contact.depth > best.depth) {
			best = contact;
			best.collider = (int)(a_tree.contact.y + index);
		}
	}
	return best;
}

/** @brief One bone's contacts merged into one, the port of ChainSolver::BoneContact. */
struct HairBoneContact
{
	float depth;
	float deepest;
	int collider;
	float3 normal;
	float3 position;
};

RWStructuredBuffer<HairBoneContact> HairBoneContactsOut : register(u3);  // Contact's write view: per bone slot, its merged contact at the start of the substep
StructuredBuffer<HairBoneContact> HairBoneContactsIn : register(t14);    // Step's read view of the same buffer; the two are never bound together

/** @brief The sphere sums of one bone, so that the serial and the cooperative merge finish the same arithmetic. */
struct HairContactPartial
{
	float weight;
	float3 normalSum;
	float3 positionSum;
	float deepest;
	int collider;
};

HairContactPartial EmptyContactPartial()
{
	HairContactPartial result;
	result.weight = 0.0f;
	result.normalSum = float3(0.0f, 0.0f, 0.0f);
	result.positionSum = float3(0.0f, 0.0f, 0.0f);
	result.deepest = 0.0f;
	result.collider = -1;
	return result;
}

/** @brief Folds a bone's sphere sums into its merged contact. */
HairBoneContact FinishBoneContact(HairContactPartial a_partial)
{
	HairBoneContact result;
	result.depth = 0.0f;
	result.deepest = a_partial.deepest;
	result.collider = a_partial.collider;
	result.normal = float3(0.0f, 0.0f, 0.0f);
	result.position = float3(0.0f, 0.0f, 0.0f);
	float length = sqrt(dot(a_partial.normalSum, a_partial.normalSum));
	if (a_partial.weight > 0.0f && length > 0.0f) {
		result.normal = a_partial.normalSum / length;
		result.position = a_partial.positionSum / a_partial.weight;
		result.depth = length / a_partial.weight;
	}
	return result;
}

/** @brief One sphere's deepest contact at the start of the substep, written by the sphere pass for the reduce pass. */
struct HairSphereHit
{
	float depth;
	float3 normal;
	float3 centre;
	int collider;
};

RWStructuredBuffer<HairSphereHit> HairSphereHits : register(u5);  // per sphere list entry
RWStructuredBuffer<uint4> HairBoneCull : register(u6);            // per bone slot: x the surfaces its bound reaches, y a kCull state, z the colliders it touches

static const uint kCullSkip = 0u;   // the bone takes no contact this substep, so its merged contact is left as it was
static const uint kCullEmpty = 1u;  // the bone merges to no contact: its spheres cannot reach anything
static const uint kCullWork = 2u;   // the bone's spheres are queried

/**
 * @brief The colliders a bone's bound touches, one bit per collider index below kMaximumExemptColliders.
 * @param a_any Whether any collider at all, past the mask's reach included, is touched.
 */
uint ColliderBoundMask(HairTreeStatic a_tree, float3 a_centre, float a_reach, out bool a_any)
{
	a_any = false;
	uint mask = 0u;
	if (!HairChainSolver::IsFiniteVector(a_centre) || !isfinite(a_reach))
		return mask;
	uint driverFirst = a_tree.contact.z;
	uint driverCount = a_tree.contact.w;
	for (uint index = 0; index < a_tree.contact.y; ++index) {
		HairCollider raw = HairColliders[a_tree.contact.x + index];
		if (raw.kind.y < driverFirst || raw.kind.y >= driverFirst + driverCount)
			continue;
		HairDriverWorld rawWorld = HairDrivers[raw.kind.y];
		HairChainSolver::Transform world = LoadTransform(rawWorld.rotation, rawWorld.translation);
		if (!HairChainSolver::IsFiniteTransform(world))
			continue;
		HairChainSolver::Contact touched = HairChainSolver::NoContact();
		HairChainSolver::MergeContact(touched, LoadCollider(raw), world, a_centre, a_reach, (int)index);
		if (touched.depth > 0.0f) {
			a_any = true;
			if (index < HairChainSolver::kMaximumExemptColliders)
				mask |= 1u << index;
		}
	}
	return mask;
}

/**
 * @brief Whether a bone's bound reaches any collider or surface at all, the prologue the contact pass shares.
 * @details The bound encloses every sphere and reaches past any of them, so a bound that touches nothing means no
 *          member sphere can touch anything either.
 * @param a_surfaceMask The surfaces whose grid the bound overlaps, one bit per surface index up to 32.
 * @param a_colliderMask The colliders the bound touches, one bit per collider index up to 32.
 */
bool BoneBoundReaches(HairTreeStatic a_tree, uint4 a_surface, HairBoneSphere a_bone, HairChainSolver::Transform a_pose, float a_skin,
	out uint a_surfaceMask, out uint a_colliderMask)
{
	a_surfaceMask = 0u;
	float3 boundCentre = a_pose.translation + HairChainSolver::QuatRotate(a_pose.rotation, a_bone.bound.xyz);
	for (uint surface = 0; surface < a_surface.y && surface < 32u; ++surface) {
		HairSurfaceMeta meta = HairSurfaces[a_surface.x + surface];
		if (meta.base.z == 0u)
			continue;
		float deepPush = meta.contact.y > 0.0f ? meta.contact.z : 0.0f;
		float reach = a_bone.bound.w + a_skin + meta.contact.x + abs(meta.contact.y) + deepPush;
		if (BoxOverlapsNode(HairSurfaceNodes[meta.base.y], boundCentre - float3(reach, reach, reach), boundCentre + float3(reach, reach, reach)))
			a_surfaceMask |= 1u << surface;
	}
	float boundReach = a_bone.bound.w * 1.25f + a_skin + HairChainSolver::kBoundCullPad;
	bool anyCollider;
	a_colliderMask = ColliderBoundMask(a_tree, boundCentre, boundReach, anyCollider);
	return a_surfaceMask != 0u || anyCollider || a_surface.y > 32u;
}

/** @brief Longest dynamic parent chain HangsFrom follows. */
static const uint kMaxTreeContactBones = 64u;  // keep equal to GpuRig::kMaxTreeBones

/** @brief Whether a_below hangs, through dynamic parents only, from a_above. */
bool HangsFrom(uint a_below, uint a_above)
{
	uint cursor = a_below;
	for (uint guard = 0; guard < kMaxTreeContactBones; ++guard) {
		HairBoneStatic bone = HairBoneStatics[cursor];
		if (bone.topology.y != 0u)
			return false;
		cursor = bone.topology.x;
		if (cursor == a_above)
			return true;
	}
	return false;
}

/**
 * @brief Reverts the axes a bone's contact pushed further in once its depth passed the allowance, the port of the
 *        post-update clamp in ChainSolver::AdvanceBone, which reads the contact taken at the start of the substep.
 */
void ApplyContactClamp(HairTreeStatic a_tree, uint a_bone, HairChainSolver::JointFrame a_frame,
	HairChainSolver::SolverScales a_scales, inout HairChainSolver::BoneState a_state,
	HairChainSolver::BoneAdvance a_advance, inout float3 a_worldTranslation)
{
	HairBoneContact contact = HairBoneContactsIn[a_bone];
	if (contact.collider < 0)
		return;
	float allowance = (uint)contact.collider < a_tree.contact.y ? HairColliders[a_tree.contact.x + (uint)contact.collider].normal.w : HairChainSolver::kSurfacePenetration;
	if (contact.depth > allowance + a_scales.contactSkin)
		HairChainSolver::ClampBoneContact(a_state, a_frame, a_advance, contact.position - a_advance.pivot, contact.normal, a_scales, a_worldTranslation);
}

HairChainSolver::BoneState LoadBoneState(HairBoneState a_state)
{
	HairChainSolver::BoneState result;
	result.rotation = LoadQuat(a_state.rotation);
	result.angularVelocity = a_state.angularVelocity.xyz;
	result.previousPivot = a_state.previousPivot.xyz;
	result.previousPivotVelocity = a_state.previousPivotVelocity.xyz;
	result.previousParentRotation = LoadQuat(a_state.previousParentRotation);
	result.hasHistory = a_state.history.x != 0u;
	result.hasVelocityHistory = a_state.history.y != 0u;
	result.limitsFreed = a_state.history.z != 0u;
	return result;
}

void StoreBoneState(uint a_bone, HairChainSolver::BoneState a_state, float3 a_worldTranslation)
{
	HairBoneState state;
	state.rotation = StoreQuat(a_state.rotation);
	state.angularVelocity = float4(a_state.angularVelocity, 0.0f);
	state.previousPivot = float4(a_state.previousPivot, 0.0f);
	state.previousPivotVelocity = float4(a_state.previousPivotVelocity, 0.0f);
	state.previousParentRotation = StoreQuat(a_state.previousParentRotation);
	state.history = uint4(a_state.hasHistory ? 1u : 0u, a_state.hasVelocityHistory ? 1u : 0u, a_state.limitsFreed ? 1u : 0u, 0u);
	HairBoneStates[a_bone] = state;

	HairBoneWorld world;
	world.rotation = StoreQuat(a_state.rotation);
	world.translation = float4(a_worldTranslation, 0.0f);
	HairBoneWorlds[a_bone] = world;
}

HairChainSolver::Transform LoadParentPose(HairBoneStatic a_bone, HairChainSolver::Transform a_kinematicPose)
{
	HairChainSolver::Transform result = a_kinematicPose;
	if (a_bone.topology.y == 0u) {
		HairBoneWorld parentWorld = HairBoneWorlds[a_bone.topology.x];
		result.rotation = LoadQuat(parentWorld.rotation);
		result.translation = parentWorld.translation.xyz;
	}
	return result;
}

/** @brief Returns every bone of a tree to its rest relationship with its parent, parent first. */
void ResetTree(HairTreeStatic a_tree, HairChainSolver::Transform a_kinematicPose)
{
	uint offset = a_tree.range.x;
	uint count = a_tree.range.y;
	for (uint index = 0; index < count; ++index) {
		uint bone = HairOrder[offset + index];
		HairBoneStatic rawBone = HairBoneStatics[bone];
		HairChainSolver::Transform restPose = HairChainSolver::ResetTransform(LoadParentPose(rawBone, a_kinematicPose), LoadJointFrame(rawBone));
		StoreBoneState(bone, HairChainSolver::ResetBoneState(restPose), restPose.translation);
	}
}

/** @brief Writes every bone of a tree's root-relative pose, which is what the CPU applies to the nodes. */
void WriteTreeOutput(HairTreeStatic a_tree, HairChainSolver::Transform a_kinematicPose, bool a_failed)
{
	HairChainSolver::Transform rootInverse = HairChainSolver::TransformInverse(a_kinematicPose);
	uint offset = a_tree.range.x;
	uint count = a_tree.range.y;
	for (uint index = 0; index < count; ++index) {
		uint bone = HairOrder[offset + index];
		HairBoneWorld rawWorld = HairBoneWorlds[bone];
		HairChainSolver::Transform relative = HairChainSolver::TransformMul(rootInverse, LoadTransform(rawWorld.rotation, rawWorld.translation));
		HairBoneOutput output;
		output.rotation = StoreQuat(relative.rotation);
		output.translation = float4(relative.translation, 0.0f);
		output.status = uint4(a_failed ? 1u : 0u, 0u, 0u, 0u);
		HairBoneOutputs[bone] = output;
	}
}

/** @brief Clears a tree's failure flag and, when a reset was requested this frame, returns it to rest before its first substep. */
[numthreads(64, 1, 1)] void ResetRequested(uint3 a_dispatchThreadId : SV_DispatchThreadID) {
	uint tree = a_dispatchThreadId.x;
	if (tree >= HairTreeCount)
		return;
	HairTreeFlags[tree] = 0u;
	HairTreeStatic treeStatic = HairTrees[tree];
	if (treeStatic.range.y == 0u)
		return;
	HairTreeInput input = HairTreeInputs[tree];
	if (input.control.z == 0u)
		return;
	ResetTree(treeStatic, LoadTransform(input.currentRotation, input.currentTranslation));
}

	/**
 * @brief Decides, per bone, whether its spheres can touch anything this substep and which surfaces they can reach.
 * @details The sphere pass reads the verdict, so a bone nothing can touch costs one thread rather than a thread per sphere.
 */
	[numthreads(64, 1, 1)] void ContactCull(uint3 a_dispatchThreadId : SV_DispatchThreadID)
{
	uint bone = a_dispatchThreadId.x;
	if (bone >= HairBoneCount)
		return;
	HairBoneCull[bone] = uint4(0u, kCullSkip, 0u, 0u);
	uint tree = HairBoneTrees[bone];
	if (tree == 0xFFFFFFFFu)
		return;
	HairTreeStatic treeStatic = HairTrees[tree];
	HairTreeInput input = HairTreeInputs[tree];
	const bool rigidTree = input.surface.z != 0u;
	const bool ownsTree = HairContactOwner != 0u ? rigidTree : (!rigidTree && input.control.x != 0u && HairSubStep <= input.control.x);
	if ((treeStatic.contact.y == 0u && input.surface.y == 0u) || !ownsTree || HairTreeFlags[tree] != 0u)
		return;
	HairBoneSphere sphereRange = HairBoneSpheres[bone];
	uint surfaceMask = 0u;
	uint colliderMask = 0u;
	uint state = kCullEmpty;
	if (sphereRange.range.y != 0u) {
		HairBoneWorld world = HairBoneWorlds[bone];
		HairChainSolver::Transform pose = LoadTransform(world.rotation, world.translation);
		if (BoneBoundReaches(treeStatic, input.surface, sphereRange, pose, LoadSolverScales(input).contactSkin, surfaceMask, colliderMask))
			state = kCullWork;
	}
	HairBoneCull[bone] = uint4(surfaceMask, state, colliderMask, 0u);
}

/** @brief Queries one sphere of a bone ContactCull kept, a thread per sphere so the work is the same however the bones are sized. */
[numthreads(64, 1, 1)] void ContactSpheres(uint3 a_dispatchThreadId : SV_DispatchThreadID) {
	uint index = a_dispatchThreadId.x;
	if (index >= HairSphereCount)
		return;
	HairSphereRecord sphere = HairSphereList[index];
	uint bone = sphere.exempt.y;
	uint4 cull = HairBoneCull[bone];
	if (cull.y != kCullWork)
		return;
	uint tree = HairBoneTrees[bone];
	HairTreeInput input = HairTreeInputs[tree];
	HairBoneWorld world = HairBoneWorlds[bone];
	HairChainSolver::Transform pose = LoadTransform(world.rotation, world.translation);
	float3 centre = pose.translation + HairChainSolver::QuatRotate(pose.rotation, sphere.centreRadius.xyz);
	HairChainSolver::Contact contact = QueryContacts(HairTrees[tree], input.surface, cull.x, cull.z, centre,
		sphere.centreRadius.w + LoadSolverScales(input).contactSkin, sphere.exempt.x);
	HairSphereHit hit;
	hit.depth = contact.depth > 0.0f ? contact.depth : 0.0f;
	hit.normal = contact.normal;
	hit.centre = centre;
	hit.collider = contact.collider;
	HairSphereHits[index] = hit;
}

	/** @brief Merges each bone's sphere hits into the one contact the chain pass reads, the port of ChainSolver::BoneContact. */
	[numthreads(64, 1, 1)] void ContactReduce(uint3 a_dispatchThreadId : SV_DispatchThreadID)
{
	uint bone = a_dispatchThreadId.x;
	if (bone >= HairBoneCount)
		return;
	uint4 cull = HairBoneCull[bone];
	if (cull.y == kCullSkip)
		return;
	HairContactPartial partial = EmptyContactPartial();
	if (cull.y == kCullWork) {
		HairBoneSphere sphereRange = HairBoneSpheres[bone];
		for (uint index = 0u; index < sphereRange.range.y; ++index) {
			HairSphereHit hit = HairSphereHits[sphereRange.range.x + index];
			if (!(hit.depth > 0.0f))
				continue;
			if (partial.weight > 0.0f && dot(partial.normalSum, hit.normal) < 0.0f)
				continue;
			float squared = hit.depth * hit.depth;
			partial.weight += squared;
			partial.normalSum += hit.normal * (squared * hit.depth);
			partial.positionSum += hit.centre * squared;
			if (hit.depth > partial.deepest) {
				partial.deepest = hit.depth;
				partial.collider = hit.collider;
			}
		}
	}
	HairBoneContactsOut[bone] = FinishBoneContact(partial);
}

/** @brief Advances one tree by one substep, a thread per tree, from the contacts the contact pass took at its start. */
[numthreads(64, 1, 1)] void Step(uint3 a_dispatchThreadId : SV_DispatchThreadID) {
	uint tree = a_dispatchThreadId.x;
	if (tree >= HairTreeCount)
		return;
	HairTreeStatic treeStatic = HairTrees[tree];
	uint orderCount = treeStatic.range.y;
	if (orderCount == 0u)
		return;
	HairTreeInput input = HairTreeInputs[tree];
	// A zero substep count means the frame was not stepped, so the tree keeps its state and pose; a tree that
	// failed on an earlier substep was already reset and written.
	uint subSteps = input.control.x;
	if (subSteps == 0u || HairSubStep > subSteps || HairTreeFlags[tree] != 0u)
		return;

	HairChainSolver::Transform current = LoadTransform(input.currentRotation, input.currentTranslation);
	HairChainSolver::Transform previous = LoadTransform(input.previousRotation, input.previousTranslation);
	// A reset clears the solver's previous input, so both of those force the substeps to
	// interpolate from the current pose rather than the last frame's.
	bool resetRequested = input.control.z != 0u;
	if (resetRequested || input.control.y == 0u)
		previous = current;

	bool failed = false;
	HairChainSolver::SolverScales scales = LoadSolverScales(input);
	float step = input.solver.x;
	uint orderOffset = treeStatic.range.x;
	bool collides = treeStatic.contact.y != 0u || input.surface.y != 0u;

	float amount = (float)HairSubStep / (float)subSteps;
	HairChainSolver::Transform kinematic;
	kinematic.rotation = HairChainSolver::Slerp(previous.rotation, current.rotation, amount);
	kinematic.translation = previous.translation + (current.translation - previous.translation) * amount;

	// Each bone's wind force at the substep's start, so a joint can sum the load below it without repeating the gust maths.
	bool windActive = scales.windFactor > 0.0f && dot(scales.wind, scales.wind) > 0.0f;
	uint windCount = min(orderCount, kMaxTreeContactBones);
	float3 windForces[kMaxTreeContactBones];
	float3 windMoments[kMaxTreeContactBones];
	if (windActive) {
		for (uint windIndex = 0; windIndex < windCount; ++windIndex) {
			uint windBone = HairOrder[orderOffset + windIndex];
			HairBoneStatic windRaw = HairBoneStatics[windBone];
			HairChainSolver::BoneState windState = LoadBoneState(HairBoneStates[windBone]);
			HairChainSolver::JointFrame windFrame = LoadJointFrame(windRaw);
			HairChainSolver::BoneParameters windParameters = LoadBoneParameters(windRaw);
			float3 position = HairBoneWorlds[windBone].translation.xyz;
			float3 centreOffset = -HairChainSolver::QuatRotate(windState.rotation, windFrame.originA);
			float3 velocity = windState.previousPivotVelocity + cross(windState.angularVelocity, centreOffset);
			float3 force = HairChainSolver::WindForce(scales, windParameters, windParameters.windFactor * scales.windFactor,
				HairChainSolver::QuatRotate(windState.rotation, windParameters.windAxis), position, velocity);
			windForces[windIndex] = force;
			windMoments[windIndex] = cross(position, force);
		}
	}

	for (uint index = 0; index < orderCount && !failed; ++index) {
		uint bone = HairOrder[orderOffset + index];
		HairBoneStatic rawBone = HairBoneStatics[bone];
		HairChainSolver::BoneState state = LoadBoneState(HairBoneStates[bone]);
		HairChainSolver::JointFrame frame = LoadJointFrame(rawBone);
		HairChainSolver::Transform parentPose = LoadParentPose(rawBone, kinematic);
		// Contact terms are gathered before the update.
		HairChainSolver::Quat jointRotation;
		float3 pivot;
		HairChainSolver::JointPose(parentPose, frame, jointRotation, pivot);
		HairChainSolver::ContactTerms terms = HairChainSolver::NoContactTerms();
		if (collides) {
			// A contact anywhere below a joint loads that joint; each is the one taken at the start of the substep.
			for (uint below = index; below < orderCount; ++below) {
				uint belowBone = HairOrder[orderOffset + below];
				if (below != index && !HangsFrom(belowBone, bone))
					continue;
				HairBoneContact merged = HairBoneContactsIn[belowBone];
				if (!(merged.depth > 0.0f))
					continue;
				HairChainSolver::Contact contact;
				contact.depth = merged.depth;
				contact.normal = merged.normal;
				contact.collider = merged.collider;
				HairChainSolver::AddContactTorque(terms, scales, jointRotation, merged.position - pivot, contact);
			}
		}

		if (input.control.w != 0u) {
			// The bones below a joint add their weight, acceleration load and inertia to it, from their poses at the substep's start.
			float3 gravityVector = float3(0.0f, 0.0f, -scales.gravity);
			for (uint below = index + 1u; below < orderCount; ++below) {
				uint belowBone = HairOrder[orderOffset + below];
				if (!HangsFrom(belowBone, bone))
					continue;
				HairBoneStatic belowRaw = HairBoneStatics[belowBone];
				HairChainSolver::BoneParameters belowBody = LoadBoneParameters(belowRaw);
				float3 offset = HairBoneWorlds[belowBone].translation.xyz - pivot;
				terms.subtreeGravityTorque += cross(offset, gravityVector * (belowBody.mass * belowBody.gravityFactor * scales.gravityScale));
				terms.subtreeMassMoment += offset * belowBody.mass;
				terms.subtreeInertia += belowBody.inertiaMean + belowBody.mass * dot(offset, offset);
			}
		}

		bool heldBelow = false;
		[unroll] for (uint pin = 0u; pin < 4u; ++pin)
		{
			// A hand holds a point of a bone: each pin pulls like a contact, and loads every joint above it the way a contact below does.
			uint slotPlusOne = pin == 0u ? input.grabSlots.x : (pin == 1u ? input.grabSlots.y : (pin == 2u ? input.grabSlots.z : input.grabSlots.w));
			if (slotPlusOne == 0u || !(input.grabTargets[pin].w > 0.0f))
				continue;
			uint heldBone = slotPlusOne - 1u;
			if (heldBone != bone && !HangsFrom(heldBone, bone))
				continue;
			heldBelow = true;
			HairBoneWorld heldWorld = HairBoneWorlds[heldBone];
			float3 held = heldWorld.translation.xyz + HairChainSolver::QuatRotate(LoadQuat(heldWorld.rotation), input.grabLocals[pin].xyz);
			HairChainSolver::Contact pull = HairChainSolver::GrabPull(held, input.grabTargets[pin].xyz);
			if (pull.depth > 0.0f) {
				HairChainSolver::SolverScales pinScales = scales;
				pinScales.contactStiffness = input.grabTargets[pin].w;
				HairChainSolver::AddContactTorque(terms, pinScales, jointRotation, held - pivot, pull);
			}
		}

		float3 windForce = float3(0.0f, 0.0f, 0.0f);
		float3 windMoment = float3(0.0f, 0.0f, 0.0f);
		if (windActive && index < windCount) {
			windForce = windForces[index];
			windMoment = windMoments[index];
			if (scales.windChainLoad) {
				for (uint below = index + 1u; below < windCount; ++below) {
					if (!HangsFrom(HairOrder[orderOffset + below], bone))
						continue;
					windForce += windForces[below];
					windMoment += windMoments[below];
				}
			}
		}

		HairChainSolver::BoneAdvance advance = (HairChainSolver::BoneAdvance)0;
		float3 worldTranslation = float3(0.0f, 0.0f, 0.0f);
		HairChainSolver::JointSprings springs = LoadJointSprings(rawBone);
		if (heldBelow) {
			// A lower limit above the upper one switches the clamp off: a held strand frees the joints above it.
			springs.lower = float3(1.0f, 1.0f, 1.0f);
			springs.upper = float3(-1.0f, -1.0f, -1.0f);
		}
		bool easeBack = state.limitsFreed && !heldBelow;
		bool finite = HairChainSolver::AdvanceBone(state, LoadBoneParameters(rawBone), frame,
			springs, parentPose, scales, step, terms, windForce, windMoment, easeBack, advance, worldTranslation);
		failed = !finite;
		if (failed)
			break;
		state.limitsFreed = heldBelow || advance.beyondLimits;

		if (collides)
			ApplyContactClamp(treeStatic, bone, frame, scales, state, advance, worldTranslation);
		StoreBoneState(bone, state, worldTranslation);
	}

	if (failed) {
		ResetTree(treeStatic, current);
		WriteTreeOutput(treeStatic, current, true);
		HairTreeFlags[tree] = 1u;
	} else if (HairSubStep == subSteps) {
		WriteTreeOutput(treeStatic, current, false);
	}
}

/** @brief The kinematic root pose a tree's substep sees: the previous frame's pose turned toward the current one. */
HairChainSolver::Transform SubstepKinematic(HairTreeInput a_input, uint a_subSteps)
{
	HairChainSolver::Transform current = LoadTransform(a_input.currentRotation, a_input.currentTranslation);
	HairChainSolver::Transform previous = LoadTransform(a_input.previousRotation, a_input.previousTranslation);
	if (a_input.control.z != 0u || a_input.control.y == 0u)
		previous = current;
	float amount = (float)HairSubStep / (float)a_subSteps;
	HairChainSolver::Transform result;
	result.rotation = HairChainSolver::Slerp(previous.rotation, current.rotation, amount);
	result.translation = previous.translation + (current.translation - previous.translation) * amount;
	return result;
}

/** @brief The shortest rotation taking a_from's direction to a_to's, the identity when either is degenerate or they are opposite. */
HairChainSolver::Quat SwingBetween(float3 a_from, float3 a_to)
{
	float lengths = length(a_from) * length(a_to);
	float3 axis = cross(a_from, a_to);
	float w = dot(a_from, a_to) + lengths;
	if (!(lengths > 1.0e-6f) || !(w > 1.0e-6f * lengths))
		return HairChainSolver::QuatIdentity();
	HairChainSolver::Quat result;
	result.w = w;
	result.x = axis.x;
	result.y = axis.y;
	result.z = axis.z;
	return HairChainSolver::QuatNormalized(result);
}

static const uint kMaxTieProjectBones = 128u;
static const uint kTieProjectIterations = 4u;
static const float kTieProjectMinimumCorrection = 1.0e-3f;

/**
 * @brief Pulls the bones a rig's ties join back inside their authored windows after a substep, one thread per claim.
 * @details A position-based pass: every tie moves its two bones apart or together in proportion to their inverse
 *          masses until the child sits inside the window its parent's frame allows, then each tree is refitted
 *          root first, swinging every bone about its own pivot toward its corrected position inside its joint
 *          limits. The correction adds no velocity, so a window the chain solver keeps pressing against does not ring. The claim's first
 *          tree carries the claim's tree count and its input the bone span, as GpuRigPacking packs them.
 */
[numthreads(64, 1, 1)] void TieProject(uint3 a_dispatchThreadId : SV_DispatchThreadID) {
	uint firstTree = a_dispatchThreadId.x;
	if (firstTree >= HairTreeCount)
		return;
	uint treeCount = HairTrees[firstTree].range.w;
	HairTreeInput firstInput = HairTreeInputs[firstTree];
	uint span = firstInput.surface.w;
	if (treeCount == 0u || span == 0u || span > kMaxTieProjectBones)
		return;
	uint subSteps = firstInput.control.x;
	HairChainSolver::SolverScales scales = LoadSolverScales(firstInput);
	if (!(scales.tieStiffness > 0.0f) || subSteps == 0u || HairSubStep > subSteps)
		return;
	for (uint check = 0u; check < treeCount; ++check) {
		if (HairTreeFlags[firstTree + check] != 0u)
			return;
	}

	float3 original[kMaxTieProjectBones];
	float3 position[kMaxTieProjectBones];
	float4 oldRotation[kMaxTieProjectBones];
	for (uint load = 0u; load < span; ++load) {
		HairBoneWorld world = HairBoneWorlds[firstTree + load];
		original[load] = world.translation.xyz;
		position[load] = world.translation.xyz;
		oldRotation[load] = world.rotation;
	}

	for (uint iteration = 0u; iteration < kTieProjectIterations; ++iteration) {
		for (uint child = 0u; child < span; ++child) {
			HairBoneStatic childStatic = HairBoneStatics[firstTree + child];
			[unroll] for (uint tie = 0u; tie < 2u; ++tie)
			{
				uint code = tie == 0u ? childStatic.tieSlots.x : childStatic.tieSlots.y;
				if (code == 0u || (code & 1u) != 0u)
					continue;
				uint parent = (code >> 1u) - 1u - firstTree;
				if (parent >= span)
					continue;
				HairChainSolver::Quat parentRotation = LoadQuat(oldRotation[parent]);
				float3 local = HairChainSolver::QuatRotate(HairChainSolver::QuatConjugate(parentRotation), position[child] - position[parent]) -
				               (tie == 0u ? childStatic.tieOffset0.xyz : childStatic.tieOffset1.xyz);
				float3 excess = local - clamp(local, childStatic.tieLower.xyz, childStatic.tieUpper.xyz);
				if (!(length(excess) > kTieProjectMinimumCorrection))
					continue;
				float childInverseMass = childStatic.body.x > 0.0f ? 1.0f / childStatic.body.x : 0.0f;
				float parentMass = HairBoneStatics[firstTree + parent].body.x;
				float parentInverseMass = parentMass > 0.0f ? 1.0f / parentMass : 0.0f;
				float total = childInverseMass + parentInverseMass;
				if (!(total > 0.0f))
					continue;
				float3 shift = HairChainSolver::QuatRotate(parentRotation, excess);
				position[child] -= shift * (childInverseMass / total);
				position[parent] += shift * (parentInverseMass / total);
			}
		}
	}

	float3 correction[kMaxTieProjectBones];
	float largest = 0.0f;
	for (uint measure = 0u; measure < span; ++measure) {
		correction[measure] = position[measure] - original[measure];
		largest = max(largest, length(correction[measure]));
	}
	if (!(largest > kTieProjectMinimumCorrection))
		return;

	for (uint treeIndex = 0u; treeIndex < treeCount; ++treeIndex) {
		uint tree = firstTree + treeIndex;
		HairTreeStatic treeStatic = HairTrees[tree];
		HairChainSolver::Transform kinematic = SubstepKinematic(HairTreeInputs[tree], subSteps);
		for (uint index = 0u; index < treeStatic.range.y; ++index) {
			uint bone = HairOrder[treeStatic.range.x + index];
			uint local = bone - firstTree;
			HairBoneStatic rawBone = HairBoneStatics[bone];
			HairChainSolver::BoneState state = LoadBoneState(HairBoneStates[bone]);
			HairChainSolver::JointFrame frame = LoadJointFrame(rawBone);
			HairChainSolver::Transform parentPose = LoadParentPose(rawBone, kinematic);
			HairChainSolver::Quat parentTurn = HairChainSolver::QuatIdentity();
			if (rawBone.topology.y == 0u)
				parentTurn = HairChainSolver::QuatMul(parentPose.rotation, HairChainSolver::QuatConjugate(LoadQuat(oldRotation[rawBone.topology.x - firstTree])));
			HairChainSolver::Quat jointRotation;
			float3 pivot;
			HairChainSolver::JointPose(parentPose, frame, jointRotation, pivot);

			HairChainSolver::Quat kept = HairChainSolver::QuatNormalized(HairChainSolver::QuatMul(parentTurn, state.rotation));
			float3 keptCentre = -HairChainSolver::QuatRotate(kept, frame.originA);
			HairChainSolver::Quat turned = HairChainSolver::QuatNormalized(HairChainSolver::QuatMul(SwingBetween(keptCentre, keptCentre + correction[local]), kept));
			float3 angles = HairChainSolver::ToEulerXYZ(HairChainSolver::QuatMul(HairChainSolver::QuatMul(HairChainSolver::QuatConjugate(jointRotation), turned), frame.rotationA));
			HairChainSolver::JointSprings springs = LoadJointSprings(rawBone);
			[unroll] for (uint axis = 0u; axis < 3u; ++axis)
			{
				if (springs.lower[axis] <= springs.upper[axis])
					angles[axis] = clamp(angles[axis], springs.lower[axis], springs.upper[axis]);
			}
			HairChainSolver::Transform pose = HairChainSolver::PoseFor(jointRotation, frame, pivot, angles);
			if (!HairChainSolver::IsFiniteTransform(pose))
				continue;

			state.rotation = pose.rotation;
			state.previousPivot = pivot;
			state.previousParentRotation = parentPose.rotation;
			StoreBoneState(bone, state, pose.translation);
		}
	}

	if (HairSubStep == subSteps) {
		for (uint outputTree = 0u; outputTree < treeCount; ++outputTree) {
			HairTreeInput input = HairTreeInputs[firstTree + outputTree];
			WriteTreeOutput(HairTrees[firstTree + outputTree], LoadTransform(input.currentRotation, input.currentTranslation), false);
		}
	}
}

	[numthreads(64, 1, 1)] void Reset(uint3 a_dispatchThreadId : SV_DispatchThreadID)
{
	uint index = a_dispatchThreadId.x;
	if (index >= HairResetCount)
		return;
	uint tree = HairResetList[index];
	if (tree >= HairTreeCount)
		return;
	HairTreeStatic treeStatic = HairTrees[tree];
	if (treeStatic.range.y == 0u)
		return;
	HairTreeInput input = HairTreeInputs[tree];
	HairChainSolver::Transform current = LoadTransform(input.currentRotation, input.currentTranslation);
	ResetTree(treeStatic, current);
	WriteTreeOutput(treeStatic, current, false);
}
