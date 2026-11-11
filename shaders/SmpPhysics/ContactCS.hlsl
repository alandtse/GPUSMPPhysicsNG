#include "Common/HairCollision.hlsli"

// Per rigid substep ContactCull, ContactSpheres and ContactReduce build each bone's merged contact.
// The record layouts mirror src/Features/SmpPhysics/GpuRigPacking.h; keep them in step.

struct HairTreeStatic
{
	uint4 range;    // x = first order slot, y = order count, z = kinematic parent bone slot
	uint4 contact;  // x = first collider slot, y = collider count, z = first driver slot, w = driver count
};

struct HairBoneSphere
{
	uint4 range;   // x = first sphere in the sphere list, y = how many, w = 1 when z is the mask of surfaces the bone may touch
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

struct HairBoneWorld
{
	float4 rotation;
	float4 translation;
};

StructuredBuffer<HairTreeStatic> HairTrees : register(t1);
StructuredBuffer<HairTreeInput> HairTreeInputs : register(t2);
StructuredBuffer<HairBoneSphere> HairBoneSpheres : register(t5);
StructuredBuffer<HairCollider> HairColliders : register(t6);
StructuredBuffer<HairDriverWorld> HairDrivers : register(t7);
StructuredBuffer<HairSurfaceMeta> HairSurfaces : register(t8);
StructuredBuffer<HairSurfaceTriangle> HairSurfaceTriangles : register(t9);
StructuredBuffer<HairBvhNode> HairSurfaceNodes : register(t10);
StructuredBuffer<HairSphereRecord> HairSphereList : register(t12);
StructuredBuffer<uint> HairBoneTrees : register(t13);  // per bone slot: its tree slot, or 0xFFFFFFFF for a kinematic bone

RWStructuredBuffer<HairBoneWorld> HairBoneWorlds : register(u1);

cbuffer HairContactParams : register(b0)
{
	uint HairBoneCount;    // one past the highest bone slot any claim uses
	uint HairSphereCount;  // entries in the sphere list
};

HairCollision::Quat LoadQuat(float4 a_value)
{
	HairCollision::Quat result;
	result.w = a_value.x;
	result.x = a_value.y;
	result.y = a_value.z;
	result.z = a_value.w;
	return result;
}

HairCollision::Transform LoadTransform(float4 a_rotation, float4 a_translation)
{
	HairCollision::Transform result;
	result.rotation = LoadQuat(a_rotation);
	result.translation = a_translation.xyz;
	return result;
}

/** @brief Unpacks one collider, whose radius and penetration ride in the spare lanes of two of its vectors. */
HairCollision::Collider LoadCollider(HairCollider a_collider)
{
	HairCollision::Collider result;
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
HairCollision::Contact QueryColliders(HairTreeStatic a_tree, float3 a_point, float a_radius, uint a_exemptMask, uint a_candidateMask)
{
	HairCollision::Contact best = HairCollision::NoContact();
	if (!HairCollision::IsFiniteVector(a_point) || !isfinite(a_radius))
		return best;
	uint colliderCount = a_tree.contact.y;
	uint driverFirst = a_tree.contact.z;
	uint driverCount = a_tree.contact.w;
	for (uint index = 0; index < colliderCount; ++index) {
		// A collider the sphere was authored inside is not a contact at all, or its spring would fight the clamp.
		if (index < HairCollision::kMaximumExemptColliders && ((a_exemptMask & (1u << index)) != 0u || (a_candidateMask & (1u << index)) == 0u))
			continue;
		HairCollider raw = HairColliders[a_tree.contact.x + index];
		if (raw.kind.y < driverFirst || raw.kind.y >= driverFirst + driverCount)
			continue;
		HairDriverWorld rawWorld = HairDrivers[raw.kind.y];
		HairCollision::Transform world = LoadTransform(rawWorld.rotation, rawWorld.translation);
		if (!HairCollision::IsFiniteTransform(world))
			continue;
		HairCollision::MergeContact(best, LoadCollider(raw), world, a_point, a_radius, (int)index);
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
	float a_allowance, float a_behind, inout HairCollision::Contact a_best, inout uint a_bestId)
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
HairCollision::Contact QuerySurface(HairSurfaceMeta a_meta, float3 a_point, float a_reach)
{
	HairCollision::Contact best = HairCollision::NoContact();
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
 *        the CPU contact query. A surface's contact index is the collider count plus its record's position.
 */
HairCollision::Contact QueryContacts(HairTreeStatic a_tree, uint4 a_surface, uint a_surfaceMask, uint a_colliderMask, float3 a_point, float a_radius, uint a_exemptMask)
{
	HairCollision::Contact best = QueryColliders(a_tree, a_point, a_radius, a_exemptMask, a_colliderMask);
	if (!HairCollision::IsFiniteVector(a_point) || !isfinite(a_radius))
		return best;
	for (uint index = 0; index < a_surface.y; ++index) {
		if (index < 32u && (a_surfaceMask & (1u << index)) == 0u)
			continue;
		HairSurfaceMeta meta = HairSurfaces[a_surface.x + index];
		HairCollision::Contact contact = QuerySurface(meta, a_point, a_radius + meta.contact.x);
		if (contact.depth > best.depth) {
			best = contact;
			best.collider = (int)(a_tree.contact.y + index);
		}
	}
	return best;
}

/** @brief One bone's contacts merged into one, the port of the CPU BoneContact. */
struct HairBoneContact
{
	float depth;
	float deepest;
	int collider;
	float3 normal;
	float3 position;
};

RWStructuredBuffer<HairBoneContact> HairBoneContactsOut : register(u3);  // Contact's write view: per bone slot, its merged contact at the start of the substep

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
	if (!HairCollision::IsFiniteVector(a_centre) || !isfinite(a_reach))
		return mask;
	uint driverFirst = a_tree.contact.z;
	uint driverCount = a_tree.contact.w;
	for (uint index = 0; index < a_tree.contact.y; ++index) {
		HairCollider raw = HairColliders[a_tree.contact.x + index];
		if (raw.kind.y < driverFirst || raw.kind.y >= driverFirst + driverCount)
			continue;
		HairDriverWorld rawWorld = HairDrivers[raw.kind.y];
		HairCollision::Transform world = LoadTransform(rawWorld.rotation, rawWorld.translation);
		if (!HairCollision::IsFiniteTransform(world))
			continue;
		HairCollision::Contact touched = HairCollision::NoContact();
		HairCollision::MergeContact(touched, LoadCollider(raw), world, a_centre, a_reach, (int)index);
		if (touched.depth > 0.0f) {
			a_any = true;
			if (index < HairCollision::kMaximumExemptColliders)
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
bool BoneBoundReaches(HairTreeStatic a_tree, uint4 a_surface, HairBoneSphere a_bone, HairCollision::Transform a_pose, float a_skin,
	out uint a_surfaceMask, out uint a_colliderMask)
{
	a_surfaceMask = 0u;
	float3 boundCentre = a_pose.translation + HairCollision::QuatRotate(a_pose.rotation, a_bone.bound.xyz);
	for (uint surface = 0; surface < a_surface.y && surface < 32u; ++surface) {
		HairSurfaceMeta meta = HairSurfaces[a_surface.x + surface];
		if (meta.base.z == 0u)
			continue;
		float deepPush = meta.contact.y > 0.0f ? meta.contact.z : 0.0f;
		float reach = a_bone.bound.w + a_skin + meta.contact.x + abs(meta.contact.y) + deepPush;
		if (BoxOverlapsNode(HairSurfaceNodes[meta.base.y], boundCentre - float3(reach, reach, reach), boundCentre + float3(reach, reach, reach)))
			a_surfaceMask |= 1u << surface;
	}
	float boundReach = a_bone.bound.w * 1.25f + a_skin + HairCollision::kBoundCullPad;
	bool anyCollider;
	a_colliderMask = ColliderBoundMask(a_tree, boundCentre, boundReach, anyCollider);
	return a_surfaceMask != 0u || anyCollider || a_surface.y > 32u;
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
	if ((treeStatic.contact.y == 0u && input.surface.y == 0u) || !rigidTree)
		return;
	HairBoneSphere sphereRange = HairBoneSpheres[bone];
	uint surfaceMask = 0u;
	uint colliderMask = 0u;
	uint state = kCullEmpty;
	if (sphereRange.range.y != 0u) {
		HairBoneWorld world = HairBoneWorlds[bone];
		HairCollision::Transform pose = LoadTransform(world.rotation, world.translation);
		if (BoneBoundReaches(treeStatic, input.surface, sphereRange, pose, input.contact.z, surfaceMask, colliderMask)) {
			if (sphereRange.range.w != 0u)
				surfaceMask &= sphereRange.range.z;
			if (surfaceMask != 0u || colliderMask != 0u || input.surface.y > 32u)
				state = kCullWork;
		}
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
	HairCollision::Transform pose = LoadTransform(world.rotation, world.translation);
	float3 centre = pose.translation + HairCollision::QuatRotate(pose.rotation, sphere.centreRadius.xyz);
	HairCollision::Contact contact = QueryContacts(HairTrees[tree], input.surface, cull.x, cull.z, centre,
		sphere.centreRadius.w + input.contact.z, sphere.exempt.x);
	HairSphereHit hit;
	hit.depth = contact.depth > 0.0f ? contact.depth : 0.0f;
	hit.normal = contact.normal;
	hit.centre = centre;
	hit.collider = contact.collider;
	HairSphereHits[index] = hit;
}

	/** @brief Merges each bone's sphere hits into the one contact the rigid solver reads, the port of the CPU BoneContact. */
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
