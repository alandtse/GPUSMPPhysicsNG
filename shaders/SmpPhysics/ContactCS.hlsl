#include "Common/HairCollision.hlsli"

// Per rigid substep ContactCull, ContactSpheres and ContactReduce build each bone's merged contact; ClothVertices and
// ClothTriangles run between the first two when a claim collides its cloth's triangles with a body proxy's vertices.
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
	uint4 exempt;         // x = rest-exemption mask, y = the slot of the bone the sphere belongs to, z = 1 when it leaves per-vertex surfaces to its shape's triangles
	float4 other[4];      // per other skin bone: xyz = the vertex in that bone's rest frame, w = its weight
	uint4 otherBone;      // the other bones' slots, kNoSphereBone when unused
	float4 weights;       // x = the owning bone's weight, y = hdtSMP64's flexible, the depth scale
};
static const uint kNoSphereBone = 0xFFFFFFFFu;  // mirrors kNoSphereBone in GpuRigPacking.h
static const float kPruneSlack = 1.0e-3f;        // keeps a face tied with the best one inside the shrunken search box
static const uint kFarSearchNodeBudget = 256u;  // most BVH nodes the search past a one-sided surface's allowance visits

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
	float4 contact;  // z = contact skin
	uint4 surface;   // x = first surface record of the tree's claim this frame, y = how many, z = 1 when the rigid solver owns the tree, w = the claim's bone span
};

struct HairSurfaceMeta
{
	uint4 dimensions;   // w = face count
	uint4 base;         // x = first face, y = first node, z = node count, w = 1 when built from a per-vertex shape
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
struct HairDriverDelta
{
	float4 rotation;     // w, x, y, z lane order: the map from the driver's pose at the last step to its pose now
	float4 translation;  // xyz translation, w = 1 / the step's seconds, 0 when the driver has no delta
};

StructuredBuffer<HairDriverDelta> HairDriverDeltas : register(t14);
StructuredBuffer<uint> HairBoneTrees : register(t13);  // per bone slot: its tree slot, or kNoTree for a kinematic bone
static const uint kNoTree = 0xFFFFFFFFu;  // mirrors kNoTree in GpuRigPacking.h
static const uint kNoTriangle = 0xFFFFFFFFu;  // no face has offered itself yet

struct HairClothTriangle
{
	uint4 corners;   // xyz = the corners in the cloth vertex list, w = the slot of a simulated bone that owns the triangle
	float4 contact;  // x = the margin, y = the signed penetration, z = the surfaces it may touch as the bits of a uint
};

struct HairClothBoneEntry
{
	uint clothTriangle;
	float flexible;  // scales the depth
	uint corners;    // how many times the contact is added
	uint pad;
};

StructuredBuffer<HairSphereRecord> HairClothVertices : register(t0);  // a cloth vertex in a sphere's blend layout; its radius is unused
StructuredBuffer<HairClothTriangle> HairClothTriangleList : register(t3);
StructuredBuffer<uint4> HairBoneCloth : register(t4);  // per bone slot: x = first entry in the cloth bone list, y = how many
StructuredBuffer<HairClothBoneEntry> HairClothBoneList : register(t11);

RWStructuredBuffer<float4> HairClothPosed : register(u0);  // per cloth vertex, its world position this substep
RWStructuredBuffer<HairBoneWorld> HairBoneWorlds : register(u1);

cbuffer HairContactParams : register(b0)
{
	uint HairBoneCount;           // one past the highest bone slot any claim uses
	uint HairSphereCount;         // entries in the sphere list
	uint HairClothVertexCount;    // entries in the cloth vertex list
	uint HairClothTriangleCount;  // entries in the cloth triangle list; 0 leaves the cloth buffers unread
};

/** @brief Unpacks one collider, whose radius and penetration ride in the spare lanes of two of its vectors. */
HairCollision::Collider LoadCollider(HairCollider a_collider)
{
	HairCollision::Collider result;
	result.kind = (int)a_collider.kind.x;
	result.orientation = HairCollision::LoadQuat(a_collider.orientation);
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
		HairCollision::Transform world = HairCollision::LoadTransform(rawWorld.rotation, rawWorld.translation);
		if (!HairCollision::IsFiniteTransform(world))
			continue;
		const float depthBefore = best.depth;
		HairCollision::MergeContact(best, LoadCollider(raw), world, a_point, a_radius, (int)index);
		if (best.depth > depthBefore) {
			best.driver = raw.kind.y + 1u;
			best.driverOther = 0u;
		}
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
	float a_allowance, float a_behind, inout HairCollision::Contact a_best, inout float a_bestRank, inout uint a_bestId)
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
	float rank = -abs(distance);
	if (rank < a_bestRank || (rank == a_bestRank && a_id >= a_bestId))
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
	a_best.driver = asuint(a_face.b.w);
	a_best.driverOther = asuint(a_face.c.w);
	a_bestRank = rank;
	a_bestId = a_id;
}

/**
 * @brief The deepest contact a sphere makes with one posed surface, the port of TriangleGrid::Query in TriangleGrid.cpp, enumerating through a BVH.
 *        Only its second, deep search is capped by kFarSearchNodeBudget; TriangleGrid::Query and ProbeSpheres are not.
 * @param a_reach The sphere's radius plus the surface margin.
 */
HairCollision::Contact QuerySurface(HairSurfaceMeta a_meta, float3 a_point, float a_reach)
{
	HairCollision::Contact best = HairCollision::NoContact();
	float penetration = a_meta.contact.y;
	bool flipped = penetration < 0.0f;
	float allowance = abs(penetration);
	// Only a one-sided face has an inside a sphere can be stuck in, so only it is searched again past its allowance.
	bool deep = penetration > 0.0f && a_meta.contact.z > 0.0f;
	uint bestId = kNoTriangle;
	float bestRank = -1.0e30f;
	for (uint turn = 0u; turn < (deep ? 2u : 1u) && bestId == kNoTriangle; ++turn) {
		float behind = turn == 0u ? allowance : allowance + a_meta.contact.z;
		float extent = max(a_reach, behind);
		float3 low = a_point - float3(extent, extent, extent);
		float3 high = a_point + float3(extent, extent, extent);
		// A sphere in front of the surface accepts no face and would walk the whole deep box; one behind a face accepts early and the box shrink ends its walk.
		uint budget = turn == 0u ? 0xFFFFFFFFu : kFarSearchNodeBudget;
		uint visited = 0u;
		uint node = 0u;
		while (node < a_meta.base.z && visited < budget) {
			visited = visited + 1u;
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
					OfferSurfaceFace(face, asuint(face.a.w), a_point, a_reach, flipped, allowance, behind, best, bestRank, bestId);
				}
				// A face can only replace the best one by lying no further from the point, so the search box shrinks to that distance.
				if (-bestRank < extent) {
					extent = -bestRank + kPruneSlack;
					low = a_point - float3(extent, extent, extent);
					high = a_point + float3(extent, extent, extent);
				}
				node = current.skip;
			} else {
				node = node + 1u;
			}
		}
	}
	return best;
}

/**
 * @brief The deepest contact a sphere makes with the claim's colliders and posed surfaces, the port of
 *        the CPU contact query. A surface's contact index is the collider count plus its record's position.
 */
HairCollision::Contact QueryContacts(HairTreeStatic a_tree, uint4 a_surface, uint a_surfaceMask, uint a_colliderMask, float3 a_point, float a_radius, uint a_exemptMask,
	bool a_skipVertexSurfaces)
{
	HairCollision::Contact best = QueryColliders(a_tree, a_point, a_radius, a_exemptMask, a_colliderMask);
	if (!HairCollision::IsFiniteVector(a_point) || !isfinite(a_radius))
		return best;
	for (uint index = 0; index < a_surface.y; ++index) {
		if (index < HairCollision::kMaskedSurfaceLimit && (a_surfaceMask & (1u << index)) == 0u)
			continue;
		HairSurfaceMeta meta = HairSurfaces[a_surface.x + index];
		if (a_skipVertexSurfaces && meta.base.w != 0u)
			continue;
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
	float3 velocity;
	uint velocityValid;
};

static const uint kContactLanes = 4u;  // mirrors kContactLanes in Collision.h

RWStructuredBuffer<HairBoneContact> HairBoneContactsOut : register(u3);  // Contact's write view: kContactLanes per bone slot, its contacts at the start of the substep, one per contacted driver

/** @brief The sphere sums of one bone, so that the serial and the cooperative merge finish the same arithmetic. */
struct HairContactPartial
{
	float weight;
	float3 normalSum;
	float3 positionSum;
	float3 velocitySum;
	bool velocityMissing;
	float deepest;
	int collider;
};

HairContactPartial EmptyContactPartial()
{
	HairContactPartial result;
	result.weight = 0.0f;
	result.normalSum = float3(0.0f, 0.0f, 0.0f);
	result.positionSum = float3(0.0f, 0.0f, 0.0f);
	result.velocitySum = float3(0.0f, 0.0f, 0.0f);
	result.velocityMissing = false;
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
	result.velocity = float3(0.0f, 0.0f, 0.0f);
	result.velocityValid = 0u;
	float length = sqrt(dot(a_partial.normalSum, a_partial.normalSum));
	if (a_partial.weight > 0.0f && length > 0.0f) {
		result.normal = a_partial.normalSum / length;
		result.position = a_partial.positionSum / a_partial.weight;
		result.depth = length / a_partial.weight;
		if (!a_partial.velocityMissing) {
			result.velocity = a_partial.velocitySum / a_partial.weight;
			result.velocityValid = 1u;
		}
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
	float4 velocityValid;  // xyz the contacted surface point's velocity, w 1 when its driver is known
	uint key;              // the contacted driver's slot plus one, 0 when unknown; hits of one key merge into one lane
	uint keyOther;         // a second driver the hit face is weighted to, 0 when none: hdtSMP64 credits a hit to every bone of the body vertex
};

/** @brief The velocity of a point a driver carries, w 1 when the driver is known and has a delta. */
float4 DriverPointVelocity(uint a_driverPlusOne, float3 a_point)
{
	if (a_driverPlusOne == 0u)
		return float4(0.0f, 0.0f, 0.0f, 0.0f);
	HairDriverDelta delta = HairDriverDeltas[a_driverPlusOne - 1u];
	if (!(delta.translation.w > 0.0f))
		return float4(0.0f, 0.0f, 0.0f, 0.0f);
	HairCollision::Transform map = HairCollision::LoadTransform(delta.rotation, delta.translation);
	return float4((map.translation + HairCollision::QuatRotate(map.rotation, a_point) - a_point) * delta.translation.w, 1.0f);
}

static const uint kClothTriangleHits = 2u;  // mirrors kClothTriangleHits in ClothTriangles.h

RWStructuredBuffer<HairSphereHit> HairClothHits : register(u2);   // kClothTriangleHits per cloth triangle: centre is the contact point on the triangle
RWStructuredBuffer<HairSphereHit> HairSphereHits : register(u5);  // per sphere list entry
RWStructuredBuffer<uint4> HairBoneCull : register(u6);            // per bone slot: x the surfaces its bound reaches, y a kCull state, z the colliders it touches

static const float kBoundCullScale = 1.25f;  // slack on a bone's bound radius before the cull test
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
		HairCollision::Transform world = HairCollision::LoadTransform(rawWorld.rotation, rawWorld.translation);
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
	for (uint surface = 0; surface < a_surface.y && surface < HairCollision::kMaskedSurfaceLimit; ++surface) {
		HairSurfaceMeta meta = HairSurfaces[a_surface.x + surface];
		if (meta.base.z == 0u)
			continue;
		float deepPush = meta.contact.y > 0.0f ? meta.contact.z : 0.0f;
		float reach = a_bone.bound.w + a_skin + meta.contact.x + abs(meta.contact.y) + deepPush;
		if (BoxOverlapsNode(HairSurfaceNodes[meta.base.y], boundCentre - float3(reach, reach, reach), boundCentre + float3(reach, reach, reach)))
			a_surfaceMask |= 1u << surface;
	}
	float boundReach = a_bone.bound.w * kBoundCullScale + a_skin + HairCollision::kBoundCullPad;
	bool anyCollider;
	a_colliderMask = ColliderBoundMask(a_tree, boundCentre, boundReach, anyCollider);
	return a_surfaceMask != 0u || anyCollider || a_surface.y > HairCollision::kMaskedSurfaceLimit;
}

/** @brief A record's vertex at the weight blend of its bones: hdtSMP64 collides the skinned vertex, not a copy riding one bone. */
float3 BlendedCentre(HairSphereRecord a_record)
{
	HairBoneWorld world = HairBoneWorlds[a_record.exempt.y];
	HairCollision::Transform pose = HairCollision::LoadTransform(world.rotation, world.translation);
	float3 centre = pose.translation + HairCollision::QuatRotate(pose.rotation, a_record.centreRadius.xyz);
	float total = a_record.weights.x;
	float3 blended = centre * total;
	for (uint other = 0u; other < 4u; ++other) {
		uint otherSlot = a_record.otherBone[other];
		if (otherSlot == kNoSphereBone)
			continue;
		HairBoneWorld otherWorld = HairBoneWorlds[otherSlot];
		HairCollision::Transform otherPose = HairCollision::LoadTransform(otherWorld.rotation, otherWorld.translation);
		float weight = a_record.other[other].w;
		blended += (otherPose.translation + HairCollision::QuatRotate(otherPose.rotation, a_record.other[other].xyz)) * weight;
		total += weight;
	}
	if (total > 0.0f && all(isfinite(blended)))
		centre = blended / total;
	return centre;
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
	if (tree == kNoTree)
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
		HairCollision::Transform pose = HairCollision::LoadTransform(world.rotation, world.translation);
		if (BoneBoundReaches(treeStatic, input.surface, sphereRange, pose, input.contact.z, surfaceMask, colliderMask)) {
			if (sphereRange.range.w != 0u)
				surfaceMask &= sphereRange.range.z;
			if (surfaceMask != 0u || colliderMask != 0u || input.surface.y > HairCollision::kMaskedSurfaceLimit)
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
	float3 centre = BlendedCentre(sphere);
	HairCollision::Contact contact = QueryContacts(HairTrees[tree], input.surface, cull.x, cull.z, centre,
		sphere.centreRadius.w + input.contact.z, sphere.exempt.x, sphere.exempt.z != 0u);
	HairSphereHit hit;
	hit.depth = contact.depth > 0.0f ? contact.depth * sphere.weights.y : 0.0f;
	hit.normal = contact.normal;
	hit.centre = centre;
	hit.collider = contact.collider;
	hit.velocityValid = contact.depth > 0.0f ? DriverPointVelocity(contact.driver, centre) : float4(0.0f, 0.0f, 0.0f, 0.0f);
	hit.key = contact.driver;
	hit.keyOther = contact.depth > 0.0f && contact.driverOther != contact.driver ? contact.driverOther : 0u;
	HairSphereHits[index] = hit;
}

/** @brief Poses one cloth vertex, a thread per vertex, for the triangles that share it. */
[numthreads(64, 1, 1)] void ClothVertices(uint3 a_dispatchThreadId : SV_DispatchThreadID) {
	uint index = a_dispatchThreadId.x;
	if (index >= HairClothVertexCount)
		return;
	HairClothPosed[index] = float4(BlendedCentre(HairClothVertices[index]), 1.0f);
}

static const float kTriangleContactEpsilon = 1.1920929e-7f;  // mirrors kTriangleContactEpsilon in ClothTriangles.h

/**
 * @brief hdtSMP64's sphere-against-triangle test, the port of TestVertexAgainstTriangle in ClothTriangles.cpp.
 * @param a_normal Receives the way the cloth is pushed: along the triangle's normal, away from the vertex.
 * @param a_point Receives the vertex projected onto the triangle's plane.
 * @param a_bodyPoint Receives the point of the vertex's sphere nearest the plane.
 */
bool ClothTriangleContact(float3 a_vertex, float a_radius, float3 a_a, float3 a_b, float3 a_c, float a_margin, float a_penetration,
	out float a_depth, out float3 a_normal, out float3 a_point, out float3 a_bodyPoint)
{
	a_depth = 0.0f;
	a_normal = float3(0.0f, 0.0f, 0.0f);
	a_point = float3(0.0f, 0.0f, 0.0f);
	a_bodyPoint = float3(0.0f, 0.0f, 0.0f);
	float3 raw = cross(a_b - a_a, a_c - a_a);
	float twiceArea = length(raw);
	if (!(twiceArea >= kTriangleContactEpsilon))
		return false;
	float3 normal = raw / twiceArea;
	float penetration = abs(a_penetration) < kTriangleContactEpsilon ? 0.0f : a_penetration;
	if (penetration < 0.0f) {
		normal = -normal;
		penetration = -penetration;
	}

	float distance = dot(a_vertex - a_a, normal);
	float3 projection = a_vertex - normal * distance;
	float reach = a_radius + a_margin;
	if (penetration >= kTriangleContactEpsilon) {
		if (!(distance < reach && distance >= -penetration))
			return false;
	} else {
		if (distance < 0.0f) {
			distance = -distance;
			normal = -normal;
		}
		if (!(distance < reach))
			return false;
	}

	float3 toA = projection - a_a;
	float3 toB = projection - a_b;
	float3 toC = projection - a_c;
	float areaA = length(cross(toB, toC));
	float areaB = length(cross(toC, toA));
	float areaC = length(cross(toA, toB));
	if (areaA + areaB > twiceArea || areaB + areaC > twiceArea || areaC + areaA > twiceArea)
		return false;

	float depth = reach - distance;
	if (!(depth > kTriangleContactEpsilon))
		return false;
	a_depth = depth;
	a_normal = -normal;
	a_point = projection;
	a_bodyPoint = a_vertex - normal * a_radius;
	return true;
}

HairSphereHit NoSphereHit()
{
	HairSphereHit hit;
	hit.depth = 0.0f;
	hit.normal = float3(0.0f, 0.0f, 0.0f);
	hit.centre = float3(0.0f, 0.0f, 0.0f);
	hit.collider = -1;
	hit.velocityValid = float4(0.0f, 0.0f, 0.0f, 0.0f);
	hit.key = 0u;
	hit.keyOther = 0u;
	return hit;
}

/** @brief Keeps a triangle's deepest hit in a_best and its deepest hit on any other driver in a_other. */
void OfferClothHit(inout HairSphereHit a_best, inout HairSphereHit a_other, HairSphereHit a_hit)
{
	if (a_hit.depth > a_best.depth) {
		if (a_hit.key != a_best.key && a_best.depth > 0.0f)
			a_other = a_best;
		a_best = a_hit;
	} else if (a_hit.key != a_best.key && a_hit.depth > a_other.depth) {
		a_other = a_hit;
	}
}

/**
 * @brief Tests one cloth triangle against the vertices of its claim's per-vertex surfaces, a thread per triangle.
 * @details A surface's vertices are reached through its posed faces, so one shared by several faces is tested once for each.
 *          Nothing is searched for past the triangle's window, so a vertex that has gone through the cloth is let go, as hdtSMP64 lets it.
 */
[numthreads(64, 1, 1)] void ClothTriangles(uint3 a_dispatchThreadId : SV_DispatchThreadID) {
	uint index = a_dispatchThreadId.x;
	if (index >= HairClothTriangleCount)
		return;
	HairClothTriangle cloth = HairClothTriangleList[index];
	HairSphereHit best = NoSphereHit();
	HairSphereHit other = NoSphereHit();
	uint tree = cloth.corners.w < HairBoneCount ? HairBoneTrees[cloth.corners.w] : kNoTree;
	if (tree != kNoTree) {
		HairTreeInput input = HairTreeInputs[tree];
		float3 a = HairClothPosed[cloth.corners.x].xyz;
		float3 b = HairClothPosed[cloth.corners.y].xyz;
		float3 c = HairClothPosed[cloth.corners.z].xyz;
		if (input.surface.z != 0u && all(isfinite(a)) && all(isfinite(b)) && all(isfinite(c))) {
			uint surfaceMask = asuint(cloth.contact.z);
			float margin = cloth.contact.x;
			float3 low = min(a, min(b, c));
			float3 high = max(a, max(b, c));
			uint colliderCount = HairTrees[tree].contact.y;
			for (uint surface = 0u; surface < input.surface.y; ++surface) {
				if (surface < HairCollision::kMaskedSurfaceLimit && (surfaceMask & (1u << surface)) == 0u)
					continue;
				HairSurfaceMeta meta = HairSurfaces[input.surface.x + surface];
				if (meta.base.w == 0u)
					continue;
				float radius = meta.contact.x;
				float extent = max(radius + margin, abs(cloth.contact.y));
				float3 boxLow = low - float3(extent, extent, extent);
				float3 boxHigh = high + float3(extent, extent, extent);
				uint node = 0u;
				// Each node is visited once at most, so the count only ends a walk over a hierarchy whose skips do not advance.
				uint visited = 0u;
				while (node < meta.base.z && visited < meta.base.z) {
					visited = visited + 1u;
					HairBvhNode current = HairSurfaceNodes[meta.base.y + node];
					if (!BoxOverlapsNode(current, boxLow, boxHigh)) {
						node = current.skip;
						continue;
					}
					if (current.leaf == 0u) {
						node = node + 1u;
						continue;
					}
					uint first = current.leaf >> 3;
					uint count = current.leaf & 7u;
					for (uint slot = 0u; slot < count; ++slot) {
						HairSurfaceTriangle face = HairSurfaceTriangles[meta.base.x + first + slot];
						if (face.normalArea.w <= 0.0f)
							continue;
						float3 corners[3] = { face.a.xyz, face.b.xyz, face.c.xyz };
						for (uint corner = 0u; corner < 3u; ++corner) {
							HairSphereHit hit = NoSphereHit();
							float3 bodyPoint;
							if (!ClothTriangleContact(corners[corner], radius, a, b, c, margin, cloth.contact.y, hit.depth, hit.normal, hit.centre, bodyPoint))
								continue;
							hit.collider = (int)(colliderCount + surface);
							hit.velocityValid = float4(bodyPoint, 0.0f);
							hit.key = asuint(face.b.w);
							OfferClothHit(best, other, hit);
						}
					}
					node = current.skip;
				}
			}
		}
	}
	// Until here velocityValid holds the body point the velocity is taken at.
	best.velocityValid = best.depth > 0.0f ? DriverPointVelocity(best.key, best.velocityValid.xyz) : float4(0.0f, 0.0f, 0.0f, 0.0f);
	other.velocityValid = other.depth > 0.0f ? DriverPointVelocity(other.key, other.velocityValid.xyz) : float4(0.0f, 0.0f, 0.0f, 0.0f);
	HairClothHits[index * kClothTriangleHits] = best;
	HairClothHits[index * kClothTriangleHits + 1u] = other;
}

/**
 * @brief Adds one hit to the lane of a_key, the port of the CPU ContactLanes::Add.
 * @param a_count How many times the hit is added: hdtSMP64 adds a triangle's contact once per corner weight of the bone.
 */
void FoldLaneHit(inout HairContactPartial a_lanes[kContactLanes], inout uint a_keys[kContactLanes], inout uint a_used, HairSphereHit a_hit, uint a_key, float a_count)
{
	uint lane = a_used;
	for (uint seen = 0u; seen < kContactLanes; ++seen) {
		if (seen < a_used && lane == a_used && a_keys[seen] == a_key)
			lane = seen;
	}
	if (lane == a_used) {
		if (a_used < kContactLanes) {
			a_keys[a_used] = a_key;
			a_used = a_used + 1u;
		} else {
			lane = 0u;
			float bestAligned = -2.0f;
			for (uint candidate = 0u; candidate < kContactLanes; ++candidate) {
				float sumLength = sqrt(dot(a_lanes[candidate].normalSum, a_lanes[candidate].normalSum));
				float aligned = sumLength > 0.0f ? dot(a_lanes[candidate].normalSum, a_hit.normal) / sumLength : -1.0f;
				if (aligned > bestAligned) {
					bestAligned = aligned;
					lane = candidate;
				}
			}
		}
	}
	HairContactPartial partial = a_lanes[lane];
	if (partial.weight > 0.0f && dot(partial.normalSum, a_hit.normal) < 0.0f)
		return;
	float squared = a_hit.depth * a_hit.depth * a_count;
	partial.weight += squared;
	partial.normalSum += a_hit.normal * (squared * a_hit.depth);
	partial.positionSum += a_hit.centre * squared;
	if (a_hit.velocityValid.w > 0.5f)
		partial.velocitySum += a_hit.velocityValid.xyz * squared;
	else
		partial.velocityMissing = true;
	if (a_hit.depth > partial.deepest) {
		partial.deepest = a_hit.depth;
		partial.collider = a_hit.collider;
	}
	a_lanes[lane] = partial;
}

	/** @brief Merges each bone's sphere and cloth triangle hits into one contact per contacted driver, kContactLanes at most, the port of the CPU ContactLanes. */
	[numthreads(64, 1, 1)] void ContactReduce(uint3 a_dispatchThreadId : SV_DispatchThreadID)
{
	uint bone = a_dispatchThreadId.x;
	if (bone >= HairBoneCount)
		return;
	uint4 cull = HairBoneCull[bone];
	if (cull.y == kCullSkip)
		return;
	HairContactPartial lanes[kContactLanes];
	uint keys[kContactLanes];
	uint used = 0u;
	for (uint clearLane = 0u; clearLane < kContactLanes; ++clearLane) {
		lanes[clearLane] = EmptyContactPartial();
		keys[clearLane] = 0u;
	}
	if (cull.y == kCullWork) {
		HairBoneSphere sphereRange = HairBoneSpheres[bone];
		for (uint index = 0u; index < sphereRange.range.y; ++index) {
			HairSphereHit hit = HairSphereHits[sphereRange.range.x + index];
			if (!(hit.depth > 0.0f))
				continue;
			for (uint turn = 0u; turn < 2u; ++turn) {
				uint key = turn == 0u ? hit.key : hit.keyOther;
				if (turn == 1u && key == 0u)
					break;
				FoldLaneHit(lanes, keys, used, hit, key, 1.0f);
			}
		}
	}
	if (HairClothTriangleCount != 0u) {
		uint4 cloth = HairBoneCloth[bone];
		for (uint owned = 0u; owned < cloth.y; ++owned) {
			HairClothBoneEntry entry = HairClothBoneList[cloth.x + owned];
			for (uint slot = 0u; slot < kClothTriangleHits; ++slot) {
				HairSphereHit hit = HairClothHits[entry.clothTriangle * kClothTriangleHits + slot];
				if (!(hit.depth > 0.0f))
					continue;
				hit.depth *= entry.flexible;
				FoldLaneHit(lanes, keys, used, hit, hit.key, (float)entry.corners);
			}
		}
	}
	for (uint lane = 0u; lane < kContactLanes; ++lane)
		HairBoneContactsOut[bone * kContactLanes + lane] = FinishBoneContact(lanes[lane]);
}
