#include "Common/HairChainSolver.hlsli"

// GPU counterpart of PoseSurface, AppendSurface and TriangleBvh::Refit, with one refit dispatch per depth level.
// The record layout mirrors GpuRigPacking.h; keep the two in step.

struct HairSurfacePrepVertex
{
	float4 position;  // xyz = the bind position
	uint4 bones;      // one global skin-bone slot per weight, or 0xFFFFFFFF
	float4 weights;
};

struct HairSurfacePrepBone
{
	float4 rotation;     // skinToBone rotation in w, x, y, z lane order
	float4 translation;  // xyz = skinToBone translation
	uint4 driver;        // x = the global driver slot, or 0xFFFFFFFF when undriven
};

struct HairSurfacePrepTriangle
{
	uint4 corners;  // xyz = global vertex slots, or 0xFFFFFFFF; w = the source triangle index
};

struct HairSurfacePrepNode
{
	uint4 topology;  // x = surface-relative skip, y = leaf field, z/w = global child node slots
};

struct HairSurfacePrepLeaf
{
	uint4 range;  // x = global node slot, y = global first triangle slot, z = triangle count
};

struct HairSurfacePrepRefit
{
	uint4 link;  // x = global node slot, y/z = global child node slots, w = depth
};

struct HairSurfaceTriangle
{
	float4 a;  // w = the source face index
	float4 b;
	float4 c;
	float4 normalArea;  // xyz = unit face normal, w = twice the area
};

struct HairBvhNode
{
	float3 low;
	uint skip;  // surface-relative index of the first node after this one's subtree
	float3 high;
	uint leaf;  // 0 for an internal node, else (first slot << 3) | count
};

struct HairPrepDriver
{
	float4 rotation;     // w, x, y, z lane order
	float4 translation;  // xyz
};

StructuredBuffer<HairSurfacePrepVertex> HairPrepVertices : register(t0);
StructuredBuffer<HairSurfacePrepBone> HairPrepBones : register(t1);
StructuredBuffer<HairSurfacePrepTriangle> HairPrepTriangles : register(t2);
StructuredBuffer<HairSurfacePrepNode> HairPrepNodes : register(t3);
StructuredBuffer<HairSurfacePrepLeaf> HairPrepLeaves : register(t4);
StructuredBuffer<HairSurfacePrepRefit> HairPrepRefit : register(t5);
StructuredBuffer<HairPrepDriver> HairPrepDrivers : register(t6);

RWStructuredBuffer<float4> HairPosedVertices : register(u0);
RWStructuredBuffer<float4> HairSkinTranslations : register(u1);  // w = 1 when the bone is driven
RWStructuredBuffer<float4> HairSkinRotations : register(u2);
RWStructuredBuffer<HairSurfaceTriangle> HairSurfaceTriangleOut : register(u3);
RWStructuredBuffer<HairBvhNode> HairSurfaceNodeOut : register(u4);

cbuffer HairSurfacePrepParams : register(b0)
{
	uint BoneCount;
	uint VertexCount;
	uint TriangleCount;
	uint LeafCount;
	uint RefitFirst;
	uint RefitCount;
	uint2 pad;
};

static const float kPrepMinimumTriangleArea = 1e-8f;  // mirrors kMinimumTriangleArea in GpuRigPacking.cpp
static const float kPrepEmptyLow = 1e30f;             // mirrors kBvhEmptyLow in TriangleBvh.h
static const float kPrepEmptyHigh = -1e30f;           // mirrors kBvhEmptyHigh in TriangleBvh.h
static const uint kPrepInvalidSlot = 0xFFFFFFFFu;     // mirrors kInvalidDriverSlot in GpuRigPacking.h

HairChainSolver::Quat LoadPrepQuat(float4 a_value)
{
	HairChainSolver::Quat result;
	result.w = a_value.x;
	result.x = a_value.y;
	result.y = a_value.z;
	result.z = a_value.w;
	return result;
}

HairChainSolver::Transform LoadPrepTransform(float4 a_rotation, float4 a_translation)
{
	HairChainSolver::Transform result;
	result.rotation = LoadPrepQuat(a_rotation);
	result.translation = a_translation.xyz;
	return result;
}

/** @brief Reads one posed corner; false when its slot is out of range or its position is not finite. */
bool LoadPosedCorner(uint a_index, out float3 a_position)
{
	a_position = float3(0.0f, 0.0f, 0.0f);
	if (a_index >= VertexCount)
		return false;
	float4 posed = HairPosedVertices[a_index];
	a_position = posed.xyz;
	return isfinite(posed.x) && isfinite(posed.y) && isfinite(posed.z);
}

/** @brief One skin bone's world transform: the driver's pose composed with the bone's bind transform. */
[numthreads(64, 1, 1)] void SurfaceBones(uint3 a_dispatchThreadId : SV_DispatchThreadID) {
	uint bone = a_dispatchThreadId.x;
	if (bone >= BoneCount)
		return;
	HairSurfacePrepBone packed = HairPrepBones[bone];
	uint driver = packed.driver.x;
	if (driver == kPrepInvalidSlot) {
		HairSkinRotations[bone] = float4(0.0f, 0.0f, 0.0f, 0.0f);
		HairSkinTranslations[bone] = float4(0.0f, 0.0f, 0.0f, 0.0f);
		return;
	}
	HairPrepDriver rawWorld = HairPrepDrivers[driver];
	HairChainSolver::Transform skin = HairChainSolver::TransformMul(
		LoadPrepTransform(rawWorld.rotation, rawWorld.translation),
		LoadPrepTransform(packed.rotation, packed.translation));
	HairSkinRotations[bone] = float4(skin.rotation.w, skin.rotation.x, skin.rotation.y, skin.rotation.z);
	HairSkinTranslations[bone] = float4(skin.translation, 1.0f);
}

	/** @brief One vertex: its driven bones' transforms blended by weight, or its bind position when none is driven. */
	[numthreads(64, 1, 1)] void SurfacePose(uint3 a_dispatchThreadId : SV_DispatchThreadID)
{
	uint vertex = a_dispatchThreadId.x;
	if (vertex >= VertexCount)
		return;
	HairSurfacePrepVertex packed = HairPrepVertices[vertex];
	float3 bind = packed.position.xyz;
	uint bones[4] = { packed.bones.x, packed.bones.y, packed.bones.z, packed.bones.w };
	float weights[4] = { packed.weights.x, packed.weights.y, packed.weights.z, packed.weights.w };
	float3 sum = float3(0.0f, 0.0f, 0.0f);
	float total = 0.0f;
	[unroll] for (uint slot = 0u; slot < 4u; ++slot)
	{
		uint bone = bones[slot];
		if (bone == kPrepInvalidSlot || bone >= BoneCount || HairSkinTranslations[bone].w == 0.0f)
			continue;
		float weight = weights[slot];
		if (!(weight > 0.0f))
			continue;
		float3 posed = HairSkinTranslations[bone].xyz + HairChainSolver::QuatRotate(LoadPrepQuat(HairSkinRotations[bone]), bind);
		sum += posed * weight;
		total += weight;
	}
	float3 result = total > 0.0f ? sum / total : bind;
	HairPosedVertices[vertex] = float4(result, 1.0f);
}

/** @brief One posed triangle: its corners, unit normal and twice its area, or a degenerate record when unusable. */
[numthreads(64, 1, 1)] void SurfaceTriangles(uint3 a_dispatchThreadId : SV_DispatchThreadID) {
	uint slot = a_dispatchThreadId.x;
	if (slot >= TriangleCount)
		return;
	HairSurfacePrepTriangle prep = HairPrepTriangles[slot];
	HairSurfaceTriangle record;
	record.a = float4(0.0f, 0.0f, 0.0f, asfloat(prep.corners.w));
	record.b = float4(0.0f, 0.0f, 0.0f, 0.0f);
	record.c = float4(0.0f, 0.0f, 0.0f, 0.0f);
	record.normalArea = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float3 a;
	float3 b;
	float3 c;
	bool validA = LoadPosedCorner(prep.corners.x, a);
	bool validB = LoadPosedCorner(prep.corners.y, b);
	bool validC = LoadPosedCorner(prep.corners.z, c);
	if (validA && validB && validC) {
		float3 faceCross = cross(b - a, c - a);
		float twiceArea = length(faceCross);
		if (twiceArea > kPrepMinimumTriangleArea) {
			record.a = float4(a, asfloat(prep.corners.w));
			record.b = float4(b, 0.0f);
			record.c = float4(c, 0.0f);
			record.normalArea = float4(faceCross / twiceArea, twiceArea);
		}
	}
	HairSurfaceTriangleOut[slot] = record;
}

	/** @brief One leaf node's box: the union of the triangles it holds, which keeps the empty sentinel when none is usable. */
	[numthreads(64, 1, 1)] void SurfaceLeaves(uint3 a_dispatchThreadId : SV_DispatchThreadID)
{
	uint leaf = a_dispatchThreadId.x;
	if (leaf >= LeafCount)
		return;
	HairSurfacePrepLeaf entry = HairPrepLeaves[leaf];
	float3 low = float3(kPrepEmptyLow, kPrepEmptyLow, kPrepEmptyLow);
	float3 high = float3(kPrepEmptyHigh, kPrepEmptyHigh, kPrepEmptyHigh);
	for (uint index = 0u; index < entry.range.z; ++index) {
		HairSurfacePrepTriangle source = HairPrepTriangles[entry.range.y + index];
		float3 a;
		float3 b;
		float3 c;
		bool validA = LoadPosedCorner(source.corners.x, a);
		bool validB = LoadPosedCorner(source.corners.y, b);
		bool validC = LoadPosedCorner(source.corners.z, c);
		if (!validA || !validB || !validC)
			continue;
		low = min(low, min(a, min(b, c)));
		high = max(high, max(a, max(b, c)));
	}
	HairSurfacePrepNode topology = HairPrepNodes[entry.range.x];
	HairBvhNode node;
	node.low = low;
	node.high = high;
	node.skip = topology.topology.x;
	node.leaf = topology.topology.y;
	HairSurfaceNodeOut[entry.range.x] = node;
}

/** @brief One internal node's box at one depth level, from the two children an earlier dispatch already wrote. */
[numthreads(64, 1, 1)] void SurfaceRefit(uint3 a_dispatchThreadId : SV_DispatchThreadID) {
	uint entry = a_dispatchThreadId.x;
	if (entry >= RefitCount)
		return;
	HairSurfacePrepRefit refit = HairPrepRefit[RefitFirst + entry];
	HairBvhNode first = HairSurfaceNodeOut[refit.link.y];
	HairBvhNode second = HairSurfaceNodeOut[refit.link.z];
	HairSurfacePrepNode topology = HairPrepNodes[refit.link.x];
	HairBvhNode node;
	node.low = min(first.low, second.low);
	node.high = max(first.high, second.high);
	node.skip = topology.topology.x;
	node.leaf = topology.topology.y;
	HairSurfaceNodeOut[refit.link.x] = node;
}
