// Rebuilds the skinning matrices of the bones the solver simulates. The record layouts mirror
// SkinPaletteConstants and SkinPaletteBone in GpuSkinning.h; keep them in step.

#include "Common/HairSkinPalette.hlsli"

struct SkinBone
{
	uint slot;  // solver slot of the bone's world pose, or kUnsimulatedBone for a bone the solver does not simulate
	float worldScale;
	uint2 padding;
	float4 skinToBone0;  // skin-to-bone as three rows with its scale folded in, translation in w
	float4 skinToBone1;
	float4 skinToBone2;
};

static const uint kUnsimulatedBone = 0xFFFFFFFFu;

cbuffer SkinPaletteConstants : register(b0)
{
	uint BoneCount;
	uint HasLast;
	uint2 Padding;
};

StructuredBuffer<HairBoneWorld> Worlds : register(t0);
StructuredBuffer<SkinBone> Bones : register(t1);
ByteAddressBuffer Last : register(t2);

RWByteAddressBuffer Current : register(u0);
RWByteAddressBuffer Previous : register(u1);

[numthreads(64, 1, 1)] void main(uint3 a_id : SV_DispatchThreadID) {
	uint index = a_id.x;
	if (index >= BoneCount)
		return;

	SkinBone bone = Bones[index];
	if (bone.slot == kUnsimulatedBone)
		return;

	HairBoneWorld world = Worlds[bone.slot];
	float4 rows[3];
	HairSkinPalette::BuildRows(world, bone.worldScale, bone.skinToBone0, bone.skinToBone1, bone.skinToBone2, rows);

	uint offset = index * HairSkinPalette::kBytesPerBone;
	Current.Store4(offset, asuint(rows[0]));
	Current.Store4(offset + HairSkinPalette::kBytesPerRow, asuint(rows[1]));
	Current.Store4(offset + 2u * HairSkinPalette::kBytesPerRow, asuint(rows[2]));

	if (HasLast != 0u) {
		Previous.Store4(offset, Last.Load4(offset));
		Previous.Store4(offset + HairSkinPalette::kBytesPerRow, Last.Load4(offset + HairSkinPalette::kBytesPerRow));
		Previous.Store4(offset + 2u * HairSkinPalette::kBytesPerRow, Last.Load4(offset + 2u * HairSkinPalette::kBytesPerRow));
	} else {
		Previous.Store4(offset, asuint(rows[0]));
		Previous.Store4(offset + HairSkinPalette::kBytesPerRow, asuint(rows[1]));
		Previous.Store4(offset + 2u * HairSkinPalette::kBytesPerRow, asuint(rows[2]));
	}
}
