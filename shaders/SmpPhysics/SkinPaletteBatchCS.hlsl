// Rebuilds every registered skin instance's skinning matrices once per frame after the solver step.
// The record layouts mirror SkinPaletteBatchBone and SkinPaletteBatchConstants in GpuSkinning.h; keep them in step.

#include "Common/HairSkinPalette.hlsli"

struct BatchBone
{
	uint slot;  // solver slot of the bone's world pose, kStaticBone for a recorded node pose, or kUnusedBone past the skin's last bone
	float worldScale;
	uint staticIndex;  // index into StaticWorlds when slot is kStaticBone
	uint padding;
	float4 skinToBone0;  // skin-to-bone as three rows with its scale folded in, translation in w
	float4 skinToBone1;
	float4 skinToBone2;
};

cbuffer SkinPaletteBatchConstants : register(b0)
{
	uint ActiveCount;
	uint3 Padding;
};

StructuredBuffer<HairBoneWorld> Worlds : register(t0);
StructuredBuffer<BatchBone> Bones : register(t1);
StructuredBuffer<uint2> Active : register(t2);  // x = palette slot, y = 1 when the slot already holds a previous frame
StructuredBuffer<HairBoneWorld> StaticWorlds : register(t3);

RWByteAddressBuffer Current : register(u0);
RWByteAddressBuffer Previous : register(u1);

static const uint kBonesPerPalette = 80;
static const uint kStaticBone = 0xFFFFFFFEu;
static const uint kUnusedBone = 0xFFFFFFFFu;

[numthreads(64, 1, 1)] void main(uint3 a_id : SV_DispatchThreadID) {
	uint thread = a_id.x;
	if (thread >= ActiveCount * kBonesPerPalette)
		return;

	uint2 active = Active[thread / kBonesPerPalette];
	uint index = active.x * kBonesPerPalette + (thread % kBonesPerPalette);
	BatchBone bone = Bones[index];
	if (bone.slot == kUnusedBone)
		return;

	HairBoneWorld world;
	if (bone.slot == kStaticBone)
		world = StaticWorlds[bone.staticIndex];
	else
		world = Worlds[bone.slot];
	float4 rows[3];
	HairSkinPalette::BuildRows(world, bone.worldScale, bone.skinToBone0, bone.skinToBone1, bone.skinToBone2, rows);

	uint offset = index * HairSkinPalette::kBytesPerBone;
	if (active.y != 0u) {
		Previous.Store4(offset, Current.Load4(offset));
		Previous.Store4(offset + HairSkinPalette::kBytesPerRow, Current.Load4(offset + HairSkinPalette::kBytesPerRow));
		Previous.Store4(offset + 2u * HairSkinPalette::kBytesPerRow, Current.Load4(offset + 2u * HairSkinPalette::kBytesPerRow));
	} else {
		Previous.Store4(offset, asuint(rows[0]));
		Previous.Store4(offset + HairSkinPalette::kBytesPerRow, asuint(rows[1]));
		Previous.Store4(offset + 2u * HairSkinPalette::kBytesPerRow, asuint(rows[2]));
	}
	Current.Store4(offset, asuint(rows[0]));
	Current.Store4(offset + HairSkinPalette::kBytesPerRow, asuint(rows[1]));
	Current.Store4(offset + 2u * HairSkinPalette::kBytesPerRow, asuint(rows[2]));
}
