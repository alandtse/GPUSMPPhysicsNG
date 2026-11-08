#ifndef __HAIR_SKIN_PALETTE_DEPENDENCY_HLSL__
#define __HAIR_SKIN_PALETTE_DEPENDENCY_HLSL__

// A bone's world pose as the solver stores it: rotation is w, x, y, z.
struct HairBoneWorld
{
	float4 rotation;
	float4 translation;
};

namespace HairSkinPalette
{
	static const uint kBytesPerBone = 48;
	static const float kQuaternionEpsilon = 1e-30;

	float3x3 QuatToMatrix(float4 a_wxyz)
	{
		float4 q = a_wxyz * rsqrt(max(dot(a_wxyz, a_wxyz), kQuaternionEpsilon));
		float w = q.x, x = q.y, y = q.z, z = q.w;
		return float3x3(
			1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),
			2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
			2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y));
	}

	/** @brief The three rows of a bone's skinning matrix: its world pose composed with its skin-to-bone rows. */
	void BuildRows(HairBoneWorld a_world, float a_worldScale, float4 a_skinToBone0, float4 a_skinToBone1, float4 a_skinToBone2,
		out float4 a_rows[3])
	{
		float3x3 rotation = QuatToMatrix(a_world.rotation);
		[unroll] for (uint row = 0; row < 3; ++row)
		{
			float3 axis = rotation[row];
			float4 composed = axis.x * a_skinToBone0 + axis.y * a_skinToBone1 + axis.z * a_skinToBone2;
			a_rows[row] = composed * a_worldScale;
			a_rows[row].w += a_world.translation[row];
		}
	}
}

#endif
