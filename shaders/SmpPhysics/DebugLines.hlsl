// Draws the debug overlay's lines as flat, unlit ribbons into the deferred G-buffer.
// The record layouts mirror DebugLineVertex and DebugLineConstants in DebugLines.cpp; keep them in step.

#include "Common/Color.hlsli"
#include "Common/FrameBuffer.hlsli"
#include "Common/GBuffer.hlsli"
#include "Common/Random.hlsli"
#include "Common/SharedData.hlsli"
#include "Common/VR.hlsli"

cbuffer DebugLineConstants : register(b1)
{
	uint LineCount;
	float MinimumPixelWidth;  // narrowest a line draws, so it stays visible at a distance
	float2 Viewport;
	float4 LineColor;
}

StructuredBuffer<float4> Vertices : register(t0);  // two per line: xyz world position, w world width

struct VS_OUTPUT
{
	float4 Position: SV_POSITION0;
#ifdef VR
	float ClipDistance: SV_ClipDistance0;
	float CullDistance: SV_CullDistance0;
#endif
	float Side: TEXCOORD0;  // -1..1 across the ribbon
};

#ifdef VSHADER
static const uint kCornerEndpoint[6] = { 0, 0, 1, 1, 0, 1 };
static const float kCornerSide[6] = { -1.0, 1.0, -1.0, -1.0, 1.0, 1.0 };

VS_OUTPUT VSMain(uint a_vertexId : SV_VertexID, uint a_instanceId : SV_InstanceID)
{
	uint eyeIndex = Stereo::GetEyeIndexVS(a_instanceId);

	uint line = a_vertexId / 6;
	uint corner = a_vertexId % 6;
	float4 a = Vertices[line * 2];
	float4 b = Vertices[line * 2 + 1];
	float4 self = kCornerEndpoint[corner] != 0 ? b : a;

	float3 cameraPosition = FrameBuffer::CameraPosAdjust[eyeIndex].xyz;
	float3 relative = self.xyz - cameraPosition;
	float4 clip = mul(FrameBuffer::CameraViewProj[eyeIndex], float4(relative, 1.0));
	float4 clipA = mul(FrameBuffer::CameraViewProj[eyeIndex], float4(a.xyz - cameraPosition, 1.0));
	float4 clipB = mul(FrameBuffer::CameraViewProj[eyeIndex], float4(b.xyz - cameraPosition, 1.0));

	// Each eye's half of a packed stereo target is half as wide as the viewport.
	float2 eyeViewport = Viewport;
#	ifdef VR
	eyeViewport.x *= 0.5;
#	endif

	float2 ndcA = clipA.xy / max(clipA.w, 1e-4);
	float2 ndcB = clipB.xy / max(clipB.w, 1e-4);
	float2 screenDirection = (ndcB - ndcA) * eyeViewport;
	float lengthSquared = dot(screenDirection, screenDirection);
	screenDirection = lengthSquared > 1e-8 ? screenDirection * rsqrt(lengthSquared) : float2(0.0, 1.0);
	float2 perpendicular = float2(-screenDirection.y, screenDirection.x);

	float focal = FrameBuffer::CameraProj[eyeIndex][1][1];
	float widthPixels = max(MinimumPixelWidth, self.w * focal * eyeViewport.y * 0.5 / max(clip.w, 1e-4));
	float side = kCornerSide[corner];
	clip.xy += perpendicular * (widthPixels * side) / eyeViewport * clip.w;

	VS_OUTPUT output;
	output.Position = clip;
#	ifdef VR
	Stereo::VR_OUTPUT vrOutput = Stereo::GetVRVSOutput(clip, eyeIndex);
	output.Position = vrOutput.VRPosition;
	output.ClipDistance = vrOutput.ClipDistance;
	output.CullDistance = vrOutput.CullDistance;
#	endif
	output.Side = side;
	return output;
}
#endif  // VSHADER

#ifdef PSHADER
struct PS_OUTPUT
{
	float4 Diffuse: SV_Target0;
	float4 MotionVectors: SV_Target1;
	float4 NormalGlossiness: SV_Target2;
	float4 Albedo: SV_Target3;
	float4 Specular: SV_Target4;
	float4 Reflectance: SV_Target5;
	float4 Masks: SV_Target6;
	float4 Masks2: SV_Target7;
};

PS_OUTPUT PSMain(VS_OUTPUT a_input)
{
	// A line is thinner than a few pixels, so its edge fades by stochastic coverage that TAA and the upscaler resolve.
	float coverage = saturate(1.0 - abs(a_input.Side));
	float screenNoise = Random::InterleavedGradientNoise(Stereo::EyeStableNoiseCoord(a_input.Position.xy, SharedData::BufferDim.xy), SharedData::FrameCount);
	if (coverage <= screenNoise)
		discard;

	// A flat colour that the composite leaves as it is.
	PS_OUTPUT output;
	output.Diffuse = float4(Color::IrradianceToGamma(LineColor.xyz), 1.0);
	output.MotionVectors = float4(0.0, 0.0, 0.0, 1.0);
	output.NormalGlossiness = float4(GBuffer::EncodeNormal(float3(0.0, 0.0, 1.0)), 0.0, 1.0);
	output.Albedo = float4(0.0, 0.0, 0.0, 1.0);
	output.Specular = float4(0.0, 0.0, 0.0, 1.0);
	output.Reflectance = float4(0.0, 0.0, 0.0, 1.0);
	output.Masks = float4(0.0, 0.0, 0.0, 1.0);
	output.Masks2 = float4(0.0, 0.0, 0.0, 1.0);
	return output;
}
#endif  // PSHADER
