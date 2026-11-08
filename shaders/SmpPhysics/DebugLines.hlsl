// Draws the debug overlay's lines as flat, unlit ribbons into the scene's main render target.
// The record layouts mirror DebugLineVertex and DebugLineConstants in DebugLines.cpp; keep them in step.
// The per-frame and VR buffers are the engine's own, so only the members this shader reads are declared.

cbuffer PerFrame : register(b12)
{
#ifndef VR
	row_major float4x4 CameraProj[1] : packoffset(c4);
	row_major float4x4 CameraViewProj[1] : packoffset(c8);
	float4 CameraPosAdjust[1] : packoffset(c40);
#else
	row_major float4x4 CameraProj[2] : packoffset(c8);
	row_major float4x4 CameraViewProj[2] : packoffset(c16);
	float4 CameraPosAdjust[2] : packoffset(c80);
#endif
}

#ifdef VR
cbuffer VRValues : register(b13)
{
	float AlphaTestRefRS : packoffset(c0);
	float StereoEnabled : packoffset(c0.y);
	float2 EyeOffsetScale : packoffset(c0.z);
	float4 EyeClipEdge[2] : packoffset(c1);
}
#endif

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
	float4 Position : SV_POSITION0;
#ifdef VR
	float ClipDistance : SV_ClipDistance0;
	float CullDistance : SV_CullDistance0;
#endif
	float Side : TEXCOORD0;  // -1..1 across the ribbon
};

#ifdef VSHADER
static const uint kCornerEndpoint[6] = { 0, 0, 1, 1, 0, 1 };
static const float kCornerSide[6] = { -1.0, 1.0, -1.0, -1.0, 1.0, 1.0 };

VS_OUTPUT VSMain(uint a_vertexId : SV_VertexID, uint a_instanceId : SV_InstanceID)
{
#	ifdef VR
	uint eyeIndex = StereoEnabled * (a_instanceId & 1);
#	else
	uint eyeIndex = 0;
#	endif

	uint lineIndex = a_vertexId / 6;
	uint corner = a_vertexId % 6;
	float4 a = Vertices[lineIndex * 2];
	float4 b = Vertices[lineIndex * 2 + 1];
	float4 self = kCornerEndpoint[corner] != 0 ? b : a;

	float3 cameraPosition = CameraPosAdjust[eyeIndex].xyz;
	float4 clip = mul(CameraViewProj[eyeIndex], float4(self.xyz - cameraPosition, 1.0));
	float4 clipA = mul(CameraViewProj[eyeIndex], float4(a.xyz - cameraPosition, 1.0));
	float4 clipB = mul(CameraViewProj[eyeIndex], float4(b.xyz - cameraPosition, 1.0));

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

	float focal = CameraProj[eyeIndex][1][1];
	float widthPixels = max(MinimumPixelWidth, self.w * focal * eyeViewport.y * 0.5 / max(clip.w, 1e-4));
	float side = kCornerSide[corner];
	clip.xy += perpendicular * (widthPixels * side) / eyeViewport * clip.w;

	VS_OUTPUT output;
	output.Position = clip;
#	ifdef VR
	bool isStereoEnabled = StereoEnabled != 0;
	float clipEdge = isStereoEnabled ? dot(clip, EyeClipEdge[eyeIndex]) : 1.0;
	float stereoAdjustment = 2.0 - StereoEnabled;
	float eyeOffset = eyeIndex == 0 ? EyeOffsetScale.x : EyeOffsetScale.y;
	float xPositionOffset = eyeOffset * clip.w * (isStereoEnabled ? 1.0 : 0.0);
	output.Position.x = stereoAdjustment * clip.x * 0.5 + xPositionOffset;
	output.ClipDistance = clipEdge;
	output.CullDistance = clipEdge;
#	endif
	output.Side = side;
	return output;
}
#endif  // VSHADER

#ifdef PSHADER
float4 PSMain(VS_OUTPUT a_input) : SV_Target0
{
	// A line is thinner than a few pixels, so its edge fades by stochastic coverage.
	float coverage = saturate(1.0 - abs(a_input.Side));
	float noise = frac(52.9829189 * frac(dot(a_input.Position.xy, float2(0.06711056, 0.00583715))));
	if (coverage <= noise)
		discard;
	return float4(LineColor.xyz, 1.0);
}
#endif  // PSHADER
