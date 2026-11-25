#include "Common/HairRigidSolver.hlsli"
#include "Common/HairWind.hlsli"

// RigidFrameBegin turns kinematic poses into hdtSMP64's readTransform velocities and arms the dynamic bodies; RigidExport
// feeds the GPU solver's buffers. Record layouts mirror RigidGpuSolver.h and RigidGpuPacking.h; keep them in step.

static const uint kNoBody = 0xFFFFFFFFu;

/** @brief The GPU solver's merged per-bone contact; its layout is HairBoneContact in ContactCS.hlsl. */
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

struct HairRigidBody
{
	float4 inertia;
	float4 damping;
	uint4 flags;
	float4 gravity;
	float4 basis0;
	float4 basis1;
	float4 basis2;
	float4 origin;
	float4 linearVelocity;
	float4 angularVelocity;
	float4 interpolationLinearVelocity;
	float4 interpolationAngularVelocity;
	float4 accumulatedForce;
	float4 accumulatedTorque;
	float4 grabLocal;   // xyz = the held point in the body frame, w = the pin's stiffness
	float4 grabTarget;  // xyz = where the hand holds the point, w = 1 for a pin
};

struct HairRigidFrameInput
{
	float4 rotation;     // xyzw, a kinematic body's pose this frame
	float4 translation;  // xyz
	uint4 flags;         // x = 1 when the claim has stepped before, y = 1 when the body must return to rest, z = 1 when a hold frees its joint
	float4 wind;         // xyz = the world wind vector, w = the gust clock
	float4 windScale;    // x = the actor's wind exposure
	float4 grabLocal;    // xyz = the held point in the body frame, w = the pin's stiffness
	float4 grabTarget;   // xyz = where the hand holds the point, w = 1 for a pin
};

struct HairRigidBodyMeta
{
	uint4 slots;             // x = the GPU solver's bone slot, y = the body whose pose is this body's root, or kNoBody
	float4 restRotation;     // xyzw, the rest pose relative to the root
	float4 restTranslation;  // xyz
	float4 windAxis;         // xyz = the strand's direction in the body frame, w = the wind-facing area
	float4 windParams;       // x = the bone's own wind factor
	float4 nodeRotation;     // xyzw, the node's frame in the body's frame (a bone's inverse centerOfMassTransform)
	float4 nodeTranslation;  // xyz
};

struct HairBoneWorld
{
	float4 rotation;  // wxyz
	float4 translation;
};

struct HairBoneOutput
{
	float4 rotation;  // wxyz, relative to the root
	float4 translation;
	uint4 status;  // x = 1 when the body stopped being finite and was reset
};

StructuredBuffer<HairRigidFrameInput> HairRigidFrame : register(t0);
StructuredBuffer<HairRigidBodyMeta> HairRigidMeta : register(t1);
StructuredBuffer<HairBoneContact> HairRigidContacts : register(t2);

RWStructuredBuffer<HairRigidBody> HairRigidBodies : register(u0);
RWStructuredBuffer<HairBoneWorld> HairBoneWorlds : register(u1);
RWStructuredBuffer<HairBoneOutput> HairBoneOutputs : register(u2);

cbuffer HairRigidRuntimeParams : register(b0)
{
	uint HairRigidRuntimeBodyCount;
	float HairRigidFrameTime;
	float HairRigidLookAhead;  // seconds RigidWorldExport predicts ahead
	uint HairRigidRuntimePadding;
};

HairRigidSolver::Quat LoadRotation(in float4 a_xyzw)
{
	HairRigidSolver::Quat result;
	result.x = a_xyzw.x;
	result.y = a_xyzw.y;
	result.z = a_xyzw.z;
	result.w = a_xyzw.w;
	return result;
}

HairRigidSolver::Transform LoadPose(in float4 a_rotation, in float4 a_translation)
{
	HairRigidSolver::Transform result;
	result.basis = HairRigidSolver::BasisFromRotation(LoadRotation(a_rotation));
	result.origin = a_translation.xyz;
	return result;
}

HairRigidSolver::Transform LoadBodyPose(in HairRigidBody a_record)
{
	HairRigidSolver::Transform result;
	result.basis.row0 = a_record.basis0.xyz;
	result.basis.row1 = a_record.basis1.xyz;
	result.basis.row2 = a_record.basis2.xyz;
	result.origin = a_record.origin.xyz;
	return result;
}

void StoreBodyPose(inout HairRigidBody a_record, in HairRigidSolver::Transform a_pose)
{
	a_record.basis0 = float4(a_pose.basis.row0, 0.0f);
	a_record.basis1 = float4(a_pose.basis.row1, 0.0f);
	a_record.basis2 = float4(a_pose.basis.row2, 0.0f);
	a_record.origin = float4(a_pose.origin, 0.0f);
}

HairRigidSolver::Transform InversePose(in HairRigidSolver::Transform a_pose)
{
	HairRigidSolver::Transform result;
	result.basis = HairRigidSolver::Transposed(a_pose.basis);
	result.origin = -HairRigidSolver::MatVec(result.basis, a_pose.origin);
	return result;
}

float4 StoreChainRotation(in HairRigidSolver::Quat a_rotation)
{
	return float4(a_rotation.w, a_rotation.x, a_rotation.y, a_rotation.z);
}

/** @brief btTransformUtil::calculateVelocity: the linear and angular velocity that carry a_from onto a_to in a_timeStep. */
void CalculateVelocity(in HairRigidSolver::Transform a_from, in HairRigidSolver::Transform a_to, in float a_timeStep,
	out float3 a_linear, out float3 a_angular)
{
	a_linear = (a_to.origin - a_from.origin) / a_timeStep;
	HairRigidSolver::Quat difference = HairRigidSolver::QuatSafeNormalize(
		HairRigidSolver::RotationFromBasis(HairRigidSolver::MatMul(a_to.basis, HairRigidSolver::Transposed(a_from.basis))));
	if (difference.w < 0.0f) {
		difference.x = -difference.x;
		difference.y = -difference.y;
		difference.z = -difference.z;
		difference.w = -difference.w;
	}
	const float angle = 2.0f * acos(min(difference.w, 1.0f));
	float3 axis = float3(difference.x, difference.y, difference.z);
	const float length2 = HairRigidSolver::Dot(axis, axis);
	axis = length2 < HairRigidSolver::kEpsilon * HairRigidSolver::kEpsilon ? float3(1.0f, 0.0f, 0.0f) : axis / sqrt(length2);
	a_angular = axis * (angle / a_timeStep);
}

[numthreads(64, 1, 1)] void RigidFrameBegin(uint3 a_dispatchThreadId : SV_DispatchThreadID) {
	const uint index = a_dispatchThreadId.x;
	if (index >= HairRigidRuntimeBodyCount)
		return;
	HairRigidBody record = HairRigidBodies[index];
	const HairRigidFrameInput input = HairRigidFrame[index];
	const bool reset = input.flags.y != 0u;
	if (record.flags.x != 0u) {
		const HairRigidSolver::Transform target = LoadPose(input.rotation, input.translation);
		float3 linearSpeed = 0.0f;
		float3 angularSpeed = 0.0f;
		if (reset || input.flags.x == 0u) {
			StoreBodyPose(record, target);
		} else {
			CalculateVelocity(LoadBodyPose(record), target, HairRigidFrameTime, linearSpeed, angularSpeed);
		}
		record.linearVelocity = float4(linearSpeed, 0.0f);
		record.angularVelocity = float4(angularSpeed, 0.0f);
		record.interpolationLinearVelocity = float4(linearSpeed, 0.0f);
		record.interpolationAngularVelocity = float4(angularSpeed, 0.0f);
		record.accumulatedForce = 0.0f;
		record.accumulatedTorque = 0.0f;
		record.grabLocal = 0.0f;
		record.grabTarget = 0.0f;
	} else {
		const HairRigidBodyMeta meta = HairRigidMeta[index];
		if (reset && meta.slots.y != kNoBody) {
			const HairRigidFrameInput root = HairRigidFrame[meta.slots.y];
			StoreBodyPose(record, HairRigidSolver::Compose(LoadPose(root.rotation, root.translation), LoadPose(meta.restRotation, meta.restTranslation)));
			record.linearVelocity = 0.0f;
			record.angularVelocity = 0.0f;
		}
		float3 force = record.gravity.xyz;
		if (record.inertia.w > 0.0f) {
			const HairRigidSolver::Mat3 basis = { record.basis0.xyz, record.basis1.xyz, record.basis2.xyz };
			const float3 axis = HairRigidSolver::MatVec(basis, meta.windAxis.xyz);
			force += HairWind::WindForce(input.wind.xyz, input.wind.w, meta.windAxis.w, 1.0f / record.inertia.w,
				meta.windParams.x * input.windScale.x, axis, record.origin.xyz, record.linearVelocity.xyz);
		}
		record.flags.y = input.flags.z != 0u && !reset ? 1u : 0u;
		record.accumulatedForce = float4(force, 0.0f);
		record.accumulatedTorque = 0.0f;
		record.grabLocal = input.grabLocal;
		record.grabTarget = input.grabTarget;
	}
	HairRigidBodies[index] = record;
}

	[numthreads(64, 1, 1)] void RigidExport(uint3 a_dispatchThreadId : SV_DispatchThreadID)
{
	const uint index = a_dispatchThreadId.x;
	if (index >= HairRigidRuntimeBodyCount)
		return;
	HairRigidBody record = HairRigidBodies[index];
	if (record.flags.x != 0u)
		return;
	const HairRigidBodyMeta meta = HairRigidMeta[index];
	if (meta.slots.y == kNoBody)
		return;
	const HairRigidFrameInput rootInput = HairRigidFrame[meta.slots.y];
	const HairRigidSolver::Transform root = LoadPose(rootInput.rotation, rootInput.translation);
	HairRigidSolver::Transform world = LoadBodyPose(record);
	const bool finite = all(isfinite(world.origin)) && all(isfinite(world.basis.row0)) && all(isfinite(world.basis.row1)) && all(isfinite(world.basis.row2));
	if (!finite) {
		world = HairRigidSolver::Compose(root, LoadPose(meta.restRotation, meta.restTranslation));
		StoreBodyPose(record, world);
		record.linearVelocity = 0.0f;
		record.angularVelocity = 0.0f;
		HairRigidBodies[index] = record;
	}
	world = HairRigidSolver::Compose(world, LoadPose(meta.nodeRotation, meta.nodeTranslation));
	const HairRigidSolver::Transform relative = HairRigidSolver::Compose(InversePose(root), world);
	HairBoneWorld worldOut;
	worldOut.rotation = StoreChainRotation(HairRigidSolver::QuatSafeNormalize(HairRigidSolver::RotationFromBasis(world.basis)));
	worldOut.translation = float4(world.origin, 0.0f);
	HairBoneWorlds[meta.slots.x] = worldOut;
	HairBoneOutput output;
	output.rotation = StoreChainRotation(HairRigidSolver::QuatSafeNormalize(HairRigidSolver::RotationFromBasis(relative.basis)));
	output.translation = float4(relative.origin, 0.0f);
	output.status = uint4(finite ? 0u : 1u, 0u, 0u, 0u);
	HairBoneOutputs[meta.slots.x] = output;
}

/**
 * @brief Writes each body's pose at the end of the coming substep, as its velocity predicts it, where the contact passes read bone poses.
 * @details hdtSMP64 collides at these poses: Bullet predicts every body before it detects contacts, and the collision
 *          meshes are skinned from the predictions. Posed where the substep begins, cloth meets a body one step behind
 *          the one it is drawn over.
 */
[numthreads(64, 1, 1)] void RigidWorldExport(uint3 a_dispatchThreadId : SV_DispatchThreadID) {
	const uint index = a_dispatchThreadId.x;
	if (index >= HairRigidRuntimeBodyCount)
		return;
	const HairRigidBody record = HairRigidBodies[index];
	const HairRigidBodyMeta meta = HairRigidMeta[index];
	// A kinematic body's pose is exported too: a sphere blends the poses of every bone its vertex is skinned to.
	if (record.flags.x == 0u && meta.slots.y == kNoBody)
		return;
	HairRigidSolver::Transform world = LoadBodyPose(record);
	if (record.flags.x != 0u)
		world = HairRigidSolver::IntegrateTransform(world, record.interpolationLinearVelocity.xyz, record.interpolationAngularVelocity.xyz, HairRigidLookAhead);
	else
		world = HairRigidSolver::IntegrateTransform(world, record.linearVelocity.xyz * pow(1.0f - record.damping.x, HairRigidLookAhead),
			record.angularVelocity.xyz * pow(1.0f - record.damping.y, HairRigidLookAhead), HairRigidLookAhead);
	if (record.flags.x == 0u)
		world = HairRigidSolver::Compose(world, LoadPose(meta.nodeRotation, meta.nodeTranslation));
	HairBoneWorld worldOut;
	worldOut.rotation = StoreChainRotation(HairRigidSolver::QuatSafeNormalize(HairRigidSolver::RotationFromBasis(world.basis)));
	worldOut.translation = float4(world.origin, 0.0f);
	HairBoneWorlds[meta.slots.x] = worldOut;
}
