#include "Common/HairRigidContact.hlsli"
#include "Common/HairRigidSolver.hlsli"

// One rigid-body step in the reference's order, constraints solved in stored order inside each island.
// Record layouts mirror RigidGpuPacking.h; keep them in step.

/** @brief Threads one coloured-solve group runs, which is also the stride over a colour class's constraints. */
static const uint kColoredGroupThreads = 64u;

/** @brief Set in a contact constraint's first row mask: its row 0 is the normal row and row 1 the friction row. */
static const uint kContactMaskBit = 256u;

struct HairRigidBody
{
	float4 inertia;  // xyz = inverse inertia in the body frame, w = inverse mass
	float4 damping;  // x = linear damping, y = angular damping
	uint4 flags;     // x = 1 when the body is kinematic, y = 1 while a hold frees its joint limits
	float4 gravity;  // xyz = the body's frame gravity force
	float4 basis0;   // xyz = world basis row 0
	float4 basis1;   // xyz = world basis row 1
	float4 basis2;   // xyz = world basis row 2
	float4 origin;   // xyz = world origin
	float4 linearVelocity;
	float4 angularVelocity;
	float4 interpolationLinearVelocity;
	float4 interpolationAngularVelocity;
	float4 accumulatedForce;  // xyz = the force BeginFrame accumulated for the frame
	float4 accumulatedTorque;
	float4 grabLocal;   // xyz = the held point in the body frame, w = the pin's stiffness
	float4 grabTarget;  // xyz = where the hand holds the point, w = 1 for a pin
};

struct HairRigidAngularMotor
{
	float4 limits;  // x = lower, y = upper, z = bounce, w = stop ERP
	float4 spring;  // x = stop CFM, y = spring stiffness, z = spring damping, w = equilibrium
	float4 motor;   // x = motor ERP, y = motor CFM, z = target velocity, w = maximum motor force
	float4 servo;   // x = servo target, y = non-Hookean stiffness, z = non-Hookean damping
	uint4 flags;    // x = the axis's kAxis* bits
};

struct HairRigidConstraint
{
	uint4 header;           // x = body A, y = body B, z = 1 when enabled, w = rotation order
	uint4 constraintFlags;  // x = the axis override bits, the reference's kSmpLinearOverrideFlags, y = bone contact slot + 1, z = recorded contact + 1, w = a bone contact's friction as float bits
	float4 frameABasis0;    // xyz = basis row 0 of the joint frame in body A
	float4 frameABasis1;
	float4 frameABasis2;
	float4 frameAOrigin;
	float4 frameBBasis0;
	float4 frameBBasis1;
	float4 frameBBasis2;
	float4 frameBOrigin;
	float4 lower;
	float4 upper;
	float4 stopErp;
	float4 stopCfm;
	float4 springStiffness;
	float4 springDamping;
	float4 equilibrium;
	float4 bounce;
	float4 servoTarget;
	float4 targetVelocity;
	float4 maxMotorForce;
	float4 motorErp;
	float4 motorCfm;
	float4 nonHookeanStiffness;
	float4 nonHookeanDamping;
	uint4 linearFlags;  // xyz = each linear axis's kAxis* bits
	HairRigidAngularMotor angular[3];
};

struct HairRigidSolverBody
{
	float4 deltaLinearVelocity;
	float4 deltaAngularVelocity;
	float4 linearVelocity;
	float4 angularVelocity;
	float4 externalForceImpulse;
	float4 externalTorqueImpulse;
	float4 inverseMassVector;
	float4 pushVelocity;
	float4 turnVelocity;
};

struct HairRigidRow
{
	float4 normal1;
	float4 relpos1CrossNormal;
	float4 normal2;
	float4 relpos2CrossNormal;
	float4 angularComponentA;
	float4 angularComponentB;
	float4 impulse;  // x = rhs, y = cfm, z = jacDiagABInv, w = applied impulse
	float4 limits;   // x = lower limit, y = upper limit; a contact's normal row: z = split-impulse rhs, w = applied push impulse; its friction row: z = coefficient
	int4 bodies;     // x = solver body A, y = solver body B
};

/** @brief A recorded manifold point the host hands the solver; layout is ContactRecord in RigidGpuPacking.h. */
struct HairRigidExplicitContact
{
	int4 bodies;            // x = body A, y = body B
	float4 pointDistance;   // xyz = point on B, w = distance (negative while penetrating)
	float4 normalFriction;  // xyz = normal on B, w = friction coefficient
};

/** @brief The GPU solver's merged per-bone contact; its layout is HairBoneContact in ContactCS.hlsl. */
struct HairBoneContact
{
	float depth;
	float deepest;
	int collider;
	float3 normal;
	float3 position;
};

struct HairRigidIsland
{
	uint4 range;  // x = first constraint index in the island list, y = count
};

StructuredBuffer<HairRigidConstraint> HairRigidConstraints : register(t0);
StructuredBuffer<HairRigidIsland> HairRigidIslands : register(t1);
StructuredBuffer<uint> HairRigidIslandConstraints : register(t2);
StructuredBuffer<uint> HairRigidColoredConstraints : register(t3);
StructuredBuffer<uint> HairRigidColorOffsets : register(t4);
StructuredBuffer<HairBoneContact> HairRigidContacts : register(t5);
StructuredBuffer<HairRigidExplicitContact> HairRigidExplicitContacts : register(t6);

RWStructuredBuffer<HairRigidBody> HairRigidBodies : register(u0);
RWStructuredBuffer<HairRigidSolverBody> HairRigidSolverBodies : register(u1);
RWStructuredBuffer<HairRigidRow> HairRigidRows : register(u2);
RWStructuredBuffer<uint> HairRigidRowMasks : register(u3);  // per constraint and axis, the row slots RigidSetup filled

cbuffer HairRigidParams : register(b0)
{
	uint HairRigidBodyCount;
	uint HairRigidConstraintCount;
	uint HairRigidIslandCount;
	uint HairRigidIterations;
	float HairRigidTimeStep;
	float HairRigidFramesPerSecond;
	float HairRigidErp;
	float HairRigidGlobalCfm;
	float HairRigidDamping;
	float HairRigidResidualThreshold;
	uint HairRigidAxisSlots;
	uint HairRigidPadding1;
};

HairRigidSolver::Body LoadBody(in HairRigidBody a_record)
{
	HairRigidSolver::Body body;
	body.kinematic = a_record.flags.x != 0u;
	body.inverseMass = a_record.inertia.w;
	body.inverseInertiaLocal = a_record.inertia.xyz;
	body.linearDamping = a_record.damping.x;
	body.angularDamping = a_record.damping.y;
	body.gravity = a_record.gravity.xyz;
	body.worldTransform.basis.row0 = a_record.basis0.xyz;
	body.worldTransform.basis.row1 = a_record.basis1.xyz;
	body.worldTransform.basis.row2 = a_record.basis2.xyz;
	body.worldTransform.origin = a_record.origin.xyz;
	body.linearVelocity = a_record.linearVelocity.xyz;
	body.angularVelocity = a_record.angularVelocity.xyz;
	body.interpolationLinearVelocity = a_record.interpolationLinearVelocity.xyz;
	body.interpolationAngularVelocity = a_record.interpolationAngularVelocity.xyz;
	return body;
}

void StoreBodyState(inout HairRigidBody a_record, in HairRigidSolver::Body a_body)
{
	a_record.basis0 = float4(a_body.worldTransform.basis.row0, 0.0f);
	a_record.basis1 = float4(a_body.worldTransform.basis.row1, 0.0f);
	a_record.basis2 = float4(a_body.worldTransform.basis.row2, 0.0f);
	a_record.origin = float4(a_body.worldTransform.origin, 0.0f);
	a_record.linearVelocity = float4(a_body.linearVelocity, 0.0f);
	a_record.angularVelocity = float4(a_body.angularVelocity, 0.0f);
}

HairRigidSolver::BodyState BuildBodyState(in HairRigidSolver::Body a_body)
{
	HairRigidSolver::BodyState state;
	state.transform = a_body.worldTransform;
	state.linearVelocity = a_body.linearVelocity;
	state.angularVelocity = a_body.angularVelocity;
	state.inverseInertiaWorld = HairRigidSolver::InverseWorldInertia(a_body);
	state.inverseMass = a_body.inverseMass;
	return state;
}

HairRigidSolver::SolverBody LoadSolverBody(in HairRigidSolverBody a_record)
{
	HairRigidSolver::SolverBody result;
	result.deltaLinearVelocity = a_record.deltaLinearVelocity.xyz;
	result.deltaAngularVelocity = a_record.deltaAngularVelocity.xyz;
	result.linearVelocity = a_record.linearVelocity.xyz;
	result.angularVelocity = a_record.angularVelocity.xyz;
	result.externalForceImpulse = a_record.externalForceImpulse.xyz;
	result.externalTorqueImpulse = a_record.externalTorqueImpulse.xyz;
	result.inverseMassVector = a_record.inverseMassVector.xyz;
	return result;
}

HairRigidSolverBody StoreSolverBody(in HairRigidSolver::SolverBody a_value)
{
	HairRigidSolverBody record;
	record.deltaLinearVelocity = float4(a_value.deltaLinearVelocity, 0.0f);
	record.deltaAngularVelocity = float4(a_value.deltaAngularVelocity, 0.0f);
	record.linearVelocity = float4(a_value.linearVelocity, 0.0f);
	record.angularVelocity = float4(a_value.angularVelocity, 0.0f);
	record.externalForceImpulse = float4(a_value.externalForceImpulse, 0.0f);
	record.externalTorqueImpulse = float4(a_value.externalTorqueImpulse, 0.0f);
	record.inverseMassVector = float4(a_value.inverseMassVector, 0.0f);
	record.pushVelocity = 0.0f;
	record.turnVelocity = 0.0f;
	return record;
}

HairRigidSolver::MotorSettings LoadAngularMotor(in HairRigidAngularMotor a_record)
{
	HairRigidSolver::MotorSettings motor;
	motor.lower = a_record.limits.x;
	motor.upper = a_record.limits.y;
	motor.bounce = a_record.limits.z;
	motor.stopErp = a_record.limits.w;
	motor.stopCfm = a_record.spring.x;
	motor.springStiffness = a_record.spring.y;
	motor.springDamping = a_record.spring.z;
	motor.equilibrium = a_record.spring.w;
	motor.motorErp = a_record.motor.x;
	motor.motorCfm = a_record.motor.y;
	motor.targetVelocity = a_record.motor.z;
	motor.maxMotorForce = a_record.motor.w;
	motor.servoTarget = a_record.servo.x;
	motor.nonHookeanStiffness = a_record.servo.y;
	motor.nonHookeanDamping = a_record.servo.z;
	motor.flags = a_record.flags.x;
	return motor;
}

HairRigidSolver::ConstraintConfig LoadConstraintConfig(in HairRigidConstraint a_record)
{
	HairRigidSolver::ConstraintConfig config;
	config.frameABasis.row0 = a_record.frameABasis0.xyz;
	config.frameABasis.row1 = a_record.frameABasis1.xyz;
	config.frameABasis.row2 = a_record.frameABasis2.xyz;
	config.frameAOrigin = a_record.frameAOrigin.xyz;
	config.frameBBasis.row0 = a_record.frameBBasis0.xyz;
	config.frameBBasis.row1 = a_record.frameBBasis1.xyz;
	config.frameBBasis.row2 = a_record.frameBBasis2.xyz;
	config.frameBOrigin = a_record.frameBOrigin.xyz;
	config.rotationOrder = (int)a_record.header.w;
	config.flags = (int)a_record.constraintFlags.x;
	config.linearLimits.lower = a_record.lower.xyz;
	config.linearLimits.upper = a_record.upper.xyz;
	config.linearLimits.stopErp = a_record.stopErp.xyz;
	config.linearLimits.stopCfm = a_record.stopCfm.xyz;
	config.linearLimits.springStiffness = a_record.springStiffness.xyz;
	config.linearLimits.springDamping = a_record.springDamping.xyz;
	config.linearLimits.equilibrium = a_record.equilibrium.xyz;
	config.linearLimits.bounce = a_record.bounce.xyz;
	config.linearLimits.servoTarget = a_record.servoTarget.xyz;
	config.linearLimits.targetVelocity = a_record.targetVelocity.xyz;
	config.linearLimits.maxMotorForce = a_record.maxMotorForce.xyz;
	config.linearLimits.motorErp = a_record.motorErp.xyz;
	config.linearLimits.motorCfm = a_record.motorCfm.xyz;
	config.linearLimits.nonHookeanStiffness = a_record.nonHookeanStiffness.xyz;
	config.linearLimits.nonHookeanDamping = a_record.nonHookeanDamping.xyz;
	config.linearLimits.flags = a_record.linearFlags.xyz;
	config.angular0 = LoadAngularMotor(a_record.angular[0]);
	config.angular1 = LoadAngularMotor(a_record.angular[1]);
	config.angular2 = LoadAngularMotor(a_record.angular[2]);
	return config;
}

HairRigidSolver::SolverSettings LoadSolverSettings()
{
	HairRigidSolver::SolverSettings solver;
	solver.erp = HairRigidErp;
	solver.globalCfm = HairRigidGlobalCfm;
	solver.damping = HairRigidDamping;
	return solver;
}

HairRigidSolver::SolverRow LoadRow(in HairRigidRow a_record)
{
	HairRigidSolver::SolverRow row;
	row.normal1 = a_record.normal1.xyz;
	row.relpos1CrossNormal = a_record.relpos1CrossNormal.xyz;
	row.normal2 = a_record.normal2.xyz;
	row.relpos2CrossNormal = a_record.relpos2CrossNormal.xyz;
	row.angularComponentA = a_record.angularComponentA.xyz;
	row.angularComponentB = a_record.angularComponentB.xyz;
	row.rhs = a_record.impulse.x;
	row.cfm = a_record.impulse.y;
	row.jacDiagABInv = a_record.impulse.z;
	row.appliedImpulse = a_record.impulse.w;
	row.lowerLimit = a_record.limits.x;
	row.upperLimit = a_record.limits.y;
	row.solverBodyA = a_record.bodies.x;
	row.solverBodyB = a_record.bodies.y;
	return row;
}

HairRigidRow StoreRow(in HairRigidSolver::SolverRow a_value)
{
	HairRigidRow record;
	record.normal1 = float4(a_value.normal1, 0.0f);
	record.relpos1CrossNormal = float4(a_value.relpos1CrossNormal, 0.0f);
	record.normal2 = float4(a_value.normal2, 0.0f);
	record.relpos2CrossNormal = float4(a_value.relpos2CrossNormal, 0.0f);
	record.angularComponentA = float4(a_value.angularComponentA, 0.0f);
	record.angularComponentB = float4(a_value.angularComponentB, 0.0f);
	record.impulse = float4(a_value.rhs, a_value.cfm, a_value.jacDiagABInv, a_value.appliedImpulse);
	record.limits = float4(a_value.lowerLimit, a_value.upperLimit, 0.0f, 0.0f);
	record.bodies = int4(a_value.solverBodyA, a_value.solverBodyB, 0, 0);
	return record;
}

// Mirrored by kMaxGrabPull and kMinGrabDistance in smp/SolverCommon.h.
static const float kMaxGrabPull = 25.0f;
static const float kMinGrabDistance = 1.0e-3f;

/**
 * @brief Pulls the point a hand holds toward the hand as a spring on the body, the way a contact pushes it.
 * @details The impulse is capped at the one that would carry the body half the distance in one substep, so a
 *          stiff pin on a light strand cannot overshoot and ring.
 */
void ApplyGrabPin(in HairRigidBody a_record, in HairRigidSolver::Body a_body, inout HairRigidSolver::SolverBody a_solverBody)
{
	const float3 offset = HairRigidSolver::MatVec(a_body.worldTransform.basis, a_record.grabLocal.xyz);
	const float3 toTarget = a_record.grabTarget.xyz - (a_body.worldTransform.origin + offset);
	const float distance = length(toTarget);
	if (!(distance > kMinGrabDistance))
		return;
	const float3 direction = toTarget / distance;
	const float depth = min(distance, kMaxGrabPull);
	const float impulseLimit = distance / (2.0f * HairRigidTimeStep * a_body.inverseMass);
	const float3 impulse = direction * min(a_record.grabLocal.w * depth * HairRigidTimeStep, impulseLimit);
	a_solverBody.externalForceImpulse += impulse * a_body.inverseMass;
	a_solverBody.externalTorqueImpulse += HairRigidSolver::MatVec(HairRigidSolver::InverseWorldInertia(a_body),
		HairRigidSolver::Cross(offset, impulse));
}

/** @brief Damping, gravity as the frame force, and the predicted solver-body state: World::Step's prefix. */
[numthreads(64, 1, 1)] void RigidPredict(uint3 a_dispatchThreadId : SV_DispatchThreadID) {
	uint index = a_dispatchThreadId.x;
	if (index >= HairRigidBodyCount)
		return;

	HairRigidBody record = HairRigidBodies[index];
	HairRigidSolver::Body body = LoadBody(record);
	if (!body.kinematic) {
		body.linearVelocity = body.linearVelocity * pow(1.0f - record.damping.x, HairRigidTimeStep);
		body.angularVelocity = body.angularVelocity * pow(1.0f - record.damping.y, HairRigidTimeStep);
	}

	HairRigidSolver::SolverBody solverBody = HairRigidSolver::ZeroSolverBody();
	solverBody.inverseMassVector = float3(body.inverseMass, body.inverseMass, body.inverseMass);
	solverBody.linearVelocity = body.linearVelocity;
	solverBody.angularVelocity = body.angularVelocity;
	solverBody.externalForceImpulse = record.accumulatedForce.xyz * (body.inverseMass * HairRigidTimeStep);
	if (body.inverseMass != 0.0f)
		solverBody.externalTorqueImpulse = HairRigidSolver::GyroscopicImpulse(body, HairRigidTimeStep);
	if (!body.kinematic && body.inverseMass != 0.0f && record.grabTarget.w != 0.0f)
		ApplyGrabPin(record, body, solverBody);

	StoreBodyState(record, body);
	HairRigidBodies[index] = record;
	HairRigidSolverBodies[index] = StoreSolverBody(solverBody);
}

/** @brief Builds a contact constraint's normal row and optional friction row into its slot, and flags the slot as a contact. */
void SetupContact(uint a_constraint, int a_bodyA, int a_bodyB, in HairRigidContact::Contact a_contact)
{
	const HairRigidContact::ContactRows contactRows = HairRigidContact::BuildRows(a_contact,
		BuildBodyState(LoadBody(HairRigidBodies[a_bodyA])), BuildBodyState(LoadBody(HairRigidBodies[a_bodyB])),
		LoadSolverBody(HairRigidSolverBodies[a_bodyA]), LoadSolverBody(HairRigidSolverBodies[a_bodyB]), HairRigidGlobalCfm, HairRigidFramesPerSecond);
	const uint rowBase = a_constraint * (uint)HairRigidSolver::kRowsPerConstraint;
	HairRigidRow normalRecord = StoreRow(contactRows.normal);
	normalRecord.bodies = int4(a_bodyA, a_bodyB, 0, 0);
	normalRecord.limits.z = contactRows.rhsPenetration;
	HairRigidRows[rowBase] = normalRecord;
	uint mask = 1u | kContactMaskBit;
	if (contactRows.hasFriction) {
		HairRigidRow frictionRecord = StoreRow(contactRows.friction);
		frictionRecord.bodies = int4(a_bodyA, a_bodyB, 0, 0);
		frictionRecord.limits.z = a_contact.friction;
		HairRigidRows[rowBase + 1u] = frictionRecord;
		mask |= 2u;
	}
	HairRigidRowMasks[a_constraint * (uint)HairRigidSolver::kAxisSlots] = mask;
}

/** @brief Finalises one joint row and stores it, tagged with the two bodies it acts between. */
void StoreSetupRow(uint a_destination, in HairRigidSolver::SolverRow a_row, int a_bodyA, int a_bodyB,
	in HairRigidSolver::BodyState a_stateA, in HairRigidSolver::BodyState a_stateB,
	in HairRigidSolver::SolverBody a_solverBodyA, in HairRigidSolver::SolverBody a_solverBodyB)
{
	HairRigidSolver::SolverRow solved = HairRigidSolver::FinalizeRow(a_row, a_stateA, a_stateB, a_solverBodyA, a_solverBodyB, HairRigidDamping);
	solved.solverBodyA = a_bodyA;
	solved.solverBodyB = a_bodyB;
	HairRigidRows[a_destination] = StoreRow(solved);
}

/** @brief One constraint's solver rows, into its fixed slot: getInfo2 and convertJoint for one joint. */
[numthreads(64, 1, 1)] void RigidSetup(uint3 a_dispatchThreadId : SV_DispatchThreadID) {
	uint index = a_dispatchThreadId.x;
	if (index >= HairRigidConstraintCount)
		return;

	HairRigidConstraint record = HairRigidConstraints[index];
	const int bodyA = (int)record.header.x;
	const int bodyB = (int)record.header.y;
	const bool bothValid = bodyA >= 0 && (uint)bodyA < HairRigidBodyCount && bodyB >= 0 && (uint)bodyB < HairRigidBodyCount;
	const uint maskBase = index * (uint)HairRigidSolver::kAxisSlots;
	for (uint slot = 0u; slot < (uint)HairRigidSolver::kAxisSlots; ++slot)
		HairRigidRowMasks[maskBase + slot] = 0u;
	if (record.constraintFlags.z != 0u) {
		if (!bothValid)
			return;
		const HairRigidExplicitContact recorded = HairRigidExplicitContacts[record.constraintFlags.z - 1u];
		HairRigidContact::Contact contact;
		contact.pointOnB = recorded.pointDistance.xyz;
		contact.distance = recorded.pointDistance.w;
		contact.normalOnB = recorded.normalFriction.xyz;
		contact.friction = recorded.normalFriction.w;
		SetupContact(index, bodyA, bodyB, contact);
		return;
	}
	if (record.constraintFlags.y != 0u) {
		if (!bothValid)
			return;
		const HairBoneContact bone = HairRigidContacts[record.constraintFlags.y - 1u];
		if (bone.collider < 0 || !(bone.depth > 0.0f))
			return;
		HairRigidContact::Contact contact;
		contact.pointOnB = bone.position + bone.normal * bone.depth;
		contact.distance = -bone.depth;
		contact.normalOnB = bone.normal;
		contact.friction = asfloat(record.constraintFlags.w);
		SetupContact(index, bodyA, bodyB, contact);
		return;
	}
	if (record.header.z != 0u && bothValid) {
		HairRigidSolver::ConstraintConfig config = LoadConstraintConfig(record);
		const HairRigidBody recordA = HairRigidBodies[bodyA];
		const HairRigidBody recordB = HairRigidBodies[bodyB];
		const bool endAHeld = recordA.flags.x != 0u || recordA.flags.y != 0u;
		const bool endBHeld = recordB.flags.x != 0u || recordB.flags.y != 0u;
		// A hold frees every joint between the held bone and the root: both ends are held bodies or the kinematic root.
		const bool freed = endAHeld && endBHeld && (recordA.flags.y != 0u || recordB.flags.y != 0u);
		if (freed) {
			config.angular0.lower = 1.0f;
			config.angular0.upper = -1.0f;
			config.angular1.lower = 1.0f;
			config.angular1.upper = -1.0f;
			config.angular2.lower = 1.0f;
			config.angular2.upper = -1.0f;
		}
		HairRigidSolver::BodyState stateA = BuildBodyState(LoadBody(HairRigidBodies[bodyA]));
		HairRigidSolver::BodyState stateB = BuildBodyState(LoadBody(HairRigidBodies[bodyB]));
		HairRigidSolver::Joint joint = HairRigidSolver::ZeroJoint();
		joint = HairRigidSolver::CalculateTransforms(joint, config, stateA, stateB);
		for (int axis = 0; axis < 3; ++axis)
			joint = HairRigidSolver::TestAngularLimitMotor(joint, config, axis);
		if (HairRigidSolver::CountRows(joint, config) > 0) {
			const HairRigidSolver::SolverSettings solver = LoadSolverSettings();
			const HairRigidSolver::SolverBody solverBodyA = LoadSolverBody(HairRigidSolverBodies[bodyA]);
			const HairRigidSolver::SolverBody solverBodyB = LoadSolverBody(HairRigidSolverBodies[bodyB]);
			const uint rowBase = index * (uint)HairRigidSolver::kRowsPerConstraint;
			float rowCfm = solver.globalCfm;
			bool rowCfmWritten = false;
			[loop] for (int axisSlot = 0; axisSlot < (int)HairRigidAxisSlots; ++axisSlot) {
				const HairRigidSolver::AxisRows block = HairRigidSolver::BuildAxisRows(axisSlot, joint, config, stateA, stateB, solver, HairRigidFramesPerSecond, rowCfm);
				if (block.present == 0u)
					continue;
				if (!rowCfmWritten) {
					rowCfm = HairRigidSolver::BlockRowCfm(rowCfm, block);
					rowCfmWritten = true;
				}
				const uint slotBase = rowBase + (uint)axisSlot * (uint)HairRigidSolver::kRowsPerAxis;
				if ((block.present & HairRigidSolver::kRowLimitLower) != 0u)
					StoreSetupRow(slotBase + 0u, block.limitLower, bodyA, bodyB, stateA, stateB, solverBodyA, solverBodyB);
				if ((block.present & HairRigidSolver::kRowLimitUpper) != 0u)
					StoreSetupRow(slotBase + 1u, block.limitUpper, bodyA, bodyB, stateA, stateB, solverBodyA, solverBodyB);
				if ((block.present & HairRigidSolver::kRowMotor) != 0u)
					StoreSetupRow(slotBase + 2u, block.motor, bodyA, bodyB, stateA, stateB, solverBodyA, solverBodyB);
				if ((block.present & HairRigidSolver::kRowServo) != 0u)
					StoreSetupRow(slotBase + 3u, block.servo, bodyA, bodyB, stateA, stateB, solverBodyA, solverBodyB);
				if ((block.present & HairRigidSolver::kRowSpring) != 0u)
					StoreSetupRow(slotBase + 4u, block.spring, bodyA, bodyB, stateA, stateB, solverBodyA, solverBodyB);
				HairRigidRowMasks[maskBase + (uint)axisSlot] = block.present;
			}
		}
	}
}

/**
 * @brief Solves every row one constraint owns, in row order, and returns the largest squared residual it saw.
 * @details Both solve kernels advance a constraint through it, so a constraint's rows are stepped identically whichever
 *          one runs.
 */
float SolveConstraint(uint a_constraint)
{
	const uint maskBase = a_constraint * (uint)HairRigidSolver::kAxisSlots;
	const uint rowBase = a_constraint * (uint)HairRigidSolver::kRowsPerConstraint;
	float leastSquaresResidual = 0.0f;
	for (uint axisSlot = 0u; axisSlot < (uint)HairRigidSolver::kAxisSlots; ++axisSlot) {
		const uint mask = HairRigidRowMasks[maskBase + axisSlot];
		const uint slotBase = rowBase + axisSlot * (uint)HairRigidSolver::kRowsPerAxis;
		for (uint rowIndex = 0u; rowIndex < (uint)HairRigidSolver::kRowsPerAxis; ++rowIndex) {
			if ((mask & (1u << rowIndex)) == 0u)
				continue;
			HairRigidSolver::SolverRow row = LoadRow(HairRigidRows[slotBase + rowIndex]);
			if (rowIndex == 1u && (mask & kContactMaskBit) != 0u) {
				const float normalImpulse = HairRigidRows[slotBase].impulse.w;
				if (!(normalImpulse > 0.0f))
					continue;
				const float limit = HairRigidRows[slotBase + 1u].limits.z * normalImpulse;
				row.lowerLimit = -limit;
				row.upperLimit = limit;
			}
			HairRigidSolver::SolverBody bodyA = LoadSolverBody(HairRigidSolverBodies[row.solverBodyA]);
			HairRigidSolver::SolverBody bodyB = LoadSolverBody(HairRigidSolverBodies[row.solverBodyB]);
			const float residual = HairRigidSolver::ResolveRow(row, bodyA, bodyB);
			HairRigidRows[slotBase + rowIndex].impulse.w = row.appliedImpulse;
			HairRigidSolverBodies[row.solverBodyA].deltaLinearVelocity = float4(bodyA.deltaLinearVelocity, 0.0f);
			HairRigidSolverBodies[row.solverBodyA].deltaAngularVelocity = float4(bodyA.deltaAngularVelocity, 0.0f);
			HairRigidSolverBodies[row.solverBodyB].deltaLinearVelocity = float4(bodyB.deltaLinearVelocity, 0.0f);
			HairRigidSolverBodies[row.solverBodyB].deltaAngularVelocity = float4(bodyB.deltaAngularVelocity, 0.0f);
			const float residual2 = residual * residual;
			leastSquaresResidual = leastSquaresResidual < residual2 ? residual2 : leastSquaresResidual;
		}
	}
	return leastSquaresResidual;
}

/**
 * @brief The split-impulse update of one contact constraint's normal row, and its squared residual.
 * @details Zero for any constraint that is not a contact or has no penetration share to resolve.
 */
float SolvePush(uint a_constraint)
{
	const uint maskBase = a_constraint * (uint)HairRigidSolver::kAxisSlots;
	if ((HairRigidRowMasks[maskBase] & kContactMaskBit) == 0u)
		return 0.0f;
	const uint rowIndex = a_constraint * (uint)HairRigidSolver::kRowsPerConstraint;
	const float rhsPenetration = HairRigidRows[rowIndex].limits.z;
	if (rhsPenetration == 0.0f)
		return 0.0f;

	const HairRigidSolver::SolverRow row = LoadRow(HairRigidRows[rowIndex]);
	float appliedPush = HairRigidRows[rowIndex].limits.w;
	float3 pushA = HairRigidSolverBodies[row.solverBodyA].pushVelocity.xyz;
	float3 turnA = HairRigidSolverBodies[row.solverBodyA].turnVelocity.xyz;
	float3 pushB = HairRigidSolverBodies[row.solverBodyB].pushVelocity.xyz;
	float3 turnB = HairRigidSolverBodies[row.solverBodyB].turnVelocity.xyz;
	const float residual2 = HairRigidContact::ResolvePush(appliedPush, rhsPenetration, row,
		pushA, turnA, HairRigidSolverBodies[row.solverBodyA].inverseMassVector.xyz,
		pushB, turnB, HairRigidSolverBodies[row.solverBodyB].inverseMassVector.xyz);
	HairRigidRows[rowIndex].limits.w = appliedPush;
	HairRigidSolverBodies[row.solverBodyA].pushVelocity = float4(pushA, 0.0f);
	HairRigidSolverBodies[row.solverBodyA].turnVelocity = float4(turnA, 0.0f);
	HairRigidSolverBodies[row.solverBodyB].pushVelocity = float4(pushB, 0.0f);
	HairRigidSolverBodies[row.solverBodyB].turnVelocity = float4(turnB, 0.0f);
	return residual2;
}

/** @brief One island's whole iteration loop, its rows in constraint order: solveGroup's PGS loop. */
[numthreads(64, 1, 1)] void RigidSolveIsland(uint3 a_dispatchThreadId : SV_DispatchThreadID) {
	uint island = a_dispatchThreadId.x;
	if (island >= HairRigidIslandCount)
		return;

	const uint4 range = HairRigidIslands[island].range;
	const uint iterations = HairRigidIterations;
	for (uint iteration = 0u; iteration < iterations; ++iteration) {
		float leastSquaresResidual = 0.0f;
		for (uint entry = 0u; entry < range.y; ++entry) {
			const float residual2 = SolveConstraint(HairRigidIslandConstraints[range.x + entry]);
			leastSquaresResidual = leastSquaresResidual < residual2 ? residual2 : leastSquaresResidual;
		}
		if (leastSquaresResidual <= HairRigidResidualThreshold || iteration >= iterations - 1u)
			break;
	}
	if (range.w == 0u)
		return;
	for (uint iteration = 0u; iteration < iterations; ++iteration) {
		float leastSquaresResidual = 0.0f;
		for (uint entry = 0u; entry < range.y; ++entry) {
			const float residual2 = SolvePush(HairRigidIslandConstraints[range.x + entry]);
			leastSquaresResidual = leastSquaresResidual < residual2 ? residual2 : leastSquaresResidual;
		}
		if (leastSquaresResidual <= HairRigidResidualThreshold || iteration >= iterations - 1u)
			break;
	}
}

groupshared uint gColoredResidual;

/** @brief Largest island the groupshared copy of the solver bodies serves; a bigger one solves from the global buffers. */
static const uint kCachedConstraints = 512u;
static const uint kCachedBodies = 128u;
static const uint kBodyTableSize = 256u;
static const uint kEmptyKey = 0xFFFFFFFFu;

groupshared uint gTableKey[kBodyTableSize];
groupshared uint gTableSlot[kBodyTableSize];
groupshared uint gSlotCount;
groupshared uint gOverflow;
groupshared uint gEntrySlots[kCachedConstraints];
groupshared uint gSlotBody[kCachedBodies];
groupshared float4 gInverseMass[kCachedBodies];
groupshared float4 gDeltaLinear[kCachedBodies];
groupshared float4 gDeltaAngular[kCachedBodies];
groupshared float4 gPush[kCachedBodies];
groupshared float4 gTurn[kCachedBodies];

uint BodyHash(uint a_body)
{
	return (a_body * 0x9E3779B1u) >> 24;
}

void InsertBody(uint a_body)
{
	uint position = BodyHash(a_body);
	for (uint probe = 0u; probe < kBodyTableSize; ++probe) {
		uint previous;
		InterlockedCompareExchange(gTableKey[position], kEmptyKey, a_body, previous);
		if (previous == kEmptyKey || previous == a_body)
			return;
		position = (position + 1u) & (kBodyTableSize - 1u);
	}
	InterlockedOr(gOverflow, 1u);
}

uint FindBodySlot(uint a_body)
{
	uint position = BodyHash(a_body);
	while (gTableKey[position] != a_body)
		position = (position + 1u) & (kBodyTableSize - 1u);
	return gTableSlot[position];
}

/**
 * @brief SolveConstraint with the two bodies' solver state held in groupshared slots instead of the global buffer.
 * @details The row masks load together and each row's record is requested before the previous row is solved, so a
 *          constraint's rows no longer wait on one global-memory round trip each; the rows are solved in the same order.
 */
float SolveConstraintCached(uint a_constraint, uint a_slotA, uint a_slotB)
{
	const uint maskBase = a_constraint * (uint)HairRigidSolver::kAxisSlots;
	const uint rowBase = a_constraint * (uint)HairRigidSolver::kRowsPerConstraint;
	uint masks[HairRigidSolver::kAxisSlots];
	uint remaining = 0u;
	[unroll] for (uint slot = 0u; slot < (uint)HairRigidSolver::kAxisSlots; ++slot)
	{
		masks[slot] = HairRigidRowMasks[maskBase + slot];
		remaining |= (masks[slot] & ((1u << (uint)HairRigidSolver::kRowsPerAxis) - 1u)) << (slot * (uint)HairRigidSolver::kRowsPerAxis);
	}
	float leastSquaresResidual = 0.0f;
	if (remaining == 0u)
		return leastSquaresResidual;

	uint current = firstbitlow(remaining);
	remaining &= remaining - 1u;
	HairRigidRow currentRecord = HairRigidRows[rowBase + current];
	while (true) {
		uint next = 0xFFFFFFFFu;
		HairRigidRow nextRecord = (HairRigidRow)0;
		if (remaining != 0u) {
			next = firstbitlow(remaining);
			remaining &= remaining - 1u;
			nextRecord = HairRigidRows[rowBase + next];
		}

		const uint axisSlot = current / (uint)HairRigidSolver::kRowsPerAxis;
		const uint rowIndex = current - axisSlot * (uint)HairRigidSolver::kRowsPerAxis;
		const uint slotBase = rowBase + axisSlot * (uint)HairRigidSolver::kRowsPerAxis;
		HairRigidSolver::SolverRow row = LoadRow(currentRecord);
		bool skip = false;
		if (rowIndex == 1u && (masks[axisSlot] & kContactMaskBit) != 0u) {
			const float normalImpulse = HairRigidRows[slotBase].impulse.w;
			if (!(normalImpulse > 0.0f)) {
				skip = true;
			} else {
				const float limit = HairRigidRows[slotBase + 1u].limits.z * normalImpulse;
				row.lowerLimit = -limit;
				row.upperLimit = limit;
			}
		}
		if (!skip) {
			HairRigidSolver::SolverBody bodyA = (HairRigidSolver::SolverBody)0;
			HairRigidSolver::SolverBody bodyB = (HairRigidSolver::SolverBody)0;
			bodyA.deltaLinearVelocity = gDeltaLinear[a_slotA].xyz;
			bodyA.deltaAngularVelocity = gDeltaAngular[a_slotA].xyz;
			bodyA.inverseMassVector = gInverseMass[a_slotA].xyz;
			bodyB.deltaLinearVelocity = gDeltaLinear[a_slotB].xyz;
			bodyB.deltaAngularVelocity = gDeltaAngular[a_slotB].xyz;
			bodyB.inverseMassVector = gInverseMass[a_slotB].xyz;
			const float residual = HairRigidSolver::ResolveRow(row, bodyA, bodyB);
			HairRigidRows[slotBase + rowIndex].impulse.w = row.appliedImpulse;
			gDeltaLinear[a_slotA] = float4(bodyA.deltaLinearVelocity, 0.0f);
			gDeltaAngular[a_slotA] = float4(bodyA.deltaAngularVelocity, 0.0f);
			gDeltaLinear[a_slotB] = float4(bodyB.deltaLinearVelocity, 0.0f);
			gDeltaAngular[a_slotB] = float4(bodyB.deltaAngularVelocity, 0.0f);
			const float residual2 = residual * residual;
			leastSquaresResidual = leastSquaresResidual < residual2 ? residual2 : leastSquaresResidual;
		}

		if (next == 0xFFFFFFFFu)
			break;
		current = next;
		currentRecord = nextRecord;
	}
	return leastSquaresResidual;
}

/** @brief SolvePush with the two bodies' push and turn velocities held in groupshared slots. */
float SolvePushCached(uint a_constraint, uint a_slotA, uint a_slotB)
{
	const uint maskBase = a_constraint * (uint)HairRigidSolver::kAxisSlots;
	if ((HairRigidRowMasks[maskBase] & kContactMaskBit) == 0u)
		return 0.0f;
	const uint rowIndex = a_constraint * (uint)HairRigidSolver::kRowsPerConstraint;
	const float rhsPenetration = HairRigidRows[rowIndex].limits.z;
	if (rhsPenetration == 0.0f)
		return 0.0f;

	const HairRigidSolver::SolverRow row = LoadRow(HairRigidRows[rowIndex]);
	float appliedPush = HairRigidRows[rowIndex].limits.w;
	float3 pushA = gPush[a_slotA].xyz;
	float3 turnA = gTurn[a_slotA].xyz;
	float3 pushB = gPush[a_slotB].xyz;
	float3 turnB = gTurn[a_slotB].xyz;
	const float residual2 = HairRigidContact::ResolvePush(appliedPush, rhsPenetration, row,
		pushA, turnA, gInverseMass[a_slotA].xyz,
		pushB, turnB, gInverseMass[a_slotB].xyz);
	HairRigidRows[rowIndex].limits.w = appliedPush;
	gPush[a_slotA] = float4(pushA, 0.0f);
	gTurn[a_slotA] = float4(turnA, 0.0f);
	gPush[a_slotB] = float4(pushB, 0.0f);
	gTurn[a_slotB] = float4(turnB, 0.0f);
	return residual2;
}

/** @brief The group barrier an iteration needs: groupshared state alone, or the global buffers too when the island is not cached. */
void IterationBarrier(bool a_cached)
{
	if (a_cached)
		GroupMemoryBarrierWithGroupSync();
	else
		AllMemoryBarrierWithGroupSync();
}

/**
 * @brief Builds the island's groupshared body table and loads the solver bodies its constraints touch.
 * @details A constraint's rows are always read and written by the thread that owns its position in the coloured
 *          list, so the row buffer needs no cross-thread visibility; only the bodies are shared between threads.
 * @return False when the island has too many bodies, leaving the group to solve from the global buffers.
 */
bool CacheIslandBodies(uint4 a_range, uint a_thread)
{
	for (uint clearPosition = a_thread; clearPosition < kBodyTableSize; clearPosition += kColoredGroupThreads)
		gTableKey[clearPosition] = kEmptyKey;
	if (a_thread == 0u) {
		gSlotCount = 0u;
		gOverflow = 0u;
	}
	GroupMemoryBarrierWithGroupSync();

	for (uint insertEntry = a_thread; insertEntry < a_range.y; insertEntry += kColoredGroupThreads) {
		const uint4 header = HairRigidConstraints[HairRigidColoredConstraints[a_range.x + insertEntry]].header;
		InsertBody(header.x);
		InsertBody(header.y);
	}
	GroupMemoryBarrierWithGroupSync();

	for (uint assignPosition = a_thread; assignPosition < kBodyTableSize; assignPosition += kColoredGroupThreads) {
		if (gTableKey[assignPosition] == kEmptyKey)
			continue;
		uint assigned;
		InterlockedAdd(gSlotCount, 1u, assigned);
		if (assigned >= kCachedBodies) {
			InterlockedOr(gOverflow, 1u);
		} else {
			gTableSlot[assignPosition] = assigned;
			gSlotBody[assigned] = gTableKey[assignPosition];
		}
	}
	GroupMemoryBarrierWithGroupSync();
	if (gOverflow != 0u)
		return false;

	for (uint slotEntry = a_thread; slotEntry < a_range.y; slotEntry += kColoredGroupThreads) {
		const uint4 header = HairRigidConstraints[HairRigidColoredConstraints[a_range.x + slotEntry]].header;
		gEntrySlots[slotEntry] = (FindBodySlot(header.x) << 16u) | FindBodySlot(header.y);
	}
	for (uint loadSlot = a_thread; loadSlot < gSlotCount; loadSlot += kColoredGroupThreads) {
		const HairRigidSolverBody body = HairRigidSolverBodies[gSlotBody[loadSlot]];
		gInverseMass[loadSlot] = body.inverseMassVector;
		gDeltaLinear[loadSlot] = body.deltaLinearVelocity;
		gDeltaAngular[loadSlot] = body.deltaAngularVelocity;
		gPush[loadSlot] = body.pushVelocity;
		gTurn[loadSlot] = body.turnVelocity;
	}
	GroupMemoryBarrierWithGroupSync();
	return true;
}

/**
 * @brief One island's iteration loop, one thread group per island, its rows swept colour class by colour class.
 * @details Constraints of one colour share no dynamic body, so their rows run concurrently and the barriers keep the
 *          classes in step; the sweep takes colour order rather than world order, this kernel's one deviation from
 *          World::Step's single sequential pass. The group leaves together once the largest squared residual of a sweep
 *          is at or under the threshold, as the sequential kernel does. An island small enough keeps its bodies' solver
 *          state in groupshared memory for the loops and writes it back once; the arithmetic is unchanged.
 */
[numthreads(kColoredGroupThreads, 1, 1)] void RigidSolveIslandColored(uint3 a_groupId : SV_GroupID,
	uint a_threadIndex : SV_GroupThreadID) {
	const uint island = a_groupId.x;
	if (island >= HairRigidIslandCount)
		return;

	const uint4 range = HairRigidIslands[island].range;
	const uint colorCount = range.z;
	const uint offsetBase = island * HairRigidSolver::kMaxColors;
	const bool cached = range.y <= kCachedConstraints && CacheIslandBodies(range, a_threadIndex);
	bool converged = false;
	for (uint iteration = 0u; iteration < HairRigidIterations; ++iteration) {
		if (a_threadIndex == 0u)
			gColoredResidual = 0u;
		IterationBarrier(cached);
		float residual2 = 0.0f;
		for (uint color = 0u; color < colorCount; ++color) {
			const uint first = HairRigidColorOffsets[offsetBase + color];
			const uint last = color + 1u < colorCount ? HairRigidColorOffsets[offsetBase + color + 1u] : range.y;
			if (!converged) {
				for (uint entry = first + a_threadIndex; entry < last; entry += kColoredGroupThreads) {
					const uint constraint = HairRigidColoredConstraints[range.x + entry];
					if (cached)
						residual2 = max(residual2, SolveConstraintCached(constraint, gEntrySlots[entry] >> 16u, gEntrySlots[entry] & 0xFFFFu));
					else
						residual2 = max(residual2, SolveConstraint(constraint));
				}
			}
			IterationBarrier(cached);
		}
		InterlockedMax(gColoredResidual, asuint(residual2));
		IterationBarrier(cached);
		converged = converged || asfloat(gColoredResidual) <= HairRigidResidualThreshold;
	}
	if (range.w != 0u) {
		converged = false;
		for (uint pushIteration = 0u; pushIteration < HairRigidIterations; ++pushIteration) {
			if (a_threadIndex == 0u)
				gColoredResidual = 0u;
			IterationBarrier(cached);
			float residual2 = 0.0f;
			for (uint color = 0u; color < colorCount; ++color) {
				const uint first = HairRigidColorOffsets[offsetBase + color];
				const uint last = color + 1u < colorCount ? HairRigidColorOffsets[offsetBase + color + 1u] : range.y;
				if (!converged) {
					for (uint entry = first + a_threadIndex; entry < last; entry += kColoredGroupThreads) {
						const uint constraint = HairRigidColoredConstraints[range.x + entry];
						if (cached)
							residual2 = max(residual2, SolvePushCached(constraint, gEntrySlots[entry] >> 16u, gEntrySlots[entry] & 0xFFFFu));
						else
							residual2 = max(residual2, SolvePush(constraint));
					}
				}
				IterationBarrier(cached);
			}
			InterlockedMax(gColoredResidual, asuint(residual2));
			IterationBarrier(cached);
			converged = converged || asfloat(gColoredResidual) <= HairRigidResidualThreshold;
		}
	}
	if (cached) {
		for (uint slot = a_threadIndex; slot < gSlotCount; slot += kColoredGroupThreads) {
			const uint body = gSlotBody[slot];
			HairRigidSolverBodies[body].deltaLinearVelocity = gDeltaLinear[slot];
			HairRigidSolverBodies[body].deltaAngularVelocity = gDeltaAngular[slot];
			HairRigidSolverBodies[body].pushVelocity = gPush[slot];
			HairRigidSolverBodies[body].turnVelocity = gTurn[slot];
		}
	}
}

	/** @brief Velocity write-back and integrateTransforms, kinematic bodies first and then the dynamic ones. */
	[numthreads(64, 1, 1)] void RigidIntegrate(uint3 a_dispatchThreadId : SV_DispatchThreadID)
{
	uint index = a_dispatchThreadId.x;
	if (index >= HairRigidBodyCount)
		return;

	HairRigidBody record = HairRigidBodies[index];
	HairRigidSolver::Body body = LoadBody(record);
	HairRigidSolver::SolverBody solverBody = LoadSolverBody(HairRigidSolverBodies[index]);
	solverBody.linearVelocity = solverBody.linearVelocity + solverBody.deltaLinearVelocity;
	solverBody.angularVelocity = solverBody.angularVelocity + solverBody.deltaAngularVelocity;
	body.linearVelocity = solverBody.linearVelocity + solverBody.externalForceImpulse;
	body.angularVelocity = solverBody.angularVelocity + solverBody.externalTorqueImpulse;

	const float3 pushVelocity = HairRigidSolverBodies[index].pushVelocity.xyz;
	const float3 turnVelocity = HairRigidSolverBodies[index].turnVelocity.xyz;
	if (HairRigidSolver::Dot(pushVelocity, pushVelocity) + HairRigidSolver::Dot(turnVelocity, turnVelocity) != 0.0f)
		body.worldTransform = HairRigidSolver::IntegrateTransform(body.worldTransform, pushVelocity,
			turnVelocity * HairRigidContact::kSplitImpulseTurnErp, HairRigidTimeStep);

	if (body.kinematic)
		body.worldTransform = HairRigidSolver::IntegrateTransform(body.worldTransform,
			body.interpolationLinearVelocity, body.interpolationAngularVelocity, HairRigidTimeStep);
	body.linearVelocity = HairRigidSolver::ClampVelocity(body.linearVelocity);
	body.angularVelocity = HairRigidSolver::ClampVelocity(body.angularVelocity);
	if (!body.kinematic)
		body.worldTransform = HairRigidSolver::IntegrateTransform(body.worldTransform,
			body.linearVelocity, body.angularVelocity, HairRigidTimeStep);

	StoreBodyState(record, body);
	HairRigidBodies[index] = record;
}
