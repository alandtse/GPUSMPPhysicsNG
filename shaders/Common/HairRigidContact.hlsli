#ifndef __HAIR_RIGID_CONTACT_DEPENDENCY_HLSL__
#define __HAIR_RIGID_CONTACT_DEPENDENCY_HLSL__

#include "Common/HairRigidSolver.hlsli"

// Contact rows for the rigid-body solver, mirroring BuildContactRow and BuildFrictionRow in RigidReference.cpp.

namespace HairRigidContact
{
	static const float kUnboundedImpulse = 1.0e10f;
	static const float kSor = 1.0f;
	static const float kLinearSlop = 0.0f;
	static const float kErp2 = 0.15f;
	static const float kSplitImpulseTurnErp = 0.1f;
	static const float kSplitImpulsePenetrationThreshold = -0.04f;
	static const float kSimdEpsilon = 1.192092896e-07f;
	static const float kSqrt12 = 0.7071067811865475244008443621048490f;

	/** @brief A manifold point between two bodies, as SMP's narrow phase leaves it for the solver. */
	struct Contact
	{
		float3 pointOnB;
		float3 normalOnB;
		float distance;
		float friction;
	};

	/** @brief The normal row, the share of its error the split-impulse pass resolves, and the optional friction row. */
	struct ContactRows
	{
		HairRigidSolver::SolverRow normal;
		float rhsPenetration;
		HairRigidSolver::SolverRow friction;
		bool hasFriction;
	};

	/** @brief btPlaneSpace1: a unit vector perpendicular to a_normal. */
	float3 PlaneSpace(in float3 a_normal)
	{
		if (abs(a_normal.z) > kSqrt12) {
			const float a = a_normal.y * a_normal.y + a_normal.z * a_normal.z;
			const float k = 1.0f / sqrt(a);
			return float3(0.0f, -a_normal.z * k, a_normal.y * k);
		}
		const float a = a_normal.x * a_normal.x + a_normal.y * a_normal.y;
		const float k = 1.0f / sqrt(a);
		return float3(-a_normal.y * k, a_normal.x * k, 0.0f);
	}

	/** @brief One contact's rows, built from the bodies' states and solver bodies as the step starts. */
	ContactRows BuildRows(in Contact a_contact, in HairRigidSolver::BodyState a_stateA, in HairRigidSolver::BodyState a_stateB,
		in HairRigidSolver::SolverBody a_bodyA, in HairRigidSolver::SolverBody a_bodyB, in float a_globalCfm, in float a_framesPerSecond)
	{
		ContactRows result;
		const float3 normal = a_contact.normalOnB;
		const float cfm = a_globalCfm * a_framesPerSecond;
		const float3 pointOnA = a_contact.pointOnB + normal * a_contact.distance;
		const float3 relPos1 = pointOnA - a_stateA.transform.origin;
		const float3 relPos2 = a_contact.pointOnB - a_stateB.transform.origin;
		const float3 torqueAxis0 = HairRigidSolver::Cross(relPos1, normal);
		const float3 torqueAxis1 = HairRigidSolver::Cross(relPos2, normal);

		HairRigidSolver::SolverRow row = HairRigidSolver::ZeroRow();
		row.angularComponentA = HairRigidSolver::MatVec(a_stateA.inverseInertiaWorld, torqueAxis0);
		row.angularComponentB = HairRigidSolver::MatVec(a_stateB.inverseInertiaWorld, -torqueAxis1);
		const float denom0 = a_stateA.inverseMass + HairRigidSolver::Dot(normal, HairRigidSolver::Cross(row.angularComponentA, relPos1));
		const float denom1 = a_stateB.inverseMass + HairRigidSolver::Dot(normal, HairRigidSolver::Cross(-row.angularComponentB, relPos2));
		row.jacDiagABInv = kSor / (denom0 + denom1 + cfm);
		row.normal1 = normal;
		row.relpos1CrossNormal = torqueAxis0;
		row.normal2 = -normal;
		row.relpos2CrossNormal = -torqueAxis1;

		const float vel1Dotn = HairRigidSolver::Dot(row.normal1, a_bodyA.linearVelocity + a_bodyA.externalForceImpulse) +
		                       HairRigidSolver::Dot(row.relpos1CrossNormal, a_bodyA.angularVelocity + a_bodyA.externalTorqueImpulse);
		const float vel2Dotn = HairRigidSolver::Dot(row.normal2, a_bodyB.linearVelocity + a_bodyB.externalForceImpulse) +
		                       HairRigidSolver::Dot(row.relpos2CrossNormal, a_bodyB.angularVelocity + a_bodyB.externalTorqueImpulse);
		const float penetration = a_contact.distance + kLinearSlop;
		float velocityError = -(vel1Dotn + vel2Dotn);
		float positionalError = 0.0f;
		if (penetration > 0.0f)
			velocityError -= penetration * a_framesPerSecond;
		else
			positionalError = -penetration * kErp2 * a_framesPerSecond;
		const float penetrationImpulse = positionalError * row.jacDiagABInv;
		const float velocityImpulse = velocityError * row.jacDiagABInv;
		result.rhsPenetration = 0.0f;
		if (penetration > kSplitImpulsePenetrationThreshold) {
			row.rhs = penetrationImpulse + velocityImpulse;
		} else {
			row.rhs = velocityImpulse;
			result.rhsPenetration = penetrationImpulse;
		}
		row.cfm = cfm * row.jacDiagABInv;
		row.lowerLimit = 0.0f;
		row.upperLimit = kUnboundedImpulse;
		result.normal = row;

		result.hasFriction = a_contact.friction > 0.0f;
		HairRigidSolver::SolverRow frictionRow = HairRigidSolver::ZeroRow();
		if (result.hasFriction) {
			const float3 vel1 = a_bodyA.linearVelocity + a_bodyA.externalForceImpulse +
			                    HairRigidSolver::Cross(a_bodyA.angularVelocity + a_bodyA.externalTorqueImpulse, relPos1);
			const float3 vel2 = a_bodyB.linearVelocity + a_bodyB.externalForceImpulse +
			                    HairRigidSolver::Cross(a_bodyB.angularVelocity + a_bodyB.externalTorqueImpulse, relPos2);
			const float3 vel = vel1 - vel2;
			const float relVel = HairRigidSolver::Dot(normal, vel);
			float3 direction = vel - normal * relVel;
			const float lateral2 = HairRigidSolver::Dot(direction, direction);
			if (lateral2 > kSimdEpsilon)
				direction = direction * (1.0f / sqrt(lateral2));
			else
				direction = PlaneSpace(normal);

			frictionRow.normal1 = direction;
			frictionRow.relpos1CrossNormal = HairRigidSolver::Cross(relPos1, direction);
			frictionRow.angularComponentA = HairRigidSolver::MatVec(a_stateA.inverseInertiaWorld, frictionRow.relpos1CrossNormal);
			frictionRow.normal2 = -direction;
			frictionRow.relpos2CrossNormal = HairRigidSolver::Cross(relPos2, frictionRow.normal2);
			frictionRow.angularComponentB = HairRigidSolver::MatVec(a_stateB.inverseInertiaWorld, frictionRow.relpos2CrossNormal);
			const float frictionDenom0 = a_stateA.inverseMass + HairRigidSolver::Dot(direction, HairRigidSolver::Cross(frictionRow.angularComponentA, relPos1));
			const float frictionDenom1 = a_stateB.inverseMass + HairRigidSolver::Dot(direction, HairRigidSolver::Cross(-frictionRow.angularComponentB, relPos2));
			frictionRow.jacDiagABInv = kSor / (frictionDenom0 + frictionDenom1);
			const float frictionVel1 = HairRigidSolver::Dot(frictionRow.normal1, a_bodyA.linearVelocity + a_bodyA.externalForceImpulse) +
			                           HairRigidSolver::Dot(frictionRow.relpos1CrossNormal, a_bodyA.angularVelocity);
			const float frictionVel2 = HairRigidSolver::Dot(frictionRow.normal2, a_bodyB.linearVelocity + a_bodyB.externalForceImpulse) +
			                           HairRigidSolver::Dot(frictionRow.relpos2CrossNormal, a_bodyB.angularVelocity);
			frictionRow.rhs = -(frictionVel1 + frictionVel2) * frictionRow.jacDiagABInv;
			frictionRow.cfm = 0.0f;
		}
		result.friction = frictionRow;
		return result;
	}

	/** @brief The split-impulse update of one contact row: moves the push and turn velocities and returns the squared residual. */
	float ResolvePush(inout float a_appliedPush, in float a_rhsPenetration, in HairRigidSolver::SolverRow a_row,
		inout float3 a_pushA, inout float3 a_turnA, in float3 a_inverseMassA,
		inout float3 a_pushB, inout float3 a_turnB, in float3 a_inverseMassB)
	{
		float deltaImpulse = a_rhsPenetration - a_appliedPush * a_row.cfm;
		deltaImpulse -= (HairRigidSolver::Dot(a_row.normal1, a_pushA) + HairRigidSolver::Dot(a_row.relpos1CrossNormal, a_turnA)) * a_row.jacDiagABInv;
		deltaImpulse -= (HairRigidSolver::Dot(a_row.normal2, a_pushB) + HairRigidSolver::Dot(a_row.relpos2CrossNormal, a_turnB)) * a_row.jacDiagABInv;
		const float sum = a_appliedPush + deltaImpulse;
		if (sum < a_row.lowerLimit) {
			deltaImpulse = a_row.lowerLimit - a_appliedPush;
			a_appliedPush = a_row.lowerLimit;
		} else if (sum > a_row.upperLimit) {
			deltaImpulse = a_row.upperLimit - a_appliedPush;
			a_appliedPush = a_row.upperLimit;
		} else {
			a_appliedPush = sum;
		}
		a_pushA = a_pushA + (a_row.normal1 * a_inverseMassA) * deltaImpulse;
		a_turnA = a_turnA + a_row.angularComponentA * deltaImpulse;
		a_pushB = a_pushB + (a_row.normal2 * a_inverseMassB) * deltaImpulse;
		a_turnB = a_turnB + a_row.angularComponentB * deltaImpulse;
		const float residual = deltaImpulse * (1.0f / a_row.jacDiagABInv);
		return residual * residual;
	}
}

#endif  // __HAIR_RIGID_CONTACT_DEPENDENCY_HLSL__
