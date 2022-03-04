#include "my_pch.h"
#include "myGeneric6DofMuscleConstraint.h"

// m_calculatedLinearDiff: updated in btGeneric6DofConstraint::calculateLinearInfo()
//		it is a vector3 from frameInA to frameInB expressed in P coordinate system

// m_linearLimits: class btTranslationalLimitMotor (defined inside Generic6DofConstraint)
//		


myGeneric6DofMuscleConstraint::myGeneric6DofMuscleConstraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB, bool useLinearReferenceFrameA)
	: btGeneric6DofSpringConstraint(rbA, rbB, frameInA, frameInB, useLinearReferenceFrameA) {}



myGeneric6DofMuscleConstraint::myGeneric6DofMuscleConstraint(btRigidBody& rbB, const btTransform& frameInB, bool useLinearReferenceFrameB)
	: btGeneric6DofSpringConstraint(rbB, frameInB, useLinearReferenceFrameB) {}


// currently it is the same as btGeneric6DofSpringConstraint::internalUpdateSprings
// I need to rewrite this so it doesn't act like a linear spring.
// Some of the parameters for force-length plot should be (imported?) in this class, too
void myGeneric6DofMuscleConstraint::m_internalUpdateMuscles(btConstraintInfo2* info)
{
// it is assumed that calculateTransforms() have been called before this call
	int i;
//btVector3 relVel = m_rbB.getLinearVelocity() - m_rbA.getLinearVelocity();
	for (i = 0; i < 3; i++) {
		if (m_springEnabled[i]) {
			// get current position of constraint
			btScalar currPos = m_calculatedLinearDiff[i];
			// calculate difference
			btScalar delta = currPos - m_equilibriumPoint[i];
			// spring force is (delta * m_stiffness) according to Hooke's Law
			btScalar force = delta * m_springStiffness[i];
			btScalar velFactor = info->fps * m_springDamping[i] / btScalar(info->m_numIterations);
			m_linearLimits.m_targetVelocity[i] = velFactor * force;
			m_linearLimits.m_maxMotorForce[i] = btFabs(force); // absolute value
		}
	}
	for (i = 0; i < 3; i++) {
		if (m_springEnabled[i + 3]) {
			// get current position of constraint
			btScalar currPos = m_calculatedAxisAngleDiff[i];
			// calculate difference
			btScalar delta = currPos - m_equilibriumPoint[i + 3];
			// spring force is (-delta * m_stiffness) according to Hooke's Law
			btScalar force = -delta * m_springStiffness[i + 3];
			btScalar velFactor = info->fps * m_springDamping[i + 3] / btScalar(info->m_numIterations);
			m_angularLimits[i].m_targetVelocity = velFactor * force;
			m_angularLimits[i].m_maxMotorForce = btFabs(force);
		}
	}
}

void myGeneric6DofMuscleConstraint::getInfo2(btConstraintInfo2* info) {
	// this will be called by constraint solver at the constraint setup stage
	// set current motor parameters
	m_internalUpdateMuscles(info);
	// do the rest of job for constraint setup
	btGeneric6DofConstraint::getInfo2(info);
}
