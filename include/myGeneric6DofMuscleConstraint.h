#ifndef MY_GENERIC_6DOF_MUSCLE_CONSTRAINT_H
#define MY_GENERIC_6DOF_MUSCLE_CONSTRAINT_H

#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h"

//ATTRIBUTE_ALIGNED16(class) myGeneric6DofMuscleConstraint : public btGeneric6DofSpringConstraint{ myGeneric6DofMuscleConstraint : public btGeneric6DofSpringConstraint {
class myGeneric6DofMuscleConstraint : public btGeneric6DofSpringConstraint{
protected:
	void m_internalUpdateMuscles(btConstraintInfo2* info);

public:
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	myGeneric6DofMuscleConstraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB, bool useLinearReferenceFrameA);
	myGeneric6DofMuscleConstraint(btRigidBody& rbB, const btTransform& frameInB, bool useLinearReferenceFrameB);

	virtual void getInfo2(btConstraintInfo2* info) override;

};




#endif