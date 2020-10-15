#include "MystacialPad.h"

MystacialPad::MystacialPad(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, 
						   btVector3 pos, btVector3 orient, btScalar height, btScalar radius) {
	
	btTransform trans = createTransform(pos, orient);
	btCollisionShape* shape = new btCylinderShape(btVector3(height, radius, radius));
	m_collisionShapes->push_back(shape);
	btRigidBody* body = createDynamicBody(1., trans, shape);
	m_dynamicsWorld->addRigidBody(body, COL_FOLLICLE, follicleCollideWith);
	body->setActivationState(DISABLE_DEACTIVATION);

}




