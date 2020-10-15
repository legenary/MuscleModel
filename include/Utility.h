#ifndef UTILITY_H
#define UTILITY_H

#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"

#include "CommonInterfaces/CommonGUIHelperInterface.h"

#include "LinearMath/btVector3.h"


btRigidBody* createDynamicBody(float mass, const btTransform& startTransform, btCollisionShape* shape, GUIHelperInterface* m_guiHelper);

btTransform createTransform(btVector3 origin = btVector3(0., 0., 0.), btVector3 rotation = btVector3(0., 0., 0.));


#endif