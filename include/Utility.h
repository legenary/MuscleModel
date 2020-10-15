#ifndef UTILITY_H
#define UTILITY_H

#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"

#include "LinearMath/btVector3.h"

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>

#define PI 3.14159265358979323846
#define BIT(x) (1<<(x))

enum collisiontypes {
	COL_NOTHING = 0, //Collide with nothing
	COL_FOLLICLE = BIT(0), //Collide with head
	COL_INT_MUS = BIT(1), //Collide with follicles
	COL_EXT_MUS = BIT(2),		// Collide with base
};

static int follicleCollideWith = COL_FOLLICLE;
static int intMusCollideWith = COL_FOLLICLE | COL_EXT_MUS;
static int extMusCollideWith = COL_FOLLICLE | COL_INT_MUS;


btRigidBody* createDynamicBody(float mass, const btTransform& startTransform, btCollisionShape* shape);

btTransform createTransform(btVector3 origin = btVector3(0., 0., 0.), btVector3 rotation = btVector3(0., 0., 0.));


void read_csv_float(std::string fileName, std::vector<std::vector<float> > &dataList);


#endif