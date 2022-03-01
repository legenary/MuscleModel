#ifndef UTILITY_H
#define UTILITY_H

//#include "btBulletDynamicsCommon.h"
//#include "btBulletCollisionCommon.h"
//#include "LinearMath/btVector3.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"


#define PI 3.14159265358979323846
#define BIT(x) (1<<(x))

enum collisiontypes {
	COL_NOTHING = 0, //Collide with nothing
	COL_FOLLICLE = BIT(0),
	COL_INT_MUS = BIT(1),
	COL_EXT_MUS = BIT(2),
};

// unclear bug: why set extMusCollideWith = COL_EXT_MUS only will not allow user to drag the object using mouse cursor
static int follicleCollideWith = COL_FOLLICLE;
static int intMusCollideWith = COL_INT_MUS;
static int extMusCollideWith = COL_FOLLICLE;


btRigidBody* createDynamicBody(float mass, const btTransform& startTransform, btCollisionShape* shape);
btTransform createTransform(btVector3 origin = btVector3(0., 0., 0.), btVector3 rotation = btVector3(0., 0., 0.));
float getCriticalDampingRatio(float m1, float m2, float k);

void read_csv_float(std::string fileName, std::vector<std::vector<float>>& dataList);
void read_csv_int(std::string fileName, std::vector<std::vector<int>>& dataList);

void write_csv_float(std::string folderName, std::string fileName, std::vector<std::vector<float>>& dataList);

bool isPathExist(const std::string& s);

#endif