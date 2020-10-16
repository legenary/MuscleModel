#include "Utility.h"

btRigidBody* createDynamicBody(float mass, const btTransform& startTransform, btCollisionShape* shape) {
	// rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic) {
		shape->calculateLocalInertia(mass, localInertia);
	}
	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);
	//cInfo.m_restitution = restitution;
	//cInfo.m_friction = friction;
	btRigidBody* body = new btRigidBody(cInfo);
	body->setUserIndex(-1);

	return body;
}

btTransform createTransform(btVector3 origin, btVector3 YPR) {
	// rotation: EuelrZYX rotation orders

	btTransform trans;
	trans = btTransform::getIdentity();
	trans.setOrigin(origin);
	trans.getBasis().setEulerYPR(YPR[0], YPR[1], YPR[2]);
	return trans;
}

void read_csv_float(std::string fileName, std::vector<std::vector<float>> &dataList){

	std::fstream data(fileName);
	std::string line;
	while (std::getline(data, line)) {
		std::stringstream linestream(line);
		std::string cell;
		std::vector<float> parsedRow;
		while (std::getline(linestream, cell, ','))
		{
			float cellFloat = stof(cell);
			parsedRow.push_back(cellFloat);
		}
		dataList.push_back(parsedRow);
	}
	
}
