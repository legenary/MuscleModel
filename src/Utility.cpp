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

float getCriticalDampingRatio(float m1, float m2, float k) {
	// get damping ratio (in Bullet)
	// when m1 = m2 = 1, k = 1, the critical damping ratio is 0.0465
	// damping ratio ~ sqrt( (m1 + m2) / (m1 * m2  * k) )
	float fold = std::sqrt((m1 + m2) / m1 / m2 / k);
	return 0.0465 * fold / std::sqrt(2);

}

void read_csv_float(std::string fileName, std::vector<std::vector<float>>& dataList){

	std::fstream data(fileName);
	std::string line;
	while (std::getline(data, line)) {
		std::stringstream linestream(line);
		std::string cell;
		std::vector<float> parsedRow;
		while (std::getline(linestream, cell, ','))
		{
			parsedRow.push_back(stof(cell));
		}
		dataList.push_back(parsedRow);
	}
	
}

void read_csv_int(std::string fileName, std::vector<std::vector<int>>& dataList) {

	std::fstream data(fileName);
	std::string line;
	while (std::getline(data, line)) {
		std::stringstream linestream(line);
		std::string cell;
		std::vector<int> parsedRow;
		while (std::getline(linestream, cell, ','))
		{
			parsedRow.push_back(stoi(cell));
		}
		dataList.push_back(parsedRow);
	}

}

void write_csv_float(std::string folderName, std::string fileName, std::vector<std::vector<float>>& dataList) {
	try {
		if (!isPathExist(folderName)) {	// create folder if not exist
			mkdir(folderName.c_str());
			std::cout << "Creating output folder..." << std::endl;
		}
		// outputing...
		std::ofstream outputFile;
		outputFile.open(folderName + "/" + fileName);
		for (int row = 0; row < dataList.size(); row++) {
			for (int col = 0; col < dataList[row].size(); col++) {
				outputFile << dataList[row][col] << ",";
			}
			outputFile << std::endl;
		}
		outputFile.close();

	}
	catch (...) {
		std::cout << "-saving csv float failed." << std::endl;
	}
}

bool isPathExist(const std::string& s)
{
	struct stat buffer;
	return (stat(s.c_str(), &buffer) == 0);
}
