#include "my_pch.h"
#include "Utility.h"

#include "CommonInterfaces/CommonGUIHelperInterface.h"

btRigidBody* createDynamicBody(const float mass, const btTransform& startTransform, btCollisionShape* shape, btScalar damping) {
	// rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic) {
		shape->calculateLocalInertia(mass, localInertia);
	}
	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);
	
	if (isDynamic) {
		//cInfo.m_restitution = restitution;
		//cInfo.m_friction = friction;
		cInfo.m_linearDamping = damping; // 0 by default, no damping
		//TODO: add damping differently for follicle and muscle nodes
	}
	
	btRigidBody* body = new btRigidBody(cInfo);
	
	return body;
}

btTransform createTransform(const btVector3& origin, const btVector3& YPR) {
	// rotation: EuelrZYX rotation orders

	btTransform trans;
	trans = btTransform::getIdentity();
	trans.setOrigin(origin);
	trans.getBasis().setEulerYPR(YPR[0], YPR[1], YPR[2]);
	return trans;
}

float getCriticalDampingRatio(const float m1, const float m2, const float k) {
	// get damping ratio (in Bullet)
	// when m1 = m2 = 1, k = 1, the critical damping ratio is 0.0465
	// damping ratio ~ sqrt( (m1 + m2) / (m1 * m2  * k) )
	float fold = std::sqrt((m1 + m2) / m1 / m2 / k);
	return 0.0465 * fold / std::sqrt(2);

}

void read_csv_float(const std::string& fileName, std::vector<std::vector<float>>& dataList){

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

void read_csv_int(const std::string& fileName, std::vector<std::vector<int>>& dataList) {

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

void write_csv_float(const std::string& folderName, const std::string& fileName, std::vector<std::vector<double>>& dataList) {
	try {
		if (!isPathExist(folderName)) {	// create folder if not exist
			mkdir(folderName.c_str());
			std::cout << "Creating output folder..." << std::endl;
		}
		// outputing...
		std::ofstream outputFile;
		outputFile.open(folderName + "/" + fileName);
		int m = dataList.size(), n = dataList[0].size();
		for (int row = 0; row < m; row++) {
			for (int col = 0; col < n; col++) {
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

void write_txt(const std::string& folderName, const std::string& fileName, const std::string& text) {
	try {
		std::ofstream out(folderName + "/" + fileName);
		out.clear();
		out << text;
		out.close();
	}
	catch (...) {
		std::cout << "-saving text file failed." << std::endl;
	}
}


bool isPathExist(const std::string& s)
{
	struct stat buffer;
	return (stat(s.c_str(), &buffer) == 0);
}

// assume xData is sorted and strictly monotonic increasing
// if x outside of range, take extreme values
btScalar interp1(std::vector<btScalar>& xData, std::vector<btScalar>& yData, btScalar x) {

	if (xData.size() != yData.size())
		throw std::invalid_argument("xData.size() != yData.size()");

	int sz = xData.size();
	// if out of range
	if (x <= xData[0]) {
		return yData[0];
	}
	else if (x >= xData[sz - 1]) {
		return yData[sz-1];
	}

	// find interval xData[i-1] <= x < xData[i]
	int i = 0;
	while (xData[i] <= x) i++;
	
	// interpolate
	btScalar xL = xData[i - 1], xR = xData[i], yL = yData[i - 1], yR = yData[i];
	return yL + (yR - yL) / (xR - xL)*(x - xL);
}


