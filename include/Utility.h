#ifndef UTILITY_H
#define UTILITY_H

#include "Instrumentor.h"
#define ensure(x, ...) {if(!(x)) {__debugbreak();}}

#define PI 3.14159265358979323846
#define BIT(x) (1<<(x))

#define RED btVector3(1., 0., 0.)
#define ORANGE btVector3(255./255., 69./255., 0./255.)
#define GREEN btVector3(60./255., 179./255., 113./255.)
#define BLUE btVector3(65./255., 105./255., 225./255.)
#define YELLOW btVector3(255./255., 215./255., 0./255.)

template <typename>
class btAlignedObjectArray;

enum collisiontypes {
	COL_NOTHING = 0, //Collide with nothing
	COL_FOLLICLE = BIT(0),
	COL_INT_MUS = BIT(1),
	COL_EXT_MUS = BIT(2),
};

// unclear bug: why set extMusCollideWith = COL_EXT_MUS only
// will not allow user to drag the object using mouse cursor
static int follicleCollideWith = COL_FOLLICLE;
static int intMusCollideWith = COL_INT_MUS;
static int extMusCollideWith = COL_FOLLICLE;

btRigidBody* createDynamicBody(const float mass, const btTransform& startTransform, btCollisionShape* shape, btScalar damping = 0.f);
btTransform createTransform(const btVector3& origin = btVector3(0., 0., 0.), const btVector3& rotation = btVector3(0., 0., 0.));
//float getCriticalDampingRatio(const float m1, const float m2, const float k);

void read_csv_float(const std::string& fileName, std::vector<std::vector<float>>& dataList);
void read_csv_int(const std::string& fileName, std::vector<std::vector<int>>& dataList);
void write_csv_float(const std::string& folderName, const std::string& fileName, std::vector<std::vector<double>>& dataList);
void write_csv_float(const std::string& folderName, const std::string& fileName, std::vector<double>& dataList);
void write_txt(const std::string& folderName, const std::string& fileName, const std::string& text);

bool isPathExist(const std::string& s);

template <typename T>
void freeAlignedObjectArray(btAlignedObjectArray<T*>& arr) {
	int sz = arr.size();
	for (int i = 0; i < arr.size(); i++) {
		delete arr[i];
	}
	arr.clear();
}

btScalar interp1(std::vector<btScalar>& xData, std::vector<btScalar>& yData, btScalar x);

// test outputer singelton
class S_dumpster {
private:
	S_dumpster() {}

public:
	S_dumpster(S_dumpster const&) = delete;
	void operator=(S_dumpster const&) = delete;

	static S_dumpster& Get() {
		static S_dumpster* s_dumpster = new S_dumpster();
		return *s_dumpster;
	}

	static std::vector<std::vector<btScalar>> fiber_info;
	static std::vector<btScalar> hamiltonian;
	static std::vector<std::vector<btScalar>> test_info;

};





#endif