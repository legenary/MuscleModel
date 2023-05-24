#ifndef UTILITY_H
#define UTILITY_H

#include "Instrumentor.h"

#define PI 3.14159265358979323846
#define BIT(x) (1<<(x))

#define RED btVector3(1., 0., 0.)
#define GREEN btVector3(0., 1., 0.)
#define BLUE btVector3(0., 0., 1.)


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

// scoped timer
struct Timer {
	std::chrono::time_point<std::chrono::steady_clock> start, end;
	std::chrono::duration<float> duration;
	std::string message = "no msg";

	Timer() : start(std::chrono::high_resolution_clock::now()) {}
	Timer(std::string msg) : start(std::chrono::high_resolution_clock::now()), message(msg) {}

	~Timer() {
		end = std::chrono::high_resolution_clock::now();
		duration = end - start;
		float ms = duration.count() * 1000.f;
		std::cout << "(" << message << ") Timer took: " << ms << "ms\n";
	}
};

btRigidBody* createDynamicBody(const float mass, const btTransform& startTransform, btCollisionShape* shape, btScalar damping = 0.f);
btTransform createTransform(const btVector3& origin = btVector3(0., 0., 0.), const btVector3& rotation = btVector3(0., 0., 0.));
//float getCriticalDampingRatio(const float m1, const float m2, const float k);

void read_csv_float(const std::string& fileName, std::vector<std::vector<float>>& dataList);
void read_csv_int(const std::string& fileName, std::vector<std::vector<int>>& dataList);
void write_csv_float(const std::string& folderName, const std::string& fileName, std::vector<std::vector<double>>& dataList);
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
	static std::vector<std::vector<btScalar>> test_info;

};





#endif