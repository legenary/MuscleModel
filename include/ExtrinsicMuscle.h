#ifndef EXTRINSIC_MUSCLE_H
#define EXTRINSIC_MUSCLE_H

//#include "LinearMath/btAlignedObjectArray.h"
#include "Simulation.h"

class Tissue;
class Fiber;
class Parameter;
class Follicle;

class ExtrinsicMuscle {
private:
	Simulation* m_sim;
	Parameter* m_parameter;

	int nNodes;
	int nMusclePieces;
	int nInsertionPieces;
	btAlignedObjectArray<btRigidBody*> m_nodes;
	btAlignedObjectArray<Fiber*> m_musclePieces;
	btAlignedObjectArray<Tissue*> m_insertionPieces;
	

public:
	// remember to add: a slider constraint (or point-to-point anchor) at the end of the extrinsic muscle bundle
	ExtrinsicMuscle(Simulation* sim, Parameter* param,
		btAlignedObjectArray<Follicle*>& follicleArray, 
		std::vector<std::vector<float>>& NODE_POS, 
		std::vector<std::vector<int>>& CONSTRUCTION_IDX, 
		std::vector<std::vector<int>>& INSERTION_IDX, 
		std::vector<std::vector<float>>& INSERTION_HEIGHT,
		std::set<int>& anchorNodeIdx = std::set<int>{ 0 });
	// disable copy constructor (override if needed in the future)
	ExtrinsicMuscle(const ExtrinsicMuscle&) = delete;
	ExtrinsicMuscle& operator=(ExtrinsicMuscle const&) = delete;
	virtual ~ExtrinsicMuscle();

	void contractTo(btScalar ratio);
	void update();
	void debugDraw(btVector3 clr = btVector3(0., 0., 0.));

	int getNumberOfNodes() const;
	int getNumberOfMusclePieces() const;
	int getNumberOfInsertionPices() const;

	inline btDynamicsWorld* getWorld() {
		return m_sim->getDynamicsWorld();
	}
	inline btAlignedObjectArray<btCollisionShape*>* getCollisionShapes() {
		return &(m_sim->m_collisionShapes);
	}
};


// fPE lookup table for interpolation



#endif