#ifndef EXTRINSIC_MUSCLE_H
#define EXTRINSIC_MUSCLE_H

#include "Utility.h"
#include "Parameter.h"
#include "Spring.h"
#include "Follicle.h"

#include "LinearMath/btAlignedObjectArray.h"

class ExtrinsicMuscle {
private:
	int nNodes;
	int nMusclePieces;
	int nInsertionPieces;
	btAlignedObjectArray<btRigidBody*> m_nodes;
	btAlignedObjectArray<Spring*> m_musclePieces;
	btAlignedObjectArray<Spring*> m_insertionPieces;

public:
	// remember to add: a slider constraint (or point-to-point anchor) at the end of the extrinsic muscle bundle
	ExtrinsicMuscle(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, Parameter* param,
		btAlignedObjectArray<Follicle*> follicleArray, std::vector<std::vector<float>> NODE_POS,
		std::vector<std::vector<int>> CONSTRUCTION_IDX, std::vector<std::vector<int>> INSERTION_IDX, btScalar top = 1);

	void contract(btScalar ratio);
	void update();
	void debugDraw(btDiscreteDynamicsWorld* world, btVector3 clr = btVector3(1., 0., 0.));

	int getNumberOfNodes();
	int getNumberOfMusclePieces();
	int getNumberOfInsertionPices();



};






#endif