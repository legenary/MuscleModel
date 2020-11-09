#include "ExtrinsicMuscle.h"

ExtrinsicMuscle::ExtrinsicMuscle(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param,
	btAlignedObjectArray<Follicle*> m_follicleArray, std::vector<std::vector<float>> NODE_POS, std::vector<std::vector<int>> CONSTRUCTION_IDX, 
	std::vector<std::vector<int>> INSERTION_IDX, std::vector<std::vector<float>> INSERTION_HEIGHT) {

	// create extrinsic muscle nodes
	nNodes = NODE_POS.size();
	for (int i = 0; i < nNodes; i++) {
		btTransform t = createTransform(btVector3(NODE_POS[i][0], NODE_POS[i][1], NODE_POS[i][2]));
		btCollisionShape* s = new btSphereShape(0.01);
		btRigidBody* b = createDynamicBody(0.1, t, s);
		m_nodes.push_back(b);
		m_dynamicsWorld->addRigidBody(b, COL_EXT_MUS, extMusCollideWith);
		b->setActivationState(DISABLE_DEACTIVATION);
	}
	// construct muscle structural springs
	nMusclePieces = CONSTRUCTION_IDX.size();
	for (int i = 0; i < nMusclePieces; i++) {
		btRigidBody* node1 = m_nodes[CONSTRUCTION_IDX[i][0]];
		btRigidBody* node2 = m_nodes[CONSTRUCTION_IDX[i][1]];
		Spring* spring = new Spring(node1, node2, createTransform(), createTransform(), param->k_nasolabialis, param->damping);
		m_dynamicsWorld->addConstraint(spring->getConstraint(), true);
		m_musclePieces.push_back(spring);
	}
	// construct muscle insertion to follicle
	int nInsertionGroups = INSERTION_IDX.size();
	for (int i = 0; i < nInsertionGroups; i++) {
		btRigidBody* node = m_nodes[INSERTION_IDX[i][0]];\
		// Insertion height: the height of point where the follicle is inserted by the muscle, range [-1, 1]*fol_half_height
		// Default insertion height: 1, otherwise check INSERTION HEIGHT
		btTransform trans = createTransform(btVector3((INSERTION_HEIGHT.size()?INSERTION_HEIGHT[i][1]:1) * param->fol_half_height, 0., 0.));
		for (int f = 1; f < 3; f++) {
			if (INSERTION_IDX[i][f] >= 0) {
				btRigidBody* body = m_follicleArray[INSERTION_IDX[i][f]]->getBody();
				Spring* spring = new Spring(node, body, createTransform(), trans, param->k_nasolabialis, param->damping);
				m_dynamicsWorld->addConstraint(spring->getConstraint(), true);
				m_insertionPieces.push_back(spring);
			}
		}
	}
	nInsertionPieces = m_insertionPieces.size();

}

void ExtrinsicMuscle::contract(btScalar ratio) {
	for (int i = 0; i < nMusclePieces; i++) {
		m_musclePieces[i]->setRestLength(ratio);
	}
}

void ExtrinsicMuscle::update() {
	for (int i = 0; i < nMusclePieces; i++) {
		m_musclePieces[i]->update();
	}
	for (int i = 0; i < nInsertionPieces; i++) {
		m_insertionPieces[i]->update();
	}
}

void ExtrinsicMuscle::debugDraw(btDiscreteDynamicsWorld* m_dynamicsWorld, btVector3 clr) {
	for (int i = 0; i < nMusclePieces; i++) {
		m_musclePieces[i]->debugDraw(m_dynamicsWorld, clr);
	}
	for (int i = 0; i < nInsertionPieces; i++) {
		m_insertionPieces[i]->debugDraw(m_dynamicsWorld, clr);
	}
}

int ExtrinsicMuscle::getNumberOfNodes() {
	return nNodes;
}
int ExtrinsicMuscle::getNumberOfMusclePieces() {
	return nMusclePieces;
}
int ExtrinsicMuscle::getNumberOfInsertionPices() {
	return nInsertionPieces;
}