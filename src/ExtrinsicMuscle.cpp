#include "ExtrinsicMuscle.h"

ExtrinsicMuscle::ExtrinsicMuscle(btDiscreteDynamicsWorld* m_dynamicsWorld, btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, Parameter* param,
	btAlignedObjectArray<Follicle*> m_follicleArray, std::vector<std::vector<float>> NODE_POS,
	std::vector<std::vector<int>> CONSTRUCTION_IDX, std::vector<std::vector<int>> INSERTION_IDX) {

	// create extrinsic muscle nodes
	// (remember: change this to createDynamic Body)
	nNodes = NODE_POS.size();
	for (int i = 0; i < nNodes; i++) {
		btTransform t = createTransform(btVector3(NODE_POS[i][0], NODE_POS[i][1], NODE_POS[i][2]));
		btCollisionShape* s = new btSphereShape(0.1);
		m_collisionShapes->push_back(s);
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
		btRigidBody* node = m_nodes[INSERTION_IDX[i][0]];
		btTransform trans = createTransform(btVector3(param->fol_half_height, 0., 0.));
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