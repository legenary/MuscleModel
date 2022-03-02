#include "my_pch.h"
#include "ExtrinsicMuscle.h"

#include "Utility.h"
#include "Parameter.h"
#include "Spring.h"
#include "Follicle.h"

ExtrinsicMuscle::ExtrinsicMuscle(btDiscreteDynamicsWorld* m_dynamicsWorld, 
	btAlignedObjectArray<btCollisionShape*>* m_collisionShapes, 
	Parameter* param,
	btAlignedObjectArray<Follicle*>& m_follicleArray, 
	std::vector<std::vector<float>>& NODE_POS, 
	std::vector<std::vector<int>>& CONSTRUCTION_IDX, 
	std::vector<std::vector<int>>& INSERTION_IDX, 
	std::vector<std::vector<float>>& INSERTION_HEIGHT) {

	// create extrinsic muscle nodes
	nNodes = NODE_POS.size();
	for (int i = 0; i < nNodes; i++) {
		btTransform t = createTransform(btVector3(NODE_POS[i][0], NODE_POS[i][1], NODE_POS[i][2]));
		btCollisionShape* s = new btSphereShape((i == 0) ? 0.1 : 0.01);
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
		SpringBetween* spring = new SpringBetween(
			node1, node2, createTransform(), createTransform(),
			param->k_nasolabialis, param->damping);
		m_dynamicsWorld->addConstraint(spring->getConstraint(), true);
		m_musclePieces.push_back(spring);
	}
	// construct muscle insertion to follicle
	int nInsertionGroups = INSERTION_IDX.size();
	for (int i = 0; i < nInsertionGroups; i++) {
		btRigidBody* node = m_nodes[INSERTION_IDX[i][0]];\
		// Insertion height: the height of point where the follicle is inserted by the muscle, range [-1, 1]*fol_half_height
		for (int f = 1; f < 3; f++) {
			if (INSERTION_IDX[i][f] >= 0) {
				btRigidBody* body = m_follicleArray[INSERTION_IDX[i][f]]->getBody();
				// Default insertion height: 1, otherwise check INSERTION HEIGHT
				btTransform trans = createTransform(btVector3((INSERTION_HEIGHT.size() ? INSERTION_HEIGHT[i][1] : 1) * param->FOLLICLE_POS_ORIENT_LEN_VOL[INSERTION_IDX[i][f]][6] / 2, 0., 0.));
				SpringBetween* spring = new SpringBetween(
					node, body, createTransform(), trans, 
					param->k_nasolabialis, param->damping);
				m_dynamicsWorld->addConstraint(spring->getConstraint(), true);
				m_insertionPieces.push_back(spring);
			}
		}
	}
	nInsertionPieces = m_insertionPieces.size();

	// construct muscle end anchoring (to skull/cartilage)
	SpringAnchor* anchor = new SpringAnchor(m_nodes[0], createTransform(), param->k_anchor, param->damping);	//this is a linear + torsional spring
	m_dynamicsWorld->addConstraint(anchor->getConstraint(), true); // disable collision
}


ExtrinsicMuscle::~ExtrinsicMuscle() {
	//elements of m_nodes gets deleted with m_dynamicsworld so no deletion here.
	freeAlignedObjectArray(m_musclePieces);
	freeAlignedObjectArray(m_insertionPieces);
}

void ExtrinsicMuscle::contract(btScalar ratio) {
	for (int i = 0; i < nMusclePieces; i++) {
		m_musclePieces[i]->setRestLength(ratio);
	}
}

void ExtrinsicMuscle::contract(btScalar ratio, std::vector<int>& those) {
	for (auto& that : those) {
		m_musclePieces[that]->setRestLength(ratio);
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

int ExtrinsicMuscle::getNumberOfNodes() const {
	return nNodes;
}
int ExtrinsicMuscle::getNumberOfMusclePieces() const {
	return nMusclePieces;
}
int ExtrinsicMuscle::getNumberOfInsertionPices() const {
	return nInsertionPieces;
}