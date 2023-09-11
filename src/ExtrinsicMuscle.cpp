#include "my_pch.h"
#include "ExtrinsicMuscle.h"
#include "myGeneric6DofMuscleConstraint.h"

#include "Utility.h"
#include "Parameter.h"
#include "Tissue.h"
#include "Fiber.h"
#include "Follicle.h"


ExtrinsicMuscle::ExtrinsicMuscle(btScalar _f0, Simulation* sim, Parameter* param, MystacialPad* pad,
	std::vector<std::vector<float>>& NODE_POS,
	std::vector<std::vector<int>>& CONSTRUCTION_IDX,
	std::vector<std::vector<int>>& INSERTION_IDX,
	std::vector<std::vector<float>>& INSERTION_HEIGHT,
	std::set<int>& anchorNodeIdx)
	: f0(_f0) , m_sim(sim) , m_parameter(param) , m_pad(pad) {

	// create extrinsic muscle nodes
	nNodes = NODE_POS.size();
	for (int i = 0; i < nNodes; i++) {
		btTransform t = createTransform(btVector3(NODE_POS[i][0], NODE_POS[i][1], NODE_POS[i][2]));
		btCollisionShape* s = new btSphereShape((anchorNodeIdx.count(i)) ? 0.1 : 0.01 /*radius*/);
		btScalar mass = (anchorNodeIdx.count(i)) ? 0. : f0*btScalar(0.006667) /*node mass. Mass of follicle is ~0.001g*/;
		btRigidBody* b = createDynamicBody(mass, t, s, 0 /*no damping*/);
		m_nodes.push_back(b);
		getWorld()->addRigidBody(b, COL_EXT_MUS, extMusCollideWith);
		b->setActivationState(DISABLE_DEACTIVATION);
		// add anchor tissue to the anchor nodes
		// (attach end of muscle to skull/cartilage)
		if (anchorNodeIdx.count(i)) {
			Tissue* anchor = new Tissue(m_sim, m_nodes[i], createTransform(), 
				m_parameter->k_anchor_translational, 0.0f /*no torsional*/, m_parameter->zeta_anchor_translational, 0.0f /*no torsional*/);	//this is a linear spring only
			getWorld()->addConstraint(anchor->getConstraint(), true); // disable collision
		}
	}
	// construct muscle structural fibers
	nMusclePieces = CONSTRUCTION_IDX.size();
	ensure(nMusclePieces > 0 && CONSTRUCTION_IDX[0].size() == 3);
	for (int i = 0; i < nMusclePieces; i++) {
		btRigidBody* node1 = m_nodes[CONSTRUCTION_IDX[i][0]];
		btRigidBody* node2 = m_nodes[CONSTRUCTION_IDX[i][1]];
		btScalar eq_factor = CONSTRUCTION_IDX[i][2];
		//node2->setMassProps(node2->getMass() * eq_factor/5, node2->getLocalInertia());
		Fiber* fiber = new Fiber(m_sim, node1, node2, 
			createTransform(), createTransform(),
			f0 * eq_factor);
		getWorld()->addConstraint(fiber->getConstraint(), true);
		m_musclePieces.push_back(fiber);
	}
	// construct muscle insertion tissue to follicle
	int nInsertionGroups = INSERTION_IDX.size();
	for (int i = 0; i < nInsertionGroups; i++) {
		btRigidBody* node = m_nodes[INSERTION_IDX[i][0]];
		// Insertion height: the height of point where the follicle is inserted by the muscle, range [-1, 1]*fol_half_height
		for (int f : {1, 2}) {
			if (INSERTION_IDX[i][f] >= 0) {
				btRigidBody* body = m_pad->getFollicleByIndex(INSERTION_IDX[i][f])->getBody();
				// Default insertion height: 1, otherwise check INSERTION HEIGHT
				btScalar insertion_height = 
					(INSERTION_HEIGHT.size() ? INSERTION_HEIGHT[i][1] : 1) * 
					param->FOLLICLE_POS_ORIENT_LEN_VOL[INSERTION_IDX[i][f]][6] / 2;
				btTransform trans = createTransform(btVector3(insertion_height, 0., 0.));
				Tissue* tissue = new Tissue(m_sim, 
					node, body, createTransform(), trans, 
					m_parameter->k_layer1, 0.0f /*no torsional*/, m_parameter->zeta_layer, 0.0f /*no torsional*/);
				getWorld()->addConstraint(tissue->getConstraint(), true);
				m_insertionPieces.push_back(tissue);
			}
		}
	}
	nInsertionPieces = m_insertionPieces.size();

	
}


ExtrinsicMuscle::~ExtrinsicMuscle() {
	//elements of m_nodes gets deleted with m_dynamicsworld so no deletion here.
	freeAlignedObjectArray(m_musclePieces);
	freeAlignedObjectArray(m_insertionPieces);
}

void ExtrinsicMuscle::contractTo(btScalar ratio) {
	for (int i = 0; i < nMusclePieces; i++) {
		m_musclePieces[i]->contractTo(ratio);
	}
}

void ExtrinsicMuscle::update(bool updateFiber) {
	m_Hamiltonian = 0;
	for (int i = 0; i < nInsertionPieces; i++) {
		m_insertionPieces[i]->update();
		m_Hamiltonian += m_insertionPieces[i]->getHamiltonian();
	}
	if (updateFiber) {
		for (int i = 0; i < nMusclePieces; i++) {
			m_musclePieces[i]->update();
			m_Hamiltonian += m_musclePieces[i]->getHamiltonian();
		}
	}
	for (int i = 0; i < nNodes; i++) {
		btScalar mass = m_nodes[i]->getMass();
		btVector3 vel = m_nodes[i]->getLinearVelocity();
		m_Hamiltonian += 0.5 * mass * vel.length2();
		// remark: ignore node rotation for calculating total energy. Theoretically, there shouldn't be any rotation.
	}
}

void ExtrinsicMuscle::debugDraw(btVector3 clr) {
	for (int i = 0; i < nMusclePieces; i++) {
		m_musclePieces[i]->debugDraw(clr);
	}
	for (int i = 0; i < nInsertionPieces; i++) {
		m_insertionPieces[i]->debugDraw(clr);
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

btRigidBody* ExtrinsicMuscle::getNodeByIndex(int idx) {
	ensure(idx < nNodes);
	return m_nodes[idx];
}