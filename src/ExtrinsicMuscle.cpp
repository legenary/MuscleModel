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
		// reorient node2 in the direction of node1
		btVector3 xaxis = (node1->getCenterOfMassTransform().getOrigin() - node2->getCenterOfMassTransform().getOrigin()).normalized();
		btVector3 yaxis = xaxis.cross(btVector3(0.f, 0.f, 1.f));
		btVector3 zaxis = xaxis.cross(yaxis);
		btTransform new_trans = btTransform(btMatrix3x3({
				xaxis.x(), yaxis.x(), zaxis.x(),
				xaxis.y(), yaxis.y(), zaxis.y(),
				xaxis.z(), yaxis.z(), zaxis.z(),
			}), node2->getCenterOfMassTransform().getOrigin());
		node2->setWorldTransform(new_trans);

		// add muscle fiber
		btScalar eq_factor = CONSTRUCTION_IDX[i][2];
		Fiber* fiber = new Fiber(m_sim, node1, node2, 
			createTransform(), createTransform(),
			f0 * eq_factor);
		getWorld()->addConstraint(fiber->getConstraint(), true);
		m_musclePieces.push_back(fiber);

		/////////////////////////////////////////////
		// experimental: addtional stiffness to help each strain of extrinsic muscle maintain its shape. Otherwise between nodes and nodes they can bend freely
		btGeneric6DofSpring2Constraint* node_constraint = new btGeneric6DofSpring2Constraint(*node2, *node1, createTransform(), createTransform());
		node_constraint->setLinearLowerLimit(btVector3(1, 1, 1));	// need to set lower > higher to free the dofs
		node_constraint->setLinearUpperLimit(btVector3(0, 0, 0));
		node_constraint->setAngularLowerLimit(btVector3(1, 1, 1));
		node_constraint->setAngularUpperLimit(btVector3(0, 0, 0));
		btScalar stiffness = 3.f;
		btScalar damping = 2.0f * btSqrt((node1->getMass() + node2->getMass()) / 2.0f * stiffness) * 1.0f;
		// allow free node movement in the x direction (in node2's frame), but add spring to limit movement in the y and z direction
		node_constraint->enableSpring(0, true);
		node_constraint->setStiffness(0, 0);
		node_constraint->setDamping(0, 2);
		node_constraint->setEquilibriumPoint(0);
		for (int i = 1; i < 3; i++) {
			node_constraint->enableSpring(i, true);
			node_constraint->setStiffness(i, stiffness);
			node_constraint->setDamping(i, damping);
			node_constraint->setEquilibriumPoint(i);
		}
		getWorld()->addConstraint(node_constraint, true);
		///////////////////////////////////////////
	}
	// construct muscle insertion tissue to follicle
	int nInsertionGroups = INSERTION_IDX.size();
	for (int i = 0; i < nInsertionGroups; i++) {
		btRigidBody* node = m_nodes[INSERTION_IDX[i][0]];
		// Insertion height: the height of point where the follicle is inserted by the muscle, range [-1, 1]*fol_half_height
		for (int f : {1, 2}) {
			if (INSERTION_IDX[i][f] >= 0) {
				if (const auto& fol = m_pad->getFollicleByIndex(INSERTION_IDX[i][f])) {
					btRigidBody* body = fol->getBody();
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
				else {
					// follicle is skipped during initialization
					m_insertionPieces.push_back(nullptr);
				}
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
		if (m_insertionPieces[i]) {
			m_insertionPieces[i]->update();
			m_Hamiltonian += m_insertionPieces[i]->getHamiltonian();
		}
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
		if (m_musclePieces[i]) {
			m_musclePieces[i]->debugDraw(clr);
		}
	}
	for (int i = 0; i < nInsertionPieces; i++) {
		if (m_insertionPieces[i]) {
			m_insertionPieces[i]->debugDraw(clr);
		}
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

btRigidBody* ExtrinsicMuscle::getNodeByIndex(int idx) const {
	ensure(idx < nNodes);
	return m_nodes[idx];
}

Fiber* ExtrinsicMuscle::getFiberByIndex(int idx) const {
	ensure(idx < nMusclePieces);
	return m_musclePieces[idx];
}