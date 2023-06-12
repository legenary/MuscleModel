#include "my_pch.h"
#include "Layer.h"

#include "Parameter.h"
#include "Tissue.h"
#include "Follicle.h"
#include "Utility.h"

Layer::~Layer() {
	freeAlignedObjectArray(m_edges);
	freeAlignedObjectArray(m_bendings);
	freeAlignedObjectArray(m_anchors);
}

void Layer::initEdges(bool isTop) {

	nEdges = m_param->SPRING_HEX_MESH_IDX.size();

	for (int s = 0; s < nEdges; s++) {
		Follicle* fol1 = m_pad->getFollicleByIndex(m_param->SPRING_HEX_MESH_IDX[s][0]);
		Follicle* fol2 = m_pad->getFollicleByIndex(m_param->SPRING_HEX_MESH_IDX[s][1]);

		btTransform frameLayer1fol1 = createTransform(btVector3((isTop ? 1 : -1) * m_param->FOLLICLE_POS_ORIENT_LEN_VOL[m_param->SPRING_HEX_MESH_IDX[s][0]][6] / 2, 0., 0.));
		btTransform frameLayer1fol2 = createTransform(btVector3((isTop ? 1 : -1) * m_param->FOLLICLE_POS_ORIENT_LEN_VOL[m_param->SPRING_HEX_MESH_IDX[s][1]][6] / 2, 0., 0.));

		btScalar k_eq = m_param->k_layer1;
		btScalar k_this = k_eq / 2;
		Tissue* edge = new Tissue(m_sim, fol1->getBody(), fol2->getBody(), frameLayer1fol1, frameLayer1fol2, k_this, m_param->zeta_tissue);

		getWorld()->addConstraint(edge->getConstraint(), true); // disable collision
		m_edges.push_back(edge);
	}

	nTissues += nEdges;
}

void Layer::initAnchors(bool isTop) {
	nAnchors = m_pad->getNumFollicles();
	for (int f = 0; f < nAnchors; f++) {
		Follicle* fol = m_pad->getFollicleByIndex(f);
		btTransform frameAnchor = createTransform(btVector3((isTop ? 1 : -1) * m_param->FOLLICLE_POS_ORIENT_LEN_VOL[f][6] / 2, 0., 0.));
		Tissue* anchor = new Tissue(m_sim, fol->getBody(), frameAnchor, m_param->k_anchor, m_param->zeta_tissue);	// this is a linear + torsional spring

		getWorld()->addConstraint(anchor->getConstraint(), true); // disable collision
		m_anchors.push_back(anchor);
	}
}


void Layer::initBendings(bool isTop) {

	nBendings = m_param->SPRING_BENDING_IDX.size();
	for (int s = 0; s < nBendings; s++) {
		Follicle* fol1 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[s][0]);
		Follicle* fol2 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[s][1]);
		btTransform frameLayer1fol1 = createTransform(btVector3((isTop ? 1 : -1) * m_param->FOLLICLE_POS_ORIENT_LEN_VOL[m_param->SPRING_BENDING_IDX[s][0]][6] / 2, 0., 0.));
		btTransform frameLayer1fol2 = createTransform(btVector3((isTop ? 1 : -1) * m_param->FOLLICLE_POS_ORIENT_LEN_VOL[m_param->SPRING_BENDING_IDX[s][1]][6] / 2, 0., 0.));

		btScalar k_eq = m_param->k_layer1;
		btScalar k_this = k_eq / 2;
		Tissue* bending = new Tissue(m_sim, fol1->getBody(), fol2->getBody(), frameLayer1fol1, frameLayer1fol2, k_this, m_param->zeta_tissue);

		getWorld()->addConstraint(bending->getConstraint(), true); // disable collision
		m_bendings.push_back(bending);
	}

	nTissues += nBendings;
}

void Layer::update() {
	for (int i = 0; i < nEdges; i++) {
		m_edges[i]->update();
	}
	for (int i = 0; i < nAnchors; i++) {
		m_anchors[i]->update();
	}
	for (int i = 0; i < nBendings; i++) {
		m_bendings[i]->update();
	}
}

btDynamicsWorld* Layer::getWorld() {
	return m_sim->getDynamicsWorld();
}

void Layer::debugDraw(const btVector3& clr, bool dynamic) {
	for (int i = 0; i < nEdges; i++) {
		m_edges[i]->debugDraw(clr, dynamic);
	}
	for (int i = 0; i < nAnchors; i++) {
		m_anchors[i]->debugDraw(clr, !dynamic);
	}
	for (int i = 0; i < nBendings; i++) {
		m_bendings[i]->debugDraw(clr, dynamic);
	}
}