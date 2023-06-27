#include "my_pch.h"
#include "Layer.h"

#include "Parameter.h"
#include "Tissue.h"
#include "Follicle.h"
#include "Utility.h"

void Layer::initEdges(bool isTop) {

	nEdges = m_param->SPRING_HEX_MESH_IDX.size();
	m_edges.reserve(nEdges);
	for (int s = 0; s < nEdges; s++) {
		auto& fol1 = m_pad->getFollicleByIndex(m_param->SPRING_HEX_MESH_IDX[s][0]);
		auto& fol2 = m_pad->getFollicleByIndex(m_param->SPRING_HEX_MESH_IDX[s][1]);

		btTransform frameLayer1fol1 = createTransform(btVector3((isTop ? 1 : -1) * m_param->FOLLICLE_POS_ORIENT_LEN_VOL[m_param->SPRING_HEX_MESH_IDX[s][0]][6] / 2, 0., 0.));
		btTransform frameLayer1fol2 = createTransform(btVector3((isTop ? 1 : -1) * m_param->FOLLICLE_POS_ORIENT_LEN_VOL[m_param->SPRING_HEX_MESH_IDX[s][1]][6] / 2, 0., 0.));

		btScalar k_eq = m_param->k_layer1;
		btScalar k_this = k_eq / 2;
		std::unique_ptr<Tissue> edge = std::make_unique<Tissue>(m_sim, fol1->getBody(), fol2->getBody(), frameLayer1fol1, frameLayer1fol2, k_this, m_param->zeta_tissue);

		getWorld()->addConstraint(edge->getConstraint(), true); // disable collision
		m_edges.push_back(std::move(edge));
	}

	nTissues += nEdges;
}

void Layer::initAnchors(bool isTop) {
	nAnchors = m_pad->getNumFollicles();
	m_anchors.reserve(nAnchors);
	for (int f = 0; f < nAnchors; f++) {
		auto& fol = m_pad->getFollicleByIndex(f);
		btTransform frameAnchor = createTransform(btVector3((isTop ? 1 : -1) * m_param->FOLLICLE_POS_ORIENT_LEN_VOL[f][6] / 2, 0., 0.));
		std::unique_ptr<Tissue> anchor = std::make_unique <Tissue>(m_sim, fol->getBody(), frameAnchor, m_param->k_anchor, m_param->zeta_tissue);	// this is a linear + torsional spring

		getWorld()->addConstraint(anchor->getConstraint(), true); // disable collision
		m_anchors.push_back(std::move(anchor));
	}
}


void Layer::initBendings(bool isTop) {
	nBendings = m_param->SPRING_BENDING_IDX.size();
	m_bendings.reserve(nBendings);
	for (int s = 0; s < nBendings; s++) {
		auto& fol1 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[s][0]);
		auto& fol2 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[s][1]);
		btTransform frameLayer1fol1 = createTransform(btVector3((isTop ? 1 : -1) * m_param->FOLLICLE_POS_ORIENT_LEN_VOL[m_param->SPRING_BENDING_IDX[s][0]][6] / 2, 0., 0.));
		btTransform frameLayer1fol2 = createTransform(btVector3((isTop ? 1 : -1) * m_param->FOLLICLE_POS_ORIENT_LEN_VOL[m_param->SPRING_BENDING_IDX[s][1]][6] / 2, 0., 0.));

		btScalar k_eq = m_param->k_layer1;
		btScalar k_this = k_eq / 2;
		std::unique_ptr<Tissue> bending = std::make_unique <Tissue>(m_sim, fol1->getBody(), fol2->getBody(), frameLayer1fol1, frameLayer1fol2, k_this, m_param->zeta_tissue);

		getWorld()->addConstraint(bending->getConstraint(), true); // disable collision
		m_bendings.push_back(std::move(bending));
	}

	nTissues += nBendings;
}

void Layer::initDihedralPairs(bool isTop) {
	nDihedralPairs = m_param->SPRING_BENDING_IDX.size();
	m_dihedral_pairs.reserve(nDihedralPairs);
	for (int i = 0; i < nDihedralPairs; i++) {
		if (isTop) {
			btVector3& p1 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][0])->getTopLocation();
			btVector3& p2 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][1])->getTopLocation();
			btVector3& p3 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][2])->getTopLocation();
			btVector3& p4 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][3])->getTopLocation();
			btVector3& v1 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][0])->getTopVelocity();
			btVector3& v2 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][1])->getTopVelocity();
			btVector3& v3 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][2])->getTopVelocity();
			btVector3& v4 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][3])->getTopVelocity();
			m_dihedral_pairs.emplace_back(&p1, &p2, &p3, &p4, &v1, &v2, &v3, &v4, isTop);
		}
		else {
			btVector3& p1 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][0])->getBotLocation();
			btVector3& p2 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][1])->getBotLocation();
			btVector3& p3 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][2])->getBotLocation();
			btVector3& p4 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][3])->getBotLocation();
			btVector3& v1 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][0])->getBotVelocity();
			btVector3& v2 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][1])->getBotVelocity();
			btVector3& v3 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][2])->getBotVelocity();
			btVector3& v4 = m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][3])->getBotVelocity();
			m_dihedral_pairs.emplace_back(&p1, &p2, &p3, &p4, &v1, &v2, &v3, &v4, isTop);
		}
	}
}

void Layer::update() {
	m_Hamiltonian = 0;
	for (int i = 0; i < nEdges; i++) {
		m_edges[i]->update();
		m_Hamiltonian += m_edges[i]->getHamiltonian();
	}
	for (int i = 0; i < nAnchors; i++) {
		m_anchors[i]->update();
		m_Hamiltonian += m_anchors[i]->getHamiltonian();
	}
	for (int i = 0; i < nBendings; i++) {
		m_bendings[i]->update();
		m_Hamiltonian += m_bendings[i]->getHamiltonian();
	}
	for (int i = 0; i < nDihedralPairs; i++) {
		m_dihedral_pairs[i].calculate();
		m_dihedral_pairs[i].applyForce({
			m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][0])->getBody(),
			m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][1])->getBody(),
			m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][2])->getBody(),
			m_pad->getFollicleByIndex(m_param->SPRING_BENDING_IDX[i][3])->getBody()
		});
		m_Hamiltonian += m_dihedral_pairs[i].work_acc;
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

void DihedralPair::calculate(bool init) {
	// calculate e, n1, n2, u1, u2, u3, u4
	// Bridson et al 2005
	btVector3 p41(*p1-*p4), p42(*p2-*p4), p31(*p1-*p3), p32(*p2-*p3);
	btVector3 n1 = (p31.cross(p41));	
	btVector3 n2 = (p42.cross(p32));	
	btVector3 e(*p4 - *p3);

	btScalar n1_norm_inv = 1.f / n1.norm();
	btScalar n2_norm_inv = 1.f / n2.norm();
	btVector3 n1_unit = n1.normalized();
	btVector3 n2_unit = n2.normalized();
	btVector3 e_unit = e.normalized();

	u1 = e.norm() * n1_norm_inv * n1_unit;
	u2 = e.norm() * n2_norm_inv * n2_unit;
	u3 = p41.dot(e_unit) * n1_norm_inv * n1_unit + p42.dot(e_unit) * n2_norm_inv * n2_unit;
	u4 = -p31.dot(e_unit) * n1_norm_inv * n1_unit - p32.dot(e_unit) * n2_norm_inv * n2_unit;

	angle = n1.angle(n2);	// radians
	if (init) {
		angle0 = n1.angle(n2);
		work_acc = 0;
	}

	f = k * e.norm() * e.norm() / (n1.norm() + n2.norm())
		* (btSin((PI - angle) * 0.5) - btSin((PI - angle0) * 0.5));

	work_acc += f * (u1.dot(*v1) + u2.dot(*v2) + u3.dot(*v3) + u4.dot(*v4));
}

void DihedralPair::applyForce(std::vector<btRigidBody*> bodies) {
	ensure(bodies.size() == 4);
	bodies[0]->applyForce(f * u1, *p1 - bodies[0]->getCenterOfMassTransform().getOrigin());
	bodies[1]->applyForce(f * u2, *p2 - bodies[1]->getCenterOfMassTransform().getOrigin());
	bodies[2]->applyForce(f * u3, *p3 - bodies[2]->getCenterOfMassTransform().getOrigin());
	bodies[3]->applyForce(f * u4, *p4 - bodies[3]->getCenterOfMassTransform().getOrigin());
}