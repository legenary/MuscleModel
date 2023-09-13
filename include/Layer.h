#ifndef LAYER_H
#define LAYER_H

#include "Utility.h"

class Tissue;
class Simulation;
class Parameter;
class MystacialPad;

struct DihedralPair {
	DihedralPair(const btVector3* p1, const btVector3* p2, const btVector3* p3, const btVector3* p4,
		const btVector3* v1, const btVector3* v2, const btVector3* v3, const btVector3* v4, bool isTop)
		: p1(p1), p2(p2), p3(p3), p4(p4), v1(v1), v2(v2), v3(v3), v4(v4), isTop(isTop) {
		calculate(true);
	}
	
	bool isTop;
	const btVector3* p1, *p2, *p3, *p4;
	const btVector3* v1, *v2, *v3, *v4;

	btScalar k = 1; //scaling factor
	btScalar angle0 = PI, angle;
	btVector3 u1, u2, u3, u4;
	btScalar f;
	btScalar work_acc = 0;

	/* Update u1, u2, u3, and u4. Calulate angle0 at initialization */
	void calculate(bool init = false);

	/* apply force onto tops of 4 follicles */
	void applyForce(std::vector<btRigidBody*> bodies);
};

class Layer {
private:
	Simulation* m_sim;
	Parameter* m_param;
	MystacialPad* m_pad;

	btDynamicsWorld* getWorld();

protected:
	std::vector<std::unique_ptr<Tissue>> m_edges;
	std::vector<std::unique_ptr<Tissue>> m_bendings;
	std::vector<std::unique_ptr<Tissue>> m_anchors;
	std::vector< std::unique_ptr<DihedralPair>> m_dihedral_pairs;
	int nDihedralPairs;

	int nEdges, nBendings, nAnchors;
	int nTissues;
	btScalar m_Hamiltonian;

public:
	Layer(Simulation* sim, Parameter* param, MystacialPad* pad)
		: m_sim(sim), m_param(param), m_pad(pad), nDihedralPairs(0)
		, nEdges(0), nBendings(0), nAnchors(0), nTissues(0)
		, m_Hamiltonian(0) {}

	/* Initialize structural springs. */
	void initEdges(bool isTop);

	/* Initialize anchor springs. */
	void initAnchors(bool isTop);

	/* Initialize bending springs. (Only used in bending model). See setting in Parameter.cpp */
	void initBendings(bool isTop);

	/* Initialize the dihedral pairs. (Only used in dihedral angle model). See setting in Parameter.cpp */
	void initDihedralPairs(bool isTop);

	void preUpdate();
	void postUpdate();

	int getNumTissue() const { return nTissues; }

	void debugDraw(const btVector3& clr, bool dynamic);

	btScalar getHamiltonian() const { return m_Hamiltonian; }

};







#endif