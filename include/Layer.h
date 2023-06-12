#ifndef LAYER_H
#define LAYER_H

#define PI 3.1415926

class Tissue;
class Simulation;
class Parameter;
class MystacialPad;

struct DihedralPair {
	DihedralPair(btVector3* p1, btVector3* p2, btVector3* p3, btVector3* p4, bool isTop)
		: p1(p1), p2(p2), p3(p3), p4(p4), isTop(isTop) {
		calculate(true);
	}
	
	bool isTop;
	btVector3* p1, *p2, *p3, *p4;

	btScalar k = 0; //scaling factor
	btScalar angle0 = PI, angle;
	btVector3 u1, u2, u3, u4;
	btScalar f;

	void calculate(bool init = false);
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
	std::vector<DihedralPair> m_dihedral_pairs;
	int nDihedralPairs;

	int nEdges, nBendings, nAnchors;
	int nTissues;

public:
	Layer(Simulation* sim, Parameter* param, MystacialPad* pad)
		: m_sim(sim), m_param(param), m_pad(pad), nDihedralPairs(0)
		, nEdges(0), nBendings(0), nAnchors(0), nTissues(0) {}

	void initEdges(bool isTop);
	void initAnchors(bool isTop);
	void initBendings(bool isTop);
	void initDihedralPairs(bool isTop);

	void update();

	int getNumTissue() const { return nTissues; }

	void debugDraw(const btVector3& clr, bool dynamic);

};







#endif