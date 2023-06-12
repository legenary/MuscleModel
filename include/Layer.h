#ifndef LAYER_H
#define LAYER_H


class Tissue;
class Simulation;
class Parameter;
class MystacialPad;

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

	int nEdges, nBendings, nAnchors;
	int nTissues;

public:
	Layer(Simulation* sim, Parameter* param, MystacialPad* pad)
		: m_sim(sim), m_param(param), m_pad(pad)
		, nEdges(0), nBendings(0), nAnchors(0), nTissues(0) {}

	void initEdges(bool isTop);
	void initAnchors(bool isTop);
	void initBendings(bool isTop);

	void update();

	int getNumTissue() const { return nTissues; }

	void debugDraw(const btVector3& clr, bool dynamic);

};







#endif