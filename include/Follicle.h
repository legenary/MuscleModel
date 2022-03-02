#ifndef FOLLICLE_H
#define FOLLICLE_H


class Follicle {
private:
	//btVector3 m_position;
	//btVector3 m_orientation;
	btScalar m_mass;
	btScalar m_length;
	
	btRigidBody* m_body;
	btCollisionShape* m_shape;

public:
	Follicle(btDiscreteDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* shapes, 
		     btTransform trans, btScalar radius, btScalar half_height, btScalar mass);
	Follicle(const Follicle&) = delete;
	Follicle& operator=(Follicle const&) = delete;
	virtual ~Follicle();

	btRigidBody* getBody() const;
	btScalar getLength() const;
	btScalar getMass() const;
	



};








#endif