#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

	void initTable1();
	void timestepEuler(float timestep);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;


	struct Rigidbox
	{
		Vec3 m_vPosition;
		Quat m_vOrientation;
		Vec3 m_vSize;
		Vec3 m_vVelocity; 
		Vec3 m_vAngularVelocity;
		Vec3 m_vForce;
		float m_fMass;

		Rigidbox(Vec3 position, Quat orientation, Vec3 size, Vec3 velocity, Vec3 angularVelocity, float mass) :
			m_vPosition(position),
			m_vOrientation(orientation),
			m_vSize(size),
			m_vVelocity(velocity),
			m_vAngularVelocity(angularVelocity),
			m_vForce(0.0),
			m_fMass(mass) {}

		std::string toString();
	};

	vector<Rigidbox> m_vRigidboxes;

};
#endif