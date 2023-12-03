#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"

//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 
#include "Rigidbody.h"

#define TESTCASEUSEDTORUNTEST 2

#define DEMO1_ONESTEP 0
#define DEMO2_SINGLE_BODY 1
#define DEMO3_COLLISION 2
#define DEMO4_COMPLEX 3

#define GRAVITY_FACTOR 6


class RigidBodySystemSimulator:public Simulator {
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
	void simulateTimestep(float timestep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Other functions for the mouse:
	void mousePressed(int x, int y);	// Called the first frame where the mouse is pressed
	void mouseReleased(int x, int y);	// Called the first frame where the mouse is released
	void mouseDragged(int x, int y);	// Called when the mouse is moved while pressed
	void mouseMoved(int x, int y);		// Called when the mouse is moved while released

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

private:
	// Attributes
	Vec3 m_externalForce;

	// UI Attributes
	Point2D m_prevmouse;	// Previous mouse position
	boolean m_bMousePressed;

	vector<Rigidbody> m_vRigidbodies;

	float m_fCollisionFactor;

	void setupDemo1();
	void setupDemo2();
	void setupComplex();
	void manageCollisions();
};
#endif