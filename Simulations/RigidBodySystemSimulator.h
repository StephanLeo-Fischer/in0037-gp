#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"

//add your header for your rigid body system, for e.g.,
#include "Rigidbody.h"
#include "SpringStructure.h"
#include "collisionDetect.h"
#include "CollisionDebugger.h"

#define TESTCASEUSEDTORUNTEST 2

#define TEST_DEMO 0
#define ANGRY_BIRDS_DEMO 1
#define SPRINGS_DEMO 2
#define COLLISIONS_DEMO 3


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
	// UI Attributes
	Point2D m_prevmouse;		// Previous mouse position
	boolean m_bMousePressed;

	// Vector of all the rigidbodies in the game, that are stored on the heap ! We need to store these
	// in the heap, because we want the pointers in the spring structures to point to the same objects,
	// even if we add or remove some elements in this vector:
	vector<Rigidbody*> m_vRigidbodies;
	vector<SpringStructure> m_vSpringStructures;

	// Set of parameters used for the simulation, that can be changed in the UI:
	SimulationParameters m_SimulationParameters;
	
	int m_iDebugLine = 0;
	bool m_bEnablePositionCorrection = true;
	float m_fGravity = 9.81f;
	int m_iTestScenario = 0;

	// Used to generate random colors:
	std::mt19937 eng;
	std::uniform_real_distribution<float> randFloat;

	// TODO: Delete this (used for debug):
	CollisionDebugger collisionDebugger;

	// Define a structure that encapsulates a collision info, as well as the indices 
	// of the rigidbodies that are colliding
	struct Collision {
		int i1, i2;				// The indices of the two rigidbodies that are colliding
		Vec3 collisionPoint;	// The position of the collision point in world space
		Vec3 collisionNormal;	// The direction of the impulse from j to i
		float collisionDepth;   // The distance of the collision point to the surface

		Collision(int i1, int i2, CollisionInfo info) :
			i1(i1), i2(i2),
			collisionPoint(info.collisionPointWorld),
			collisionNormal(info.normalWorld),
			collisionDepth(info.depth) {}
	};

	void setupTestDemo();
	void setupAngryBirdsDemo();
	void setupSpringsDemo();
	
	void manageCollisions(double timestep);
	void manageCollision(Rigidbody* r1, Rigidbody* r2, const Collision* collision, double timestep);
	void fireRigidbody();
};
#endif