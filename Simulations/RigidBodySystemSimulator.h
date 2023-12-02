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
		Quat m_qOrientation;
		Vec3 m_vSize;
		Vec3 m_vVelocity; 
		Vec3 m_vAngularVelocity;
		Vec3 m_vForce;
		float m_fMass;
		Vec3 m_vAcceleration;
		Vec3 m_vMomentum;
		Mat4 m_mInitialInersiaTensor;
		Mat4 m_mInersiaTensor;
		Mat4 m_mRotation;
		Vec3 m_vTorque;
		Mat4 m_mObjToWorld;
		Mat4 m_mWorldToObj;
		Mat4 m_mScaledObjToWorld;
		Mat4 m_mWorldToScaledObj;

		Rigidbox(Vec3 position, Vec3 size, float mass) :
			m_vPosition(position),
			m_vSize(size),
			m_fMass(mass) 
		{
			float c11 = 8 * mass * size.x * size.x;
			float c22 = 8 * mass * size.y * size.y;
			float c33 = 8 * mass * size.z * size.z;
			Mat4 matC = Mat4(c11, 0, 0, 0,   0, c22, 0, 0,    0, 0, c33, 0,    0, 0, 0, 0);
			float traceC = (c11 + c22 + c33);
			Mat4 inertiaI0 = Mat4(traceC, 0, 0, 0,    0, traceC, 0, 0,    0, 0, traceC, 0,    0, 0, 0, 0) - matC;
			m_mInitialInersiaTensor = inertiaI0;

			// von stephan kopiert
			Mat4 matrix;
			matrix.initTranslation(m_vPosition.x, m_vPosition.y, m_vPosition.z);
			m_mScaledObjToWorld = m_qOrientation.getRotMat() * matrix;
			m_mWorldToScaledObj = m_mScaledObjToWorld.inverse();
			matrix.initScaling(m_vSize.x, m_vSize.y, m_vSize.z);
			m_mObjToWorld = matrix * m_mScaledObjToWorld;
			m_mWorldToObj = m_mObjToWorld.inverse();
		}

		std::string toString();
	};

	vector<Rigidbox> m_vRigidboxes;

};
#endif