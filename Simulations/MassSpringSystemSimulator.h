#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include <vector>

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

struct MassPoint { 
	Vec3 position, velocity;
	bool isFixed;
	Vec3 force = Vec3(0,0,0);
	MassPoint(Vec3 position, Vec3 velocity, bool isFixed) {
		this->position = position;
		this->velocity = velocity;
		this->isFixed = isFixed;
	}
};

struct Spring {
	int point1, point2;
	float length;
	Spring(int point1, int point2, float length) {
		this->point1 = point1;
		this->point2 = point2;
		this->length = length;
	}
};


class MassSpringSystemSimulator :public Simulator {
public:
	// Construtors
	MassSpringSystemSimulator();

	// UI Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y) {}
	void onMouse(int x, int y) {}

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	void applyExternalForce(vector<MassPoint> &points, Vec3 force);

	void addRandomPoint();
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

	int m_iIntegrationKind;

	float containment;
	float floor;

	Vec3 gravity = Vec3(0, -9.81, 0);

	std::vector<MassPoint> masspoints;
	std::vector<MassPoint> temppoints;
	std::vector<Spring> springs;

	void eulerStep(float timeStep);
	void midpointStep(float timeStep);
	void leapfrogStep(float timeStep);
	void containmentBreach();

	void inittemp(vector<MassPoint> &main, vector<MassPoint> &temp);

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	bool firstTime;
};
#endif