#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include <map>

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change


class MassSpringSystemSimulator:public Simulator{
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
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);

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

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	// Additional Attributes
	struct Point {
		Vec3 m_vPosition;
		Vec3 m_vVelocity;
		Vec3 m_vForce;
		bool m_bFixed;
		float m_fMass;

		Point(Vec3 position, Vec3 velocity, bool isFixed, float mass) :
			m_vPosition(position),
			m_vVelocity(velocity),
			m_vForce(0.0),
			m_bFixed(isFixed),
			m_fMass(mass) {}

		std::string to_string();
	};

	struct Spring {
		int m_pPoint1;
		int m_pPoint2;
		float m_fInitialLength;
		float m_fStiffness;

		Spring(int point1, int point2, float initialLength, float stiffness) :
			m_pPoint1(point1),
			m_pPoint2(point2),
			m_fInitialLength(initialLength),
			m_fStiffness(stiffness) {}
	};

	vector<Spring> m_vSprings;
	vector<Point> m_vPoints;

	Vec3 size_of_ball = Vec3(0.01f);

	// If we want to apply external forces (like gravity) to all the balls, or not:
	bool m_bApplyExternalForces = true;
	
	void initTable1();
	void initDemo4();

	void resetForces();
	void calculateForces();
	void calculateDamping();
	void updateCurrentPositions(float timeStep);
	void updateCurrentVelocities(float timeStep);

	void timestep_euler(float timeStep);
	void timestep_midpoint(float timeStep);
	void timestep_leapfrog(float timeStep);
	void addBoundaries();
};
#endif
