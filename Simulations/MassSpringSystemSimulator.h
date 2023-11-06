#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change


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
	void simulateTimestep(float timestep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

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

	struct Point {
		Vec3 m_vPosition;
		Vec3 m_vVelocity;
		Vec3 m_vForce;
		bool m_bFixed;

		Point(Vec3 position, Vec3 velocity, bool isFixed) :
			m_vPosition(position),
			m_vVelocity(velocity),
			m_vForce(0.0),
			m_bFixed(isFixed) {}
	};

	struct Spring {
		int point1;
		int point2;
		float m_fInitialLength;

		Spring(int point1, int point2, float initialLength) :
			point1(point1),
			point2(point2),
			m_fInitialLength(initialLength) {}
	};

	vector<Spring> m_vSprings;
	vector<Point> m_vPoints;

	// Sum of the forces applied to all the mass points:
	Vec3 m_vExternalForce;

	void timestepEuler(float timestep);
	void timestepMidpoint(float timestep);
};
#endif