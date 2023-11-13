#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

#define DEMO1_ONESTEP 0
#define DEMO2_EULER 1
#define DEMO3_MIDPOINT 2
#define DEMO4_COMPLEX 3
#define DEMO5_LEAPFROG 4

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
	int addMassPoint(Vec3 position, Vec3 velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	void addExternalForce(Vec3 force);

	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fMass;			// Mass of the next mass points we will add using addMassPoint()
	float m_fStiffness;		// Stiffness of the next springs we will add using addSpring()
	float m_fDamping;		// Damping applied to all the mass points
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
		float m_fMass;

		Point(Vec3 position, Vec3 velocity, bool isFixed, float mass) :
			m_vPosition(position),
			m_vVelocity(velocity),
			m_vForce(0.0),
			m_bFixed(isFixed),
			m_fMass(mass) {}
	};

	struct Spring {
		int point1;
		int point2;
		float m_fInitialLength;
		float m_fStiffness;

		Spring(int point1, int point2, float initialLength, float stiffness) :
			point1(point1),
			point2(point2),
			m_fInitialLength(initialLength),
			m_fStiffness(stiffness) {}
	};

	vector<Spring> m_vSprings;
	vector<Point> m_vPoints;
	
	Vec3 m_vExternalForce;		// Sum of the forces applied to all the mass points
	float m_fPointsDensity;		// Used to compute the volume of the mass points (for UI)
	int m_iTimestepMethod;		// Used to determine the method we are using for the timestep

	// Intermediate computations for euler and midpoint:
	void computeForces();				// Compute the forces applied by the springs on the points
	void updatePoints(float timestep);	// Update the points position and speed, depending on the force applied on them

	// Different methods to update the points of the simulation:
	void timestepEuler(float timestep);
	void timestepMidpoint(float timestep);
	void timestepLeapfrog(float timestep);

	// Setup different demos:
	void setupDemo1();
	void setupComplex();
	void setupComplex2();


	// Printing:
	void printPoints(string message);
};
#endif