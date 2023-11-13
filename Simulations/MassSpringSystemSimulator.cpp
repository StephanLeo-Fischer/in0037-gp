#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	// Data Attributes
	m_fMass = 1.0;
	m_fStiffness = 1.0;
	m_fDamping = 1.0;
	m_iIntegrator = 1;

	// UI Attributes
	m_externalForce = 0.0;
	m_mouse = { 0, 0 };
	m_trackmouse = { 0, 0 };
	m_oldtrackmouse = { 0, 0 };

	m_vExternalForce = Vec3(0, 0, 0);
	m_fPointsDensity = 1000;
	m_iTimestepMethod = 0;
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return 
		"Demo1: one-step,"
		"Demo2: Euler,"
		"Demo3: Midpoint,"
		"Demo4: Complex,"
		"Demo5: Leapfrog";
}

// Called when we reset the scene, or change the test case:
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case DEMO1_ONESTEP:
		break;

	case DEMO2_EULER:
		break;

	case DEMO3_MIDPOINT:
		break;

	case DEMO4_COMPLEX:
		// Allow the user to select between Euler and Midpoint methods:
		TwType TW_TYPE_METHOD;
		TW_TYPE_METHOD = TwDefineEnumFromString("Method", "Euler,Midpoint");
		TwAddVarRW(DUC->g_pTweakBar, "Method", TW_TYPE_METHOD, &m_iTimestepMethod, "");
		break;

	case DEMO5_LEAPFROG:
		break;

	default:
		break;
	}
}

// Called once, at the beginning of the program:
void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	const Vec3 yellow = Vec3(1, 1, 0);

	// Draw all the mass points:
	for (auto& p : m_vPoints) {
		// density = mass / (4/3 * PI * R^3)
		// <=> R^3 = mass / (4/3 * PI * density)
		DUC->drawSphere(p.m_vPosition, pow(p.m_fMass / m_fPointsDensity, 1.0/3));
	}

	// Draw all the springs:
	for (auto& s : m_vSprings) {
		DUC->beginLine();
		DUC->drawLine(getPositionOfMassPoint(s.point1), yellow,
					  getPositionOfMassPoint(s.point2), yellow);
		DUC->endLine();
	}
}

// Called when we reset the scene, or change the test case:
void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case DEMO1_ONESTEP:
		cout << "Switch to Demo1: One-step !\n";

		setupDemo1();
		printPoints("Initial status of the points:");

		timestepEuler(0.1f);
		printPoints("Status of the points after one step of Euler method:");

		setupDemo1();
		timestepMidpoint(0.1f);
		printPoints("Status of the points after one step of Midpoint method:");
		break;

	case DEMO2_EULER:
		cout << "Switch to Demo2: Euler !\n";
		setupDemo1();
		break;

	case DEMO3_MIDPOINT:
		cout << "Switch to Demo3: Midpoint !";
		setupDemo1();
		break;

	case DEMO4_COMPLEX:
		cout << "Switch to Demo4: Complex !\n";
		setupComplex2();
		break;

	case DEMO5_LEAPFROG:
		cout << "Switch to Demo5: Leapfrog !\n";
		setupComplex();
		break;

	default:
		cout << "Empty test !\n";
		break;
	}
}

// Called each time the mouse is moved while pressed:
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;

	if ((mouseDiff.x != 0 || mouseDiff.y != 0) && (m_iTestCase == DEMO4_COMPLEX || m_iIntegrator == DEMO5_LEAPFROG))
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);

		// find a proper scale!
		float inputScale = 1;
		inputWorld = inputWorld * inputScale;

		//addExternalForce(inputWorld);
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timestep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case DEMO1_ONESTEP:	
		// This case requires only one update, and is thus handled by notifyCaseChanged()
		break;

	case DEMO2_EULER:
		timestepEuler(timestep);
		break;

	case DEMO3_MIDPOINT:
		timestepMidpoint(timestep);
		break;

	case DEMO4_COMPLEX:
		if (m_iTimestepMethod == 0)
			timestepEuler(timestep);
		else if (m_iTimestepMethod == 1)
			timestepMidpoint(timestep);
		else
			cerr << "Error: Impossible number for the timestep method" << endl;

		break;

	case DEMO5_LEAPFROG:
		timestepLeapfrog(timestep);
		break;

	default:
		break;
	}
}

// Based on the position of the points, compute the forces applied on them:
void MassSpringSystemSimulator::computeForces()
{
	// The force applied to each mass point is initialised to the externalForce:
	for (auto& p : m_vPoints) {
		p.m_vForce = m_vExternalForce;
	}

	// Add the force of the springs to the mass points:
	for (auto& s : m_vSprings) {
		Point& p1 = m_vPoints.at(s.point1);
		Point& p2 = m_vPoints.at(s.point2);

		// Vector from p1 to p2:
		Vec3 v = p2.m_vPosition - p1.m_vPosition;
		float distance = norm(v);

		// Compute the force applied by the spring to p1:
		Vec3 force = m_fStiffness * (distance - s.m_fInitialLength) * v / distance;

		p1.m_vForce += force;
		p2.m_vForce -= force;
	}

	// Compute the damping for the mass points:
	for (auto& p : m_vPoints) {
		if (!p.m_bFixed)
			p.m_vForce -= p.m_vVelocity * m_fDamping;
	}
}

// Update the position and velocity of the given points, depending on the force applied on them,
// and save the result in result 
void MassSpringSystemSimulator::updatePoints(float timestep)
{
	for (auto& p : m_vPoints) {
		if (p.m_bFixed) {
			p.m_vVelocity = 0;
		}
		else {
			p.m_vPosition += timestep * p.m_vVelocity;
			p.m_vVelocity += timestep * p.m_vForce / p.m_fMass;

			// Clamp position to simulate the ground:
			if (p.m_vPosition.y < 0)
				p.m_vPosition.y = 0;
		}
	}
}

void MassSpringSystemSimulator::timestepEuler(float timestep)
{
	// Compute the force applied to each mass point:
	computeForces();

	// Update the points speed and position:
	updatePoints(timestep);
}

void MassSpringSystemSimulator::timestepMidpoint(float timestep)
{
	// Save the current position and speed of each point:
	vector<Vec3> positions;
	vector<Vec3> velocities;

	for (auto& p : m_vPoints) {
		positions.push_back(p.m_vPosition);
		velocities.push_back(p.m_vVelocity);
	}

	// Compute the force applied to each mass point:
	computeForces();

	// First update with a half step:
	updatePoints(timestep / 2);

	// Compute again the forces applied to the mass points:
	computeForces();

	// Reset the position and speed of the points:
	for (int i = 0; i < m_vPoints.size(); i++) {
		m_vPoints[i].m_vPosition = positions[i];
		m_vPoints[i].m_vVelocity = velocities[i];
	}

	// Finally update the points with a full step:
	updatePoints(timestep);
}

void MassSpringSystemSimulator::timestepLeapfrog(float timestep)
{

}

void MassSpringSystemSimulator::setupDemo1()
{
	m_vPoints.clear();
	m_vSprings.clear();

	setMass(10);
	setStiffness(40);
	setDampingFactor(0);
	applyExternalForce(Vec3(0, 0, 0));

	addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
	addSpring(0, 1, 1);
}

void MassSpringSystemSimulator::setupComplex()
{
	m_vPoints.clear();
	m_vSprings.clear();

	setMass(10);
	setStiffness(100);
	setDampingFactor(1);
	applyExternalForce(Vec3(0, 0, -1));

	// Create a cylinder:
	const Vec3 startPosition = Vec3(0, 10, 0);
	const Vec3 startSpeed = Vec3(0, -2, 0);
	const int rounds = 10;
	const float radius = 2;
	const float height = 1;

	// Create the mass points:
	addMassPoint(startPosition + Vec3(-height / 2, 0, 0), startSpeed, false);
	addMassPoint(startPosition + Vec3(+height / 2, 0, 0), startSpeed, false);

	for (int i = 0; i < rounds; i++) {
		double angle = 2 * i * M_PI / rounds;
		addMassPoint(
			startPosition + Vec3(-height / 2, radius * cos(angle), radius * sin(angle)), 
			startSpeed, false);

		addMassPoint(
			startPosition + Vec3(+height / 2, radius * cos(angle), radius * sin(angle)),
			startSpeed, false);
	}

	// Springs between the points:
	addSpring(0, 1);
	for (int i = 2; i <= 2*rounds; i+=2) {
		addSpring(i, i + 1);
		addSpring(0, i);
		addSpring(1, i + 1);		

		if (i < 2 * rounds) {
			addSpring(i, i + 2);
			addSpring(i + 1, i + 3);
		}
		else {
			addSpring(i, 2);
			addSpring(i + 1, 3);
		}
	}
}

void MassSpringSystemSimulator::setupComplex2()
{
	m_vPoints.clear();
	m_vSprings.clear();

	setMass(1);
	setStiffness(100);
	setDampingFactor(0.01f);
	applyExternalForce(Vec3(0, -2, 0));		// Add gravity

	const int nPoints = 5;
	const float startAngle = 45 * M_PI / 180;	// Start angle in radians
	const float L = 1;							// Distance between points
	const Vec3 centerPosition = Vec3(0, L * nPoints + 0.5f, 0);

	// Create a pendulum:
	addMassPoint(centerPosition, Vec3(0, 0, 0), true);
	for (int i = 1; i <= nPoints; i++) {
		Vec3 deltaPosition = Vec3(i * L * sin(startAngle), -i * L * cos(startAngle), 0);
		addMassPoint(centerPosition + deltaPosition, Vec3(0, 0, 0), false);
		addSpring(i - 1, i);
	}
}

void MassSpringSystemSimulator::printPoints(string message)
{
	cout << message << endl;
	for (int i = 0; i < m_vPoints.size(); i++) {
		auto& p = m_vPoints[i];

		cout << "Point " << i << ": {";
		cout << "m" << i << " = " << p.m_fMass << "; ";
		cout << "p" << i << " = " << p.m_vPosition << "; ";

		if (p.m_bFixed)
			cout << "fixed}\n";
		else
			cout << "v" << i << " = " << p.m_vVelocity << "}\n";
	}
}

// This function is called when the mouse is clicked, 
// and when the mouse is moved while clicked:
void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// This function is called when the mouse moves, but not clicked:
void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// Set the mass of the next mass points we will add using addMassPoint()
void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

// Set the stiffness of the next springs we will ass using addSpring()
void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

// Set the damping factor of all the mass points:
void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

// Create a new mass point, add it to the list of points, and return the index of the point:
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed)
{
	m_vPoints.push_back(Point(position, velocity, isFixed, m_fMass));
	return m_vPoints.size() - 1;
}

// Create a new spring between two points, with a length equal to the distance between the points:
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2)
{
	float initialLength = norm(getPositionOfMassPoint(masspoint1) - getPositionOfMassPoint(masspoint2));
	m_vSprings.push_back(Spring(masspoint1, masspoint2, initialLength, m_fStiffness));
}

// Create a new spring between two points, with the given inital length:
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	m_vSprings.push_back(Spring(masspoint1, masspoint2, initialLength, m_fStiffness));
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_vPoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_vSprings.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return m_vPoints.at(index).m_vPosition;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return m_vPoints.at(index).m_vVelocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_vExternalForce = force;
}

void MassSpringSystemSimulator::addExternalForce(Vec3 force)
{
	m_vExternalForce += force;
}
