#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	// Data Attributes
	m_fMass = 1.0;
	m_fStiffness = 1.0;
	m_fDamping = 1.0;
	m_iIntegrator = 1;

	// UI Attributes
	m_externalForce;
	m_mouse;
	m_trackmouse;
	m_oldtrackmouse;
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Euler,Leapfrog,Mitpoint";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case EULER:break;
	case LEAPFROG:break;
	case MIDPOINT:break;
	default:break;
	}
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case EULER: break;
	case LEAPFROG: break;
	case MIDPOINT: break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case EULER:
		cout << "Test Case Euler!\n";
		break;

	case LEAPFROG:
		cout << "Test Case Leapfrog!\n";
		break;

	case MIDPOINT:
		cout << "Test Case Midpoint!\n";
		break;

	default:
		cout << "Empty Test!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		//m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		//m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timestep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{// handling different cases
	case EULER:
		timestepEuler(timestep);
		break;

	case LEAPFROG:
		break;

	case MIDPOINT:
		timestepMidpoint(timestep);
		break;

	default:
		break;
	}
}

void MassSpringSystemSimulator::timestepEuler(float timestep)
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

	// Update the points speed and position:
	for (auto& p : m_vPoints) {
		if (p.m_bFixed) {
			p.m_vVelocity = 0;
		}
		else {
			p.m_vForce -= p.m_vVelocity * m_fDamping;
			p.m_vPosition += timestep * p.m_vVelocity;
			p.m_vVelocity += timestep * p.m_vForce / m_fMass;
		}
	}
}

void MassSpringSystemSimulator::timestepMidpoint(float timestep)
{

}

void MassSpringSystemSimulator::onClick(int x, int y)
{
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
}

void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

// Create a new mass point, add it to the list of points, and return the index of the point:
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed)
{
	m_vPoints.push_back(Point(position, velocity, isFixed));
	return m_vPoints.size() - 1;
}

// Create a new spring between two points, with the given inital length:
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	m_vSprings.push_back(Spring(masspoint1, masspoint1, initialLength));
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
	m_vExternalForce += force;
}
