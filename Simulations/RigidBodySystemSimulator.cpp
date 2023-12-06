#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	//RigidBodySystem * m_pRigidBodySystem;  // TODO
	m_externalForce = Vec3(0, 0, 0);

	// UI Attributes
	m_mouse = { 0, 0 };
	m_trackmouse = { 0, 0 };
	m_oldtrackmouse = { 0, 0 };
}

// This function is called when the mouse is clicked, 
// and when the mouse is moved while clicked:
void RigidBodySystemSimulator::onClick(int x, int y) {
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// This function is called when the mouse moves, but not clicked:
void RigidBodySystemSimulator::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_vRigidbodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return m_vRigidbodies.at(i).m_vPosition;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_vRigidbodies.at(i).m_vLinearVelocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_vRigidbodies.at(i).m_vAngularVelocity;
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	Rigidbody rb = Rigidbody(mass, position, Quat(), size);
	m_vRigidbodies.push_back(rb);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_vRigidbodies.at(i).m_qOrientation = orientation;
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return
		"Demo 1: one-step,"
		"Demo 2: Single body simulation,"
		"Demo 3: 2 RB collision,"
		"Demo 4: complex simulation,";
}

// Called when we reset the scene, or change the test case:
void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler,Midpoint,LeapFrog");
	switch (m_iTestCase)
	{
	case 0:break;
	case 1:break;
	case 2:break;
	case 3:break;
	case 4:break;
	default:break;
	}
}

// Called once, at the beginning of the program:
void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for (auto& rb : m_vRigidbodies) 
	{
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		DUC->drawRigidBody(rb.m_mTransformMatrix);
	}

}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0: // single timestep
		initTable1();
		cout << "Demo 1, a simple one-step test\n";
		for (auto& rb : m_vRigidbodies) {
			cout << rb.toString();
		}
		cout << "timestepEuler(2)\n";
		demo1TimeStep(2);  // 1 step
		for (auto& rb : m_vRigidbodies) {
			cout << rb.toString();
		}
		break;

	case 1: // single body simulation
		cout << "Demo 2: a simple Euler simulation - HINT: changing the timestep has no effect on the simulation\n";
		break;
	case 2: // 2 RB collision
		cout << "Demo 3: a simple Midpoint simulation - HINT: changing the timestep has no effect on the simulation\n";
		break;
	case 3: // complex simulation
		//initDemo4();
		cout << "Demo 4: a complex simulation, compare the stability of Euler and Midpoint method\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 0: // single timestep
		// This case requires only one update, and is thus handled by notifyCaseChanged()
		break;

	case 1: // single body simulation
		break;
	case 2: // 2 RB collision
		break;
	case 3: // complex simulation
		break;
	default:
		break;
	}
}

void RigidBodySystemSimulator::initTable1() 
{
	m_vRigidbodies.clear();

	Vec3 pos = Vec3(0, 0, 0);
	Vec3 size = Vec3(1, 0.6, 0.5);

	addRigidBody(pos, size, 2);
	Rigidbody& rb = m_vRigidbodies.front();
	rb.m_mRotMat.initRotationZ(90);
	rb.m_qOrientation = Quat(rb.m_mRotMat);
	rb.addForce(Vec3(1, 1, 0), Vec3(0.3, 0.5, 0.25));
}

void RigidBodySystemSimulator::demo1TimeStep(float timeStep) {
	// single time step impl
	for (Rigidbody rb : m_vRigidbodies) 
	{
		rb.timestepEuler(timeStep);
	}
}

