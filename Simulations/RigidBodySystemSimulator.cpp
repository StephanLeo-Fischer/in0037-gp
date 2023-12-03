#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	// Data Attributes
	m_fCollisionFactor = 1;

	// UI Attributes
	m_externalForce = 0.0;
	m_mouse = { 0, 0 };
	m_trackmouse = { 0, 0 };
	m_oldtrackmouse = { 0, 0 };
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return
		"Demo1: One-step,"
		"Demo2: Single-body,"
		"Demo3: Collision,"
		"Demo4: Complex,";
}

// Called when we reset the scene, or change the test case:
void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case DEMO1_ONESTEP:
		break;

	case DEMO2_SINGLE_BODY:
		break;

	case DEMO3_COLLISION:
	case DEMO4_COMPLEX:
		TwAddVarRW(DUC->g_pTweakBar, "Collision factor", TW_TYPE_FLOAT, &m_fCollisionFactor, "min=0 max=1 step=0.01");
		break;

	default:
		break;
	}
}

// Called once, at the beginning of the program:
void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	// Draw all the rigidbodies:
	for (auto& r : m_vRigidbodies)
		r.draw(DUC);

	Sleep(1);
}

// Called when we reset the scene, or change the test case:
void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	Vec3 velocity;

	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case DEMO1_ONESTEP:
		cout << "Switch to Demo1: One-step !\n";
		setupDemo1();

		// Compute a time step of 2, using Euler method:
		m_vRigidbodies[0].timestepEuler(2);

		cout << "Result of the simulation: Euler, one step with h = 2" << endl;
		cout << "Linear velocity: " << getLinearVelocityOfRigidBody(0) << endl;
		cout << "Angular velocity: " << getAngularVelocityOfRigidBody(0) << endl;

		velocity = m_vRigidbodies[0].getVelocityOfPoint(Vec3(0.3, 0.5, 0.25));
		cout << "World space velocity of point (0.3, 0.5, 0.25): " << velocity << endl;
		break;

	case DEMO2_SINGLE_BODY:
		cout << "Switch to Demo2: Single body !\n";
		setupDemo1();
		break;

	case DEMO3_COLLISION:
		cout << "Switch to Demo3: Collision between two rigidbodies !";
		setupDemo2();
		break;

	case DEMO4_COMPLEX:
		cout << "Switch to Demo4: Complex !\n";
		setupComplex();
		break;

	default:
		cout << "Empty test !\n";
		break;
	}
}

// Called each time the mouse is moved while pressed:
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
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
		float inputScale = 1;
		inputWorld = inputWorld * inputScale;

		//addExternalForce(inputWorld);
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timestep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case DEMO1_ONESTEP:
		// This case requires only one update, and is thus handled by notifyCaseChanged()
		break;

	case DEMO2_SINGLE_BODY:
	case DEMO3_COLLISION:
	case DEMO4_COMPLEX:
		for (Rigidbody& r : m_vRigidbodies)
			r.timestepEuler(timestep);

		manageCollisions();

		break;


	default:
		break;
	}
}

// This function is called when the mouse is clicked, 
// and when the mouse is moved while clicked:
void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// This function is called when the mouse moves, but not clicked:
void RigidBodySystemSimulator::onMouse(int x, int y)
{
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
	return m_vRigidbodies.at(i).getPosition();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_vRigidbodies.at(i).getLinearVelocity();
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_vRigidbodies.at(i).getAngularVelocity();
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_vRigidbodies.at(i).applyTorque(loc, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	m_vRigidbodies.push_back(Rigidbody(mass, position, Vec3(0.0), size));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_vRigidbodies.at(i).setRotation(orientation);
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_vRigidbodies.at(i).setLinearVelocity(velocity);
}

void RigidBodySystemSimulator::setupDemo1()
{
	m_vRigidbodies.clear();

	float mass = 2;
	Vec3 position = Vec3(0, 0, 0);
	Vec3 rotation = Vec3(0, 0, 90);
	Vec3 scale = Vec3(1, 0.6, 0.5);

	Rigidbody r = Rigidbody(mass, position, rotation, scale);
	r.applyTorque(Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
	m_vRigidbodies.push_back(r);
}

void RigidBodySystemSimulator::setupDemo2()
{
	m_vRigidbodies.clear();

	Rigidbody ground = Rigidbody(1000, Vec3(0, -1, 0), Vec3(0, 0, 0), Vec3(10, 0.1, 10));
	
	Rigidbody body = Rigidbody(1, Vec3(0, 0, 0), Vec3(0, 0, 20), Vec3(2, 0.1, 0.01));
	body.applyForce(Vec3(0, -5, 0));

	m_vRigidbodies.push_back(ground);
	m_vRigidbodies.push_back(body);
}

void RigidBodySystemSimulator::setupComplex()
{
	m_vRigidbodies.clear();

	Rigidbody ground = Rigidbody(1000, Vec3(0, -1, 0), Vec3(0, 0, 0), Vec3(10, 0.1, 10));
	ground.setKinematic(true);

	Rigidbody box1 = Rigidbody(1, Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0.6));
	Rigidbody box2 = Rigidbody(1, Vec3(0, 1, 0), Vec3(0, 0, 0), Vec3(0.4));
	Rigidbody box3 = Rigidbody(1, Vec3(0, 2, 0), Vec3(0, 0, 0), Vec3(0.2));

	box1.applyForce(Vec3(0, -2, 0));
	box2.applyForce(Vec3(0, -2, 0));
	box3.applyForce(Vec3(0, -2, 0));

	m_vRigidbodies.push_back(ground);
	m_vRigidbodies.push_back(box1);
	m_vRigidbodies.push_back(box2);
	m_vRigidbodies.push_back(box3);
}

void RigidBodySystemSimulator::manageCollisions()
{
	for (int i = 0; i < m_vRigidbodies.size(); i++) {
		for (int j = i+1; j < m_vRigidbodies.size(); j++) {

			// Manage collision between rigidbodies i and j:
			m_vRigidbodies[i].manageCollision(&m_vRigidbodies[j], m_fCollisionFactor);
		}
	}
}
