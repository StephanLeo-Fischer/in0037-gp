#include "OpenSimulation.h"

#include "MassSpringSystemSimulator.h"
#include "RigidBodySystemSimulator.h"
#include "SPHSystemSimulator.h"

extern Simulator* g_pSimulator;
extern float g_fTimestep;

#define DEMO1_TRAMPOLIN1 0
#define DEMO2_TRAMPOLIN2 1
#define DEMO3_TRAMPOLIN3 2
#define DEMO4_TRAMPOLIN4 3

OpenSimulation::OpenSimulation() {
	// UI Attributes
	m_prevmouse = { 0, 0 };
	m_bMousePressed = false;

	// randCol will be a random value between 0.0 and 1.0:
	randCol = std::uniform_real_distribution<float>(0.0f, 1.0f);

	// Data Attributes
	m_SimulationParameters = {};
	m_SimulationParameters.collisionFactor = 1;
	m_SimulationParameters.linearFriction = 0;		// Disable linear friction
	m_SimulationParameters.angularFriction = 0;		// Disable angular friction
}

const char* OpenSimulation::getTestCasesStr() {
	return
		"Trampolin1,"
		"Trampolin2,"
		"Trampolin3,"
		"Trampolin4";
}

// Called when we reset the scene, or change the test case:
void OpenSimulation::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case DEMO1_TRAMPOLIN1:
		break;

	case DEMO2_TRAMPOLIN2:
		// some interaction methods
		//TwAddButton(DUC->g_pTweakBar, "Explosion", [](void* s) { ((RigidBodySystemSimulator*)g_pSimulator)->startExplosion(); }, nullptr, "");
		break;

	case DEMO3_TRAMPOLIN3:
		//TwAddVarRW(DUC->g_pTweakBar, "Collision factor", TW_TYPE_FLOAT, &m_SimulationParameters.collisionFactor, "min=0 max=1 step=0.01");
		break;

	case DEMO4_TRAMPOLIN4:
		//TwAddVarRW(DUC->g_pTweakBar, "Collision factor", TW_TYPE_FLOAT, &m_SimulationParameters.collisionFactor, "min=0 max=1 step=0.01");
		//TwAddVarRW(DUC->g_pTweakBar, "Linear friction", TW_TYPE_FLOAT, &m_SimulationParameters.linearFriction, "min=0 max=0.05 step=0.001");
		//TwAddVarRW(DUC->g_pTweakBar, "Angular friction", TW_TYPE_FLOAT, &m_SimulationParameters.angularFriction, "min=0 max=0.05 step=0.001");
		//TwAddVarRW(DUC->g_pTweakBar, "Gravity force", TW_TYPE_FLOAT, &m_fGravityForce, "min=1000 max=100000 step=1000");
		//TwAddButton(DUC->g_pTweakBar, "Fire Rigidbody", [](void* s) { ((RigidBodySystemSimulator*)g_pSimulator)->fireRigidbody(); }, nullptr, "");
		//TwAddVarRW(DUC->g_pTweakBar, "Box size", TW_TYPE_FLOAT, &m_fBoxSize, "min=0.5 max=5 step=0.1");
		//TwAddButton(DUC->g_pTweakBar, "Explosion", [](void* s) { ((RigidBodySystemSimulator*)g_pSimulator)->startExplosion(); }, nullptr, "");
		//
		//TwType TW_TYPE_METHOD;
		//TW_TYPE_METHOD = TwDefineEnumFromString("Debug lines", "None,Linear Velocity,Angular Velocity,Forces");
		//TwAddVarRW(DUC->g_pTweakBar, "Debug lines", TW_TYPE_METHOD, &m_iDebugLine, "");
		break;

	default:
		break;
	}
}

// Called once, at the beginning of the program:
void OpenSimulation::reset() {
	m_prevmouse.x = m_prevmouse.y = 0;
	m_bMousePressed = false;
}

void OpenSimulation::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	// Draw all the rigidbodies:
	//for (auto& r : m_vRigidbodies)
	//	r.draw(DUC, m_iDebugLine);
	// Sleep(1);
}

// Called when we reset the scene, or change the test case:
void OpenSimulation::notifyCaseChanged(int testCase) {
	Vec3 velocity;

	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case DEMO1_TRAMPOLIN1:
		//cout << "Switch to Demo1: One-step !" << endl;
		//setupDemoSingleBody();

		// Compute a time step of 2, using Euler method:
		//m_vRigidbodies[0].timestepEuler(2);
		//
		//cout << "Result of the simulation: Euler, one step with h = 2" << endl;
		//cout << "Linear velocity: " << getLinearVelocityOfRigidBody(0) << endl;
		//cout << "Angular velocity: " << getAngularVelocityOfRigidBody(0) << endl;
		//
		//velocity = m_vRigidbodies[0].getVelocityOfPoint(Vec3(0.3, 0.5, 0.25));
		//cout << "World space velocity of point (0.3, 0.5, 0.25): " << velocity << endl;
		break;

	case DEMO2_TRAMPOLIN2:
		//cout << "Switch to Demo2: Single body !" << endl;
		//setupDemoSingleBody();
		break;

	case DEMO3_TRAMPOLIN3:
		//cout << "Switch to Demo3: Collision between two rigidbodies !" << endl;
		//setupDemoCollision();
		break;

	case DEMO4_TRAMPOLIN4:
		//cout << "Switch to Demo4: Complex !" << endl;
		//setupDemoComplex();
		break;

	default:
		cout << "Empty test !" << endl;
		break;
	}
}

// Called each time the mouse is moved while pressed:
void OpenSimulation::externalForcesCalculations(float timeElapsed) {
	Vec3 pullforce(0, 0, 0);
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 forceView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 forceWorld = worldViewInv.transformVectorNormal(forceView);
		float forceScale = 0.2f;
		pullforce = pullforce + (forceWorld * forceScale);
	}

	m_vExternalForce = pullforce;
}

void OpenSimulation::simulateTimestep(float timestep) {
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case DEMO1_TRAMPOLIN1:
		// This case requires only one update, and is thus handled by notifyCaseChanged()
		break;

	case DEMO2_TRAMPOLIN2:
	case DEMO3_TRAMPOLIN3:
	case DEMO4_TRAMPOLIN4:
		// Compute forces applied to the rigidbodies dynamically:
		//updateForces();
		//
		//for (Rigidbody& r : m_vRigidbodies)
		//	r.timestepEuler(timestep);
		//
		//manageCollisions();
		break;

	default:
		break;
	}
}

// This function is called when the mouse is clicked, and when the mouse is moved while clicked:
void OpenSimulation::onClick(int x, int y)
{
	// added for adding external force
	m_trackmouse.x = x;
	m_trackmouse.y = y;

	// The first frame where we have a mouse pressed, we call mousePressed()
	if (!m_bMousePressed) {
		m_bMousePressed = true;
		mousePressed(x, y);
	}

	// Then, if the mouse was pressed in the previous frame, and is still pressed, 
	// we call mouseDragged()
	else
		mouseDragged(x, y);

	m_prevmouse.x = x;
	m_prevmouse.y = y;
}

// This function is called when the mouse moves or is released, but not clicked:
void OpenSimulation::onMouse(int x, int y)
{
	// added for adding external force
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;

	// The first frame where we have a mouse release, we call mouseReleased()
	if (m_bMousePressed) {
		m_bMousePressed = false;
		mouseReleased(x, y);
	}

	// Then, if the mouse was released in the previous frame, and is still released, 
	// we call mouseMoved()
	else
		mouseMoved(x, y);

	m_prevmouse.x = x;
	m_prevmouse.y = y;
}

void OpenSimulation::mousePressed(int x, int y) {
	// First frame after a mouse press
}

void OpenSimulation::mouseReleased(int x, int y) {
	// First frame after a mouse release
	if (m_iTestCase == DEMO2_TRAMPOLIN2) {
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), m_vExternalForce);
	}
}

void OpenSimulation::mouseDragged(int x, int y) {
	// Mouse moved while pressed
}

void OpenSimulation::mouseMoved(int x, int y) {
	// Mouse moved while released
}