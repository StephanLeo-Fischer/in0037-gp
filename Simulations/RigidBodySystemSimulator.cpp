#include "RigidBodySystemSimulator.h"

extern Simulator* g_pSimulator;
extern float g_fTimestep;

RigidBodySystemSimulator::RigidBodySystemSimulator() {
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
	m_SimulationParameters.minimumImpulse = 0.05;
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return
		"Test demo,"
		"Angry birds demo,"
		"Collisions debug,";
}

// Called when we reset the scene, or change the test case:
void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case TEST_DEMO:
		TwAddVarRW(DUC->g_pTweakBar, "Fire scenario", TW_TYPE_INT32, &m_iTestScenario, "min=0 max=3");

	case ANGRY_BIRDS_DEMO:
		m_iTestScenario = 3;
		TwAddVarRW(DUC->g_pTweakBar, "Collision factor", TW_TYPE_DOUBLE, &m_SimulationParameters.collisionFactor, "min=0 max=1 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Linear friction", TW_TYPE_DOUBLE, &m_SimulationParameters.linearFriction, "min=0 max=0.05 step=0.001");
		TwAddVarRW(DUC->g_pTweakBar, "Angular friction", TW_TYPE_DOUBLE, &m_SimulationParameters.angularFriction, "min=0 max=0.05 step=0.001");
		TwAddVarRW(DUC->g_pTweakBar, "Minimum impulse", TW_TYPE_DOUBLE, &m_SimulationParameters.minimumImpulse, "min=0 step=0.001");

		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "min=0");

		TwAddVarRW(DUC->g_pTweakBar, "Correct position", TW_TYPE_BOOLCPP, &m_bEnablePositionCorrection, "");

		TwAddButton(DUC->g_pTweakBar, "Fire Rigidbody", [](void* s) { ((RigidBodySystemSimulator*)g_pSimulator)->fireRigidbody(); }, nullptr, "");
		
		TwType TW_TYPE_METHOD;
		TW_TYPE_METHOD = TwDefineEnumFromString("Debug lines", "None,Linear Velocity,Angular Velocity,Angular Momentum,Forces");
		TwAddVarRW(DUC->g_pTweakBar, "Debug lines", TW_TYPE_METHOD, &m_iDebugLine, "");
		break;

	case COLLISIONS_DEMO:
		break;

	default:
		break;
	}
}

// Called once, at the beginning of the program:
void RigidBodySystemSimulator::reset() {
	m_prevmouse.x = m_prevmouse.y = 0;
	m_bMousePressed = false;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {

	if (m_iTestCase == COLLISIONS_DEMO) {
		collisionDebugger.draw(DUC);
	}
	else {
		// Draw all the rigidbodies:
		for (auto& r : m_vRigidbodies)
			r.draw(DUC, m_iDebugLine);
	}
	
	Sleep(1);
}

// Called when we reset the scene, or change the test case:
void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
	Vec3 velocity;

	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case TEST_DEMO:
		cout << "Switch to Test demo: Fire rigidbodies to see the simulator stability !" << endl;
		setupTestDemo();
		break;

	case ANGRY_BIRDS_DEMO:
		cout << "Switch to Angry Birds Demo !" << endl;
		setupAngryBirdsDemo();
		break;

	case COLLISIONS_DEMO:
		cout << "Switch to Collision debug: See the collision point returned by SAT !" <<endl;
		m_vRigidbodies.clear();
		break;
	}
}

// Called each time the mouse is moved while pressed:
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed){}

void RigidBodySystemSimulator::simulateTimestep(float timestep) {
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case TEST_DEMO:
	case ANGRY_BIRDS_DEMO:
		// Compute forces applied to the rigidbodies dynamically:
		// updateForces();

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
void RigidBodySystemSimulator::onMouse(int x, int y)
{
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

void RigidBodySystemSimulator::mousePressed(int x, int y) {
	// First frame after a mouse press
}

void RigidBodySystemSimulator::mouseReleased(int x, int y) {
	// First frame after a mouse release	
}

void RigidBodySystemSimulator::mouseDragged(int x, int y) {
	// Mouse moved while pressed
}

void RigidBodySystemSimulator::mouseMoved(int x, int y) {
	// Mouse moved while released
}

int RigidBodySystemSimulator::getNumberOfRigidBodies() {
	return m_vRigidbodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	return m_vRigidbodies.at(i).getPosition();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	return m_vRigidbodies.at(i).getLinearVelocity();
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	return m_vRigidbodies.at(i).getAngularVelocity();
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	m_vRigidbodies.at(i).addTorque(loc, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {
	m_vRigidbodies.push_back(Rigidbody(&m_SimulationParameters, mass, position, Vec3(0.0), size));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	m_vRigidbodies.at(i).setRotation(orientation);
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
	m_vRigidbodies.at(i).setLinearVelocity(velocity);
}


void RigidBodySystemSimulator::setupTestDemo()
{
	m_vRigidbodies.clear();

	// Setup simulation parameters:
	m_SimulationParameters.collisionFactor = 0.2;
	m_SimulationParameters.angularFriction = 0.008;
	m_SimulationParameters.linearFriction = 0.008;
	g_fTimestep = 0.003f;

	Rigidbody ground = Rigidbody(&m_SimulationParameters, 1, Vec3(0, -0.05, 0), Vec3(0, 0, 0), Vec3(10, 0.1, 10));
	ground.setKinematic(true);
	m_vRigidbodies.push_back(ground);
}

void RigidBodySystemSimulator::setupAngryBirdsDemo() {
	m_vRigidbodies.clear();

	// Setup simulation parameters:
	m_SimulationParameters.collisionFactor = 0.2;
	m_SimulationParameters.angularFriction = 0.008;
	m_SimulationParameters.linearFriction = 0.008;
	g_fTimestep = 0.003f;

	Rigidbody ground = Rigidbody(&m_SimulationParameters, 1, Vec3(0, -0.05, 0), Vec3(0, 0, 0), Vec3(10, 0.1, 10));
	ground.setKinematic(true);
	m_vRigidbodies.push_back(ground);

	const float mass = 1;

	// For testing: create a kind of angry birds game:
	Rigidbody planks[10]{
		Rigidbody(&m_SimulationParameters, mass, Vec3(-1, 0.1, 0), Vec3(0.0), Vec3(0.1, 0.2, 0.1)),
		Rigidbody(&m_SimulationParameters, mass, Vec3(0, 0.1, 0), Vec3(0.0), Vec3(0.1, 0.2, 0.1)),
		Rigidbody(&m_SimulationParameters, mass, Vec3(-1, 0.5, 0), Vec3(0.0), Vec3(0.1, 0.4, 0.1)),
		Rigidbody(&m_SimulationParameters, mass, Vec3(0, 0.5, 0), Vec3(0.0), Vec3(0.1, 0.4, 0.1)),
		Rigidbody(&m_SimulationParameters, mass, Vec3(0.6, 0.4, 0), Vec3(0.0), Vec3(0.1, 0.4, 0.1)),
		Rigidbody(&m_SimulationParameters, mass, Vec3(1.1, 0.4, 0), Vec3(0.0), Vec3(0.1, 0.4, 0.1)),
		Rigidbody(&m_SimulationParameters, mass, Vec3(0.6, 0.65, 0), Vec3(0.0), Vec3(0.4, 0.1, 0.1)),
		Rigidbody(&m_SimulationParameters, mass, Vec3(1.275, 0.65, 0), Vec3(0.0), Vec3(0.65, 0.1, 0.1)),
		Rigidbody(&m_SimulationParameters, mass, Vec3(-0.5, 0.25, 0), Vec3(0.0), Vec3(1.2, 0.1, 0.1)),
		Rigidbody(&m_SimulationParameters, mass, Vec3(-0.5, 0.75, 0), Vec3(0.0), Vec3(1.2, 0.1, 0.1)),
	};
	
	for (int i = 0; i < 10; i++) {
		planks[i].color = Vec3(120, 57, 0) / 255.0f;
		planks[i].setForce(Vec3(0, -m_fGravity, 0));
		m_vRigidbodies.push_back(planks[i]);
	}
	
	Rigidbody ground1 = Rigidbody(&m_SimulationParameters, mass, Vec3(1, 0.1, 0), Vec3(0.0), Vec3(1.2, 0.2, 0.1));
	Rigidbody ground2 = Rigidbody(&m_SimulationParameters, mass, Vec3(1.5, 0.4, 0), Vec3(0.0), Vec3(0.2, 0.4, 0.1));
	ground1.setKinematic(true);
	ground2.setKinematic(true);
	m_vRigidbodies.push_back(ground1);
	m_vRigidbodies.push_back(ground2);

	Rigidbody pigs[5]{
		Rigidbody(&m_SimulationParameters, mass, Vec3(-0.75, 0.4, 0), Vec3(0.0), Vec3(0.2, 0.2, 0.2)),
		Rigidbody(&m_SimulationParameters, mass, Vec3(-0.25, 0.4, 0), Vec3(0.0), Vec3(0.2, 0.2, 0.2)),
		Rigidbody(&m_SimulationParameters, mass, Vec3(-0.5, 0.9, 0), Vec3(0.0), Vec3(0.2, 0.2, 0.2)),
		Rigidbody(&m_SimulationParameters, mass, Vec3(0.6, 0.8, 0), Vec3(0.0), Vec3(0.2, 0.2, 0.2)),
		Rigidbody(&m_SimulationParameters, mass, Vec3(1.2, 0.8, 0), Vec3(0.0), Vec3(0.2, 0.2, 0.2)),
	};

	for (int i = 0; i < 5; i++) {
		pigs[i].color = Vec3(0, 120, 0) / 255.0f;
		pigs[i].setForce(Vec3(0, -m_fGravity, 0));
		m_vRigidbodies.push_back(pigs[i]);
	}
}

void RigidBodySystemSimulator::manageCollisions()
{
	for (int i = 0; i < m_vRigidbodies.size(); i++) {
		for (int j = i + 1; j < m_vRigidbodies.size(); j++) {
			CollisionInfo collision = Rigidbody::computeCollision(&m_vRigidbodies[i], &m_vRigidbodies[j]);

			if (collision.isValid) {
				double J = Rigidbody::computeImpulse(&m_vRigidbodies[i], &m_vRigidbodies[j], &m_SimulationParameters,
							collision.collisionPointWorld, collision.normalWorld, collision.depth);

				if (J < m_SimulationParameters.minimumImpulse && m_bEnablePositionCorrection) {
					if (!m_vRigidbodies[i].isKinematic() && !m_vRigidbodies[j].isKinematic())
						Rigidbody::correctPosition(&m_vRigidbodies[i], &m_vRigidbodies[j], collision.collisionPointWorld, -collision.normalWorld, collision.depth);

					else if (!m_vRigidbodies[i].isKinematic())
						Rigidbody::correctPosition(&m_vRigidbodies[i], collision.collisionPointWorld, collision.normalWorld, collision.depth);
					
					else if (!m_vRigidbodies[j].isKinematic())
						Rigidbody::correctPosition(&m_vRigidbodies[j], collision.collisionPointWorld, -collision.normalWorld, collision.depth);
				}
			}
		}
	}
}

void RigidBodySystemSimulator::fireRigidbody()
{
	// Use different scenarios to test the simulator in different situations:
	
	if (m_iTestScenario == 0) {	// Small objects with no velocity
		Rigidbody box = Rigidbody(&m_SimulationParameters, 1, Vec3(0.5, 0.5, 0), Vec3(10, 45, 10), Vec3(0.04, 0.1, 0.02));
		box.setForce(Vec3(0, -m_fGravity, 0));
		m_vRigidbodies.push_back(box);
	}

	else if (m_iTestScenario == 1) {	// Throw spinning boxes
		Rigidbody box = Rigidbody(&m_SimulationParameters, 1, Vec3(0.5, 0.5, 0), Vec3(0.0), Vec3(0.2, 0.04, 0.2));
		box.setForce(Vec3(0, -m_fGravity, 0));
		box.setLinearVelocity(Vec3(-10, 0, 0));
		box.setAngularVelocity(Vec3(0, 80, 0));
		m_vRigidbodies.push_back(box);
	}
	
	else if (m_iTestScenario == 2) {	// Stack boxes to make a tower
		Rigidbody box = Rigidbody(&m_SimulationParameters, 1, Vec3(0, 2, 0), Vec3(0.0), Vec3(1, 0.1, 1));
		box.setForce(Vec3(0, -m_fGravity, 0));
		box.setAngularVelocity(Vec3(0, 80, 0));
		m_vRigidbodies.push_back(box);
	}

	else if (m_iTestScenario == 3) {	// For angry birds
		Rigidbody bird = Rigidbody(&m_SimulationParameters, 10, Vec3(-3, 0.5, 0), Vec3(0.0), Vec3(0.2, 0.2, 0.2));
		bird.color = Vec3(1, 0, 0);
		bird.setForce(Vec3(0, -m_fGravity, 0));
		bird.setLinearVelocity(Vec3(10, 2, 0));
		m_vRigidbodies.push_back(bird);
	}
}
