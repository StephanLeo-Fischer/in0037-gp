#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
	// UI Attributes
	m_prevmouse = { 0, 0 };
	m_bMousePressed = false;
	m_bKeyF_Pressed = false;

	// Data Attributes
	m_SimulationParameters = {};
	m_SimulationParameters.collisionFactor = 0.6;
	m_SimulationParameters.enableMicroCollisions = true;
	m_SimulationParameters.linearFriction = 1;
	m_SimulationParameters.angularFriction = 1;
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
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
		TwAddVarRW(DUC->g_pTweakBar, "Collision factor", TW_TYPE_FLOAT, &m_SimulationParameters.collisionFactor, "min=0 max=1 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Enable micro collisions", TW_TYPE_BOOLCPP, &m_SimulationParameters.enableMicroCollisions, "");
		TwAddVarRW(DUC->g_pTweakBar, "Linear friction", TW_TYPE_FLOAT, &m_SimulationParameters.linearFriction, "min=0 max=1 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Angular friction", TW_TYPE_FLOAT, &m_SimulationParameters.angularFriction, "min=0 max=1 step=0.01");
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
	manageKeyEvents();

	// Draw all the rigidbodies:
	for (auto& r : m_vRigidbodies)
		r.draw(DUC);

	Sleep(1);
}

// Called when we reset the scene, or change the test case:
void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
	Vec3 velocity;

	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case DEMO1_ONESTEP:
		cout << "Switch to Demo1: One-step !" << endl;
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
		cout << "Switch to Demo2: Single body !" << endl;
		setupDemo1();
		break;

	case DEMO3_COLLISION:
		cout << "Switch to Demo3: Collision between two rigidbodies !" <<endl;
		setupDemo2();
		break;

	case DEMO4_COMPLEX:
		cout << "Switch to Demo4: Complex !" << endl;
		setupTower();
		break;

	default:
		cout << "Empty test !" << endl;
		break;
	}
}

// Called each time the mouse is moved while pressed:
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed){}

void RigidBodySystemSimulator::simulateTimestep(float timestep) {
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
	m_vRigidbodies.at(i).applyTorque(loc, force);
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

void RigidBodySystemSimulator::setupDemo1()
{
	m_vRigidbodies.clear();

	float mass = 2;
	Vec3 position = Vec3(0, 0, 0);
	Vec3 rotation = Vec3(0, 0, 90);
	Vec3 scale = Vec3(1, 0.6, 0.5);

	Rigidbody r = Rigidbody(&m_SimulationParameters, mass, position, rotation, scale);
	r.applyTorque(Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
	m_vRigidbodies.push_back(r);
}

void RigidBodySystemSimulator::setupDemo2()
{
	m_vRigidbodies.clear();

	Rigidbody ground = Rigidbody(&m_SimulationParameters, 1, Vec3(0, -1, 0), Vec3(0, 0, 0), Vec3(10, 1, 10));
	ground.color = Vec3(0.1);
	ground.setKinematic(true);
	
	Rigidbody plank = Rigidbody(&m_SimulationParameters, 1, Vec3(0, 0, 0), Vec3(0, 0, 20), Vec3(2, 0.1, 0.01));
	plank.color = Vec3(0.6, 0.27, 0.03);
	plank.applyForce(Vec3(0, -GRAVITY_FACTOR, 0));

	m_vRigidbodies.push_back(ground);
	m_vRigidbodies.push_back(plank);
}

void RigidBodySystemSimulator::setupComplex()
{
	m_vRigidbodies.clear();

	Rigidbody ground = Rigidbody(&m_SimulationParameters, 1, Vec3(0, -1, 0), Vec3(0, 0, 0), Vec3(10, 1, 10));
	ground.color = Vec3(0.1);
	ground.setKinematic(true);

	m_vRigidbodies.push_back(ground);

	Rigidbody box1 = Rigidbody(&m_SimulationParameters, 1, Vec3(0, 0.0, 0), Vec3(0, 0, 0), Vec3(1.0, 0.1, 1.0));
	Rigidbody box2 = Rigidbody(&m_SimulationParameters, 1, Vec3(0, 0.4, 0), Vec3(0, 0, 0), Vec3(0.8, 0.1, 0.8));
	Rigidbody box3 = Rigidbody(&m_SimulationParameters, 1, Vec3(0, 0.6, 0), Vec3(0, 0, 0), Vec3(0.6, 0.1, 0.6));
	Rigidbody box4 = Rigidbody(&m_SimulationParameters, 1, Vec3(0, 0.8, 0), Vec3(0, 0, 0), Vec3(0.4, 0.1, 0.4));
	Rigidbody box5 = Rigidbody(&m_SimulationParameters, 1, Vec3(0, 1.0, 0), Vec3(0, 0, 0), Vec3(0.2, 0.1, 0.2));

	box1.applyForce(Vec3(0, -GRAVITY_FACTOR, 0));
	box2.applyForce(Vec3(0, -GRAVITY_FACTOR, 0));
	box3.applyForce(Vec3(0, -GRAVITY_FACTOR, 0));
	box4.applyForce(Vec3(0, -GRAVITY_FACTOR, 0));
	box5.applyForce(Vec3(0, -GRAVITY_FACTOR, 0));

	m_vRigidbodies.push_back(ground);
	m_vRigidbodies.push_back(box1);
	m_vRigidbodies.push_back(box2);
	m_vRigidbodies.push_back(box3);
	m_vRigidbodies.push_back(box4);
	m_vRigidbodies.push_back(box5);
}

void RigidBodySystemSimulator::setupTower()
{
	const float GROUND_POSITION = -0.5;
	const int TOWER_SIZE = 10;
	const float BOX_SIZE = 0.4;

	m_vRigidbodies.clear();

	Rigidbody ground = Rigidbody(&m_SimulationParameters, 1, Vec3(0, GROUND_POSITION - 0.5, 0), Vec3(0, 0, 0), Vec3(20, 1, 20));
	ground.color = Vec3(0.1);
	ground.setKinematic(true);
	m_vRigidbodies.push_back(ground);

	Vec3 startPosition = Vec3(0, GROUND_POSITION + 0.1, 0);
	Vec3 startDirection = Vec3(0, 0, 1);

	for (int i = 0; i < TOWER_SIZE; i++) {
		Rigidbody box = Rigidbody(&m_SimulationParameters, 1, Vec3(0, GROUND_POSITION + BOX_SIZE * (i + 0.5), 0), Vec3(0, 5*i, 0), Vec3(BOX_SIZE));
		box.color = Vec3(0.6, 0.27, 0.03);
		box.applyForce(Vec3(0, -GRAVITY_FACTOR * 10, 0));

		m_vRigidbodies.push_back(box);
	}
}

void RigidBodySystemSimulator::setupDominos()
{
	const float GROUND_POSITION = -0.5;
	const int N_DOMINOS = 10;
	const float SPACE_BETWEEN_DOMINOS = 0.2;

	m_vRigidbodies.clear();

	Rigidbody ground = Rigidbody(&m_SimulationParameters, 1, Vec3(0, GROUND_POSITION - 0.5, 0), Vec3(0, 0, 0), Vec3(20, 1, 20));
	ground.color = Vec3(0.1);
	ground.setKinematic(true);
	m_vRigidbodies.push_back(ground);

	Vec3 startPosition = Vec3(0, GROUND_POSITION + 0.1, 0);
	Vec3 startDirection = Vec3(0, 0, 1);

	for(int i = 0; i < N_DOMINOS; i++) {
		Rigidbody domino = Rigidbody(&m_SimulationParameters, 1, startPosition + startDirection * i * SPACE_BETWEEN_DOMINOS, Vec3(10, 0, 0), Vec3(0.1, 0.2, 0.02));
		domino.color = Vec3(0.6, 0.27, 0.03);
		domino.applyForce(Vec3(0, -GRAVITY_FACTOR*10, 0));

		m_vRigidbodies.push_back(domino);
	}
}

void RigidBodySystemSimulator::manageCollisions()
{
	for (int i = 0; i < m_vRigidbodies.size(); i++) {
		for (int j = i+1; j < m_vRigidbodies.size(); j++) {

			// Manage collision between rigidbodies i and j:
			m_vRigidbodies[i].manageCollision(&m_vRigidbodies[j]);
		}
	}
}

void RigidBodySystemSimulator::manageKeyEvents() {
	if (DXUTIsKeyDown(0x46) && !m_bKeyF_Pressed) {
		// If the key 'F' is pressed, but wasn't pressed before:

		if (m_iTestCase == DEMO4_COMPLEX)
			fireRigidbody();

		m_bKeyF_Pressed = true;
	}
	else if (!DXUTIsKeyDown(0x46) && m_bKeyF_Pressed) {
		// If the key 'F' is released, but was pressed before:

		m_bKeyF_Pressed = false;
	}
}

void RigidBodySystemSimulator::fireRigidbody()
{
	// Fire a cube in the scene, from the camera:
	Mat4 worldView = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());

	// Position of the camera in world :
	Vec3 inputWorld = worldView.inverse().transformVector(Vec3(0, 0, 0));

	// Create a box at the position of the camera:
	Rigidbody bulletBox = Rigidbody(&m_SimulationParameters, 0.1, inputWorld, Quat(worldView), Vec3(0.05));
	bulletBox.color = Vec3(1, 0, 0);

	bulletBox.applyForce(Vec3(0, -GRAVITY_FACTOR, 0));

	// Throw the box to the center of the scene:
	bulletBox.setLinearVelocity(-20 * inputWorld / norm(inputWorld));

	m_vRigidbodies.push_back(bulletBox);
}
