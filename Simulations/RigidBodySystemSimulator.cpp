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
		// some interaction methods
		TwAddButton(DUC->g_pTweakBar, "Explosion", [](void* s) { ((RigidBodySystemSimulator*)g_pSimulator)->startExplosion(); }, nullptr, "");
		break;

	case DEMO3_COLLISION:
		TwAddVarRW(DUC->g_pTweakBar, "Collision factor", TW_TYPE_FLOAT, &m_SimulationParameters.collisionFactor, "min=0 max=1 step=0.01");
		break;

	case DEMO4_COMPLEX:
		TwAddVarRW(DUC->g_pTweakBar, "Collision factor", TW_TYPE_FLOAT, &m_SimulationParameters.collisionFactor, "min=0 max=1 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Linear friction", TW_TYPE_FLOAT, &m_SimulationParameters.linearFriction, "min=0 max=0.05 step=0.001");
		TwAddVarRW(DUC->g_pTweakBar, "Angular friction", TW_TYPE_FLOAT, &m_SimulationParameters.angularFriction, "min=0 max=0.05 step=0.001");
		TwAddVarRW(DUC->g_pTweakBar, "Gravity force", TW_TYPE_FLOAT, &m_fGravityForce, "min=1000 max=100000 step=1000");
		TwAddButton(DUC->g_pTweakBar, "Fire Rigidbody", [](void* s) { ((RigidBodySystemSimulator*)g_pSimulator)->fireRigidbody(); }, nullptr, "");
		TwAddVarRW(DUC->g_pTweakBar, "Box size", TW_TYPE_FLOAT, &m_fBoxSize, "min=0.5 max=5 step=0.1");
		TwAddButton(DUC->g_pTweakBar, "Explosion", [](void* s) { ((RigidBodySystemSimulator*)g_pSimulator)->startExplosion(); }, nullptr, "");

		TwType TW_TYPE_METHOD;
		TW_TYPE_METHOD = TwDefineEnumFromString("Debug lines", "None,Linear Velocity,Angular Velocity,Forces");
		TwAddVarRW(DUC->g_pTweakBar, "Debug lines", TW_TYPE_METHOD, &m_iDebugLine, "");
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
	// Draw all the rigidbodies:
	for (auto& r : m_vRigidbodies)
		r.draw(DUC, m_iDebugLine);

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
		setupDemoSingleBody();

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
		setupDemoSingleBody();
		break;

	case DEMO3_COLLISION:
		cout << "Switch to Demo3: Collision between two rigidbodies !" <<endl;
		setupDemoCollision();
		break;

	case DEMO4_COMPLEX:
		cout << "Switch to Demo4: Complex !" << endl;
		setupDemoComplex();
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
		// Compute forces applied to the rigidbodies dynamically:
		updateForces();

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

void RigidBodySystemSimulator::setupDemoSingleBody()
{
	m_vRigidbodies.clear();

	// Setup simulation parameters:
	m_SimulationParameters.collisionFactor = 1;
	m_SimulationParameters.angularFriction = 0;
	m_SimulationParameters.linearFriction = 0;

	float mass = 2;
	Vec3 position = Vec3(0, 0, 0);
	Vec3 rotation = Vec3(0, 0, 90);
	Vec3 scale = Vec3(1, 0.6, 0.5);

	Rigidbody r = Rigidbody(&m_SimulationParameters, mass, position, rotation, scale);
	r.addTorque(Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
	m_vRigidbodies.push_back(r);
}

void RigidBodySystemSimulator::setupDemoCollision()
{
	m_vRigidbodies.clear();

	// Setup simulation parameters:
	m_SimulationParameters.collisionFactor = 0.9;
	m_SimulationParameters.angularFriction = 0;
	m_SimulationParameters.linearFriction = 0;
	g_fTimestep = 0.003;

	Rigidbody ground = Rigidbody(&m_SimulationParameters, 1, Vec3(0, -1, 0), Vec3(0, 0, 0), Vec3(10, 1, 10));
	ground.color = Vec3(0.1);
	ground.setKinematic(true);
	
	Rigidbody plank = Rigidbody(&m_SimulationParameters, 1, Vec3(0, 0, 0), Vec3(0, 0, 20), Vec3(2, 0.1, 0.01));
	plank.color = Vec3(0.6, 0.27, 0.03);
	plank.setForce(Vec3(0, -GRAVITY_FACTOR, 0));

	m_vRigidbodies.push_back(ground);
	m_vRigidbodies.push_back(plank);
}

void RigidBodySystemSimulator::setupDemoComplex()
{
	m_vRigidbodies.clear();
	m_SimulationParameters.angularFriction = 0;
	m_SimulationParameters.linearFriction = 0;

	const Vec3 center = Vec3(0, 2, 0);
	const float CENTER_BOX_SIZE = 1.5;
	const float BOX_SIZE = 1;
	const float BOX_SPACE = 2;	// Space between boxes

	Rigidbody centerBox = Rigidbody(&m_SimulationParameters,
		CENTER_BOX_SIZE*CENTER_BOX_SIZE*CENTER_BOX_SIZE,		// mass
		center,													// position
		Vec3(0.0),												// rotation
		Vec3(CENTER_BOX_SIZE));									// scale
	centerBox.color = Vec3(1, 0.5, 0);
	centerBox.setAngularVelocity(Vec3(10, 50, 30));
	centerBox.setLinearVelocity(Vec3(0.1, 0, 0.1));
	m_vRigidbodies.push_back(centerBox);

	for (int i = 1; i <= 8; i++) {
		float radius = i * BOX_SPACE;

		Rigidbody box = Rigidbody(&m_SimulationParameters,
			BOX_SIZE * BOX_SIZE * BOX_SIZE,		// mass
			center + Vec3(radius, 0, 0),		// position
			Vec3(0.0),							// rotation
			Vec3(BOX_SIZE));					// scale
		box.color = Vec3(0.5*randCol(eng), randCol(eng), 1);

		// Set the velocity that should theoretically make the 
		// box rotate in circles around the center (Kepler's Law):
		box.setLinearVelocity(Vec3(0, 0, sqrt(m_fGravityForce / radius)));

		m_vRigidbodies.push_back(box);
	}
}

void RigidBodySystemSimulator::updateForces() {
	if (m_iTestCase == DEMO4_COMPLEX) {
		const Vec3 center = Vec3(0, 2, 0);
		const float MAX_DISTANCE = 20;

		for (Rigidbody& r : m_vRigidbodies) {
			Vec3 n = center - r.getPosition();
			float distance = norm(n);

			if (distance == 0)
				continue;

			// Prevent any rigidbody from going too far from the center of the scene:
			if (distance > MAX_DISTANCE) {
				r.setPosition(center - MAX_DISTANCE * n / distance);
				r.setLinearVelocity(Vec3(0.0));
			}

			float MAX_FORCE = 10000;
			float MIN_FORCE = 10;

			// Add a gravity force to the object:
			float force = m_fGravityForce * r.getMass() / (distance * distance);
			force = max(MIN_FORCE, min(force, MAX_FORCE));

			// Apply the force to the rigidbody:
			r.setForce(force * n / distance);
		}
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

void RigidBodySystemSimulator::fireRigidbody()
{
	// Fire a cube in the scene, from the camera:
	Mat4 worldView = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());

	// Position of the camera in world :
	Vec3 inputWorld = worldView.inverse().transformVector(Vec3(0, 0, 0));

	// Create a box at the position of the camera:
	Rigidbody box = Rigidbody(&m_SimulationParameters, 
		m_fBoxSize*m_fBoxSize*m_fBoxSize,	// mass
		inputWorld,							// position (position of the camera)
		Quat(worldView),					// rotation (rotation of the camera)
		Vec3(m_fBoxSize));					// scale

	box.color = Vec3(randCol(eng), randCol(eng), randCol(eng));

	// Throw the box to the center of the scene:
	box.setLinearVelocity(-20 * inputWorld / norm(inputWorld));

	m_vRigidbodies.push_back(box);
}

void RigidBodySystemSimulator::startExplosion() {

	Vec3 center;
	float EXPLOSION_FORCE;
	float MAX_FORCE;
	if (m_iTestCase == DEMO4_COMPLEX ) {
		center = Vec3(0, 2, 0);
		EXPLOSION_FORCE = 3000;
		MAX_FORCE = 150;
	}
	else if (m_iTestCase == DEMO2_SINGLE_BODY) {
		center = Vec3(0, 0, 0);
		EXPLOSION_FORCE = 3;
		MAX_FORCE = 3;
	}
	else {  // default case
		center = Vec3(0, 2, 0);
		EXPLOSION_FORCE = 3000;
		MAX_FORCE = 150;
	}

	for (Rigidbody& r : m_vRigidbodies) {
		Vec3 n = r.getPosition() - center;
		float distance = norm(n);

		if (distance == 0)
			continue;

		// Add a gravity force to the object:
		float force = min(EXPLOSION_FORCE / (distance * distance), MAX_FORCE);

		// Apply the force to the rigidbody:
		r.setLinearVelocity(r.getLinearVelocity() + force * n / distance);
	}

}
