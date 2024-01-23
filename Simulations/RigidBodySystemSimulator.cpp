#include "RigidBodySystemSimulator.h"

extern Simulator* g_pSimulator;
extern float g_fTimestep;

RigidBodySystemSimulator::RigidBodySystemSimulator() : cannon(1, 15) {
	// UI Attributes
	m_prevmouse = { 0, 0 };
	m_bMousePressed = false;

	// randFloat will be a random value between 0.0 and 1.0:
	randFloat = std::uniform_real_distribution<float>(0.0f, 1.0f);

	// Data Attributes
	g_fTimestep = 0.005;

	m_SimulationParameters = {};
	m_SimulationParameters.collisionFactor = 0.2;

	m_SimulationParameters.airFriction = 0.003;
	m_SimulationParameters.objectFriction = 0.015;

	m_SimulationParameters.minimumImpulse = 0.05;

	m_SimulationParameters.maxLinearCorrectionSpeed = 0.2;
	m_SimulationParameters.maxAngularCorrectionSpeed = 0.3;

	m_SimulationParameters.sqMinimumLinearVelocity = 0.215;
	m_SimulationParameters.sqMinimumAngularVelocity = 0.1;

	cannon.addBezierPoint(Vec3(-4, 0, -4), Vec3(1, 0, 0));
	cannon.addBezierPoint(Vec3(1, 0, -4),  Vec3(1, 0, 0));
	cannon.addBezierPoint(Vec3(3, 0, -2),  Vec3(0, 0, 1));
	cannon.addBezierPoint(Vec3(3, 0, 2),   Vec3(0, 0, 1));
	cannon.addBezierPoint(Vec3(1, 0, 4),   Vec3(-1, 0, 0));
	cannon.addBezierPoint(Vec3(-3, 0, 1), Vec3(0, 0, -1));
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return
		"Test demo,"
		"Angry birds demo,"
		"Springs demo,"
		"Collisions debug,";
}

// Called when we reset the scene, or change the test case:
void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case TEST_DEMO:
	case ANGRY_BIRDS_DEMO:
	case SPRINGS_DEMO:
		TwAddVarRW(DUC->g_pTweakBar, "Collision factor", TW_TYPE_DOUBLE, &m_SimulationParameters.collisionFactor, "min=0 max=1 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Air friction", TW_TYPE_DOUBLE, &m_SimulationParameters.airFriction, "min=0 max=0.5 step=0.001");
		TwAddVarRW(DUC->g_pTweakBar, "Objects friction", TW_TYPE_DOUBLE, &m_SimulationParameters.objectFriction, "min=0 max=0.5 step=0.001");
		TwAddVarRW(DUC->g_pTweakBar, "Minimum impulse", TW_TYPE_DOUBLE, &m_SimulationParameters.minimumImpulse, "min=0 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Max Linear correction speed", TW_TYPE_DOUBLE, &m_SimulationParameters.maxLinearCorrectionSpeed, "min=0 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Max Angular correction speed", TW_TYPE_DOUBLE, &m_SimulationParameters.maxAngularCorrectionSpeed, "min=0 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Min Linear velocity", TW_TYPE_DOUBLE, &m_SimulationParameters.sqMinimumLinearVelocity, "min=0 step=0.001");
		TwAddVarRW(DUC->g_pTweakBar, "Min Angular velocity", TW_TYPE_DOUBLE, &m_SimulationParameters.sqMinimumAngularVelocity, "min=0 step=0.001");

		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "min=0");

		TwAddVarRW(DUC->g_pTweakBar, "Fire scenario", TW_TYPE_INT32, &m_iTestScenario, "min=0 max=5");
		TwAddButton(DUC->g_pTweakBar, "Fire Rigidbody", [](void* s) { ((RigidBodySystemSimulator*)g_pSimulator)->fireRigidbody(); }, nullptr, "");
		
		TwType TW_TYPE_METHOD;
		TW_TYPE_METHOD = TwDefineEnumFromString("Debug lines", "None,Linear Velocity,Angular Velocity,Filtered Angular Velocity,Angular Momentum,Forces");
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
		for (Rigidbody* r : m_vRigidbodies)
			r->draw(DUC, m_iDebugLine);

		// Draw the springs from the spring structures:
		for (auto& s : m_vSpringStructures)
			s.drawSprings(DUC);

		// Draw the cannon:
		cannon.draw(DUC);
	}

	Sleep(1);
}

// Called when we reset the scene, or change the test case:
void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;

	// Make sure to delete all the rigidbodies, that were allocated on the heap !
	for (Rigidbody* r : m_vRigidbodies)
		delete r;

	// Then we can clear these two vectors:
	m_vRigidbodies.clear();
	m_vSpringStructures.clear();

	// Also reset the cannon position:
	cannon.reset();

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

	case SPRINGS_DEMO:
		cout << "Switch to Springs demo !" << endl;
		setupSpringsDemo();
		break;

	case COLLISIONS_DEMO:
		cout << "Switch to Collision debug: See the collision point returned by SAT !" <<endl;
		break;
	}
}

// Called each time the mouse is moved while pressed:
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed){}

void RigidBodySystemSimulator::simulateTimestep(float timestep) {
	// update current setup for each frame
	cannon.update(timestep);

	switch (m_iTestCase)
	{
	case SPRINGS_DEMO:
	case TEST_DEMO:
	case ANGRY_BIRDS_DEMO:
		for (auto& s : m_vSpringStructures)
			s.updateForces();

		for (Rigidbody* r : m_vRigidbodies)
			r->timestepEuler(timestep);

		manageCollisions(timestep);
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
	return m_vRigidbodies.at(i)->getPosition();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	return m_vRigidbodies.at(i)->getLinearVelocity();
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	return m_vRigidbodies.at(i)->getAngularVelocity();
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	m_vRigidbodies.at(i)->addTorque(loc, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {
	m_vRigidbodies.push_back(new Rigidbody(&m_SimulationParameters, mass, position, Vec3(0.0), size));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	m_vRigidbodies.at(i)->setRotation(orientation);
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
	m_vRigidbodies.at(i)->setLinearVelocity(velocity);
}


void RigidBodySystemSimulator::setupTestDemo()
{
	Rigidbody* ground = new Rigidbody(&m_SimulationParameters, 1, Vec3(0, -0.05, 0), Vec3(0, 0, 0), Vec3(10, 0.1, 10));
	ground->setKinematic(true);
	m_vRigidbodies.push_back(ground);
}

void RigidBodySystemSimulator::setupAngryBirdsDemo() {

	// Instantiate the kinematic parts of the scene:
	const Vec3 kinematicPos[] = { 
		Vec3(0, -0.05, 0), Vec3(-4, 0.75, -2), Vec3(-3.25, 0.75, -2), Vec3(-2.6, 0.35, -2),
		Vec3(-0.6, 1, -1), Vec3(-0.6, 1.95, -1.35), Vec3(-0.6, 1.95, -1.7), Vec3(0.4, 0.5, -2.6),
		Vec3(1.6, 0.5, -2.6), Vec3(1.6, 0.5, -1.4), Vec3(0.4, 0.5, -1.4), Vec3(1.5, 1, -0.5), 
		Vec3(1.5, 1, 2), Vec3(1.5, 2.05, 0.75) };

	const Vec3 kinematicScale[] = {
		Vec3(10, 0.1, 10), Vec3(0.1, 1.5, 0.6), Vec3(1.4, 0.1, 0.6), Vec3(0.1, 0.7, 0.6),
		Vec3(0.1, 2, 0.1), Vec3(0.1, 0.1, 0.8), Vec3(1.0, 0.1, 0.6), Vec3(0.1, 1, 0.1),
		Vec3(0.1, 1, 0.1), Vec3(0.1, 1, 0.1), Vec3(0.1, 1, 0.1), Vec3(0.1, 2, 0.1), 
		Vec3(0.1, 2, 0.1), Vec3(0.1, 0.1, 2.6)	};
	
	const int KINEMATIC_COUNT = sizeof(kinematicPos) / sizeof(Vec3);	// = 14
	for (int i = 0; i < KINEMATIC_COUNT; i++) {
		Rigidbody* kinematic = new Rigidbody(&m_SimulationParameters, 1, kinematicPos[i], 0.0, kinematicScale[i]);
		kinematic->setKinematic(true);
		m_vRigidbodies.push_back(kinematic);
	}

	// Instantiate the planks:
	const Vec3 planksPos[] = {
		Vec3(-3.45, 1.55, -2), Vec3(-2.9, 1.15, -2), Vec3(-1.5, 0.4, -2), Vec3(-2.05, 0.85, -2),
		Vec3(-3.8, 1.8, -2), Vec3(-3.45, 2.05, -2), Vec3(-3.1, 1.8, -2), Vec3(-0.95, 0.2, -2),
		Vec3(-0.6, 0.45, -2), Vec3(-0.25, 0.2, -2), Vec3(0, 0.4, 1.45), Vec3(0, 0.9, 0.75),
		Vec3(0, 0.4, 0.05), Vec3(0, 1.2, 1.1), Vec3(0, 1.45, 0.75), Vec3(0, 1.2, 0.4),
		Vec3(-0.6, 1.3, -1.7), Vec3(1, 0.4, -2), Vec3(1.5, 1, 0.05), Vec3(1.5, 1, 0.75),
		Vec3(1.5, 1, 1.45)};

	const Vec3 planksScale[] = {
			Vec3(1.2, 0.1, 0.6), Vec3(0.1, 0.7, 0.6), Vec3(0.1, 0.8, 0.6), Vec3(1.2, 0.1, 0.6),
			Vec3(0.1, 0.4, 0.6), Vec3(1.0, 0.1, 0.6), Vec3(0.1, 0.4, 0.6), Vec3(0.1, 0.4, 0.6),
			Vec3(1.0, 0.1, 0.6), Vec3(0.1, 0.4, 0.6), Vec3(0.6, 0.8, 0.2), Vec3(0.6, 0.2, 2.0),
			Vec3(0.6, 0.8, 0.2), Vec3(0.6, 0.4, 0.1), Vec3(0.6, 0.1, 1.0), Vec3(0.6, 0.4, 0.1),
			Vec3(0.8, 0.1, 0.4), Vec3(1.0, 0.1, 1.0), Vec3(0.05, 1.5, 0.4), Vec3(0.05, 1.5, 0.4), 
			Vec3(0.05, 1.5, 0.4) };

	const int PLANKS_COUNT = sizeof(planksPos) / sizeof(Vec3);		// = 21
	for (int i = 0; i < PLANKS_COUNT; i++) {
		Rigidbody* plank = new Rigidbody(&m_SimulationParameters, 1, planksPos[i], 0.0, planksScale[i]);
		plank->color = Vec3(120, 57, 0) / 255.0f;
		plank->setForce(Vec3(0, -m_fGravity, 0));
		m_vRigidbodies.push_back(plank);
	}

	// Instantiate the piggies:
	const Vec3 piggiesPos[] = {
		Vec3(-3.7, 0.9, -2), Vec3(-3.2, 0.9, -2), Vec3(-3.45, 1.7, -2), Vec3(-3.45, 2.2, -2),
		Vec3(-2.3, 1, -2), Vec3(-1.8, 1, -2), Vec3(-2, 0.1, -2), Vec3(-0.6, 0.6, -2),
		Vec3(1, 1.2, -2), Vec3(-0.6, 1.45, -1.7), Vec3(0, 1.6, 0.75) };

	const int PIGGIES_COUNT = sizeof(piggiesPos) / sizeof(Vec3);		// = 11
	for (int i = 0; i < PIGGIES_COUNT; i++) {
		Rigidbody* pig = new Rigidbody(&m_SimulationParameters, 1, piggiesPos[i], 0.0, Vec3(0.2));
		pig->color = Vec3(0, 120, 0) / 255.0f;
		pig->setForce(Vec3(0, -m_fGravity, 0));
		m_vRigidbodies.push_back(pig);
	}

	// Instantiate the spring structures:
	const float SPRINGS_STIFFNESS = 20;

	// First structure: pendulum
	SpringStructure structure = SpringStructure();
	structure.setExternalForce(Vec3(0, -m_fGravity, 0));
	structure.addRigidbody(m_vRigidbodies[6]);						// kinematic[6]
	structure.addRigidbody(m_vRigidbodies[KINEMATIC_COUNT + 16]);	// plank[16]
	
	float initialLength = 0.4;
	structure.addSpring(0, 1, Vec3(-0.5, -0.5, -0.5), Vec3(-0.5, 0.5, -0.5), initialLength, SPRINGS_STIFFNESS);
	structure.addSpring(0, 1, Vec3(-0.5, -0.5, +0.5), Vec3(-0.5, 0.5, +0.5), initialLength, SPRINGS_STIFFNESS);
	structure.addSpring(0, 1, Vec3(+0.5, -0.5, -0.5), Vec3(+0.5, 0.5, -0.5), initialLength, SPRINGS_STIFFNESS);
	structure.addSpring(0, 1, Vec3(+0.5, -0.5, +0.5), Vec3(+0.5, 0.5, +0.5), initialLength, SPRINGS_STIFFNESS);

	m_vSpringStructures.push_back(structure);

	// Second structure: trampoline
	structure = SpringStructure();
	structure.setExternalForce(Vec3(0, -m_fGravity, 0));
	structure.addRigidbody(m_vRigidbodies[KINEMATIC_COUNT + 17]);	// plank[17]
	structure.addRigidbody(m_vRigidbodies[7]);						// kinematic[7]
	structure.addRigidbody(m_vRigidbodies[8]);						// kinematic[8]
	structure.addRigidbody(m_vRigidbodies[9]);						// kinematic[9]
	structure.addRigidbody(m_vRigidbodies[10]);						// kinematic[10]

	structure.addSpring(0, 1, Vec3(-0.5, 0.5, -0.5), Vec3(0, 0.5, 0), initialLength, SPRINGS_STIFFNESS);
	structure.addSpring(0, 2, Vec3(+0.5, 0.5, -0.5), Vec3(0, 0.5, 0), initialLength, SPRINGS_STIFFNESS);
	structure.addSpring(0, 3, Vec3(+0.5, 0.5, +0.5), Vec3(0, 0.5, 0), initialLength, SPRINGS_STIFFNESS);
	structure.addSpring(0, 4, Vec3(-0.5, 0.5, +0.5), Vec3(0, 0.5, 0), initialLength, SPRINGS_STIFFNESS);

	m_vSpringStructures.push_back(structure);

	// Third structure: wall of planks:
	structure = SpringStructure();
	structure.setExternalForce(Vec3(0, -m_fGravity, 0));
	structure.addRigidbody(m_vRigidbodies[13]);						// kinematic[13]
	structure.addRigidbody(m_vRigidbodies[KINEMATIC_COUNT + 18]);	// plank[18]
	structure.addRigidbody(m_vRigidbodies[KINEMATIC_COUNT + 19]);	// plank[19]
	structure.addRigidbody(m_vRigidbodies[KINEMATIC_COUNT + 20]);	// plank[20]

	initialLength = 0.05;
	structure.addSpring(0, 1, Vec3(0, 0, -0.27), Vec3(0, 0.5, 0), initialLength, 40);
	structure.addSpring(0, 2, Vec3(0.0), Vec3(0, 0.5, 0), initialLength, 40);
	structure.addSpring(0, 3, Vec3(0, 0, +0.27), Vec3(0, 0.5, 0), initialLength, 40);
	
	m_vSpringStructures.push_back(structure);
}

/* OLD VERSION :
void RigidBodySystemSimulator::setupAngryBirdsDemo() {
	Rigidbody* ground = new Rigidbody(&m_SimulationParameters, 1, Vec3(0, -0.05, 0), Vec3(0, 0, 0), Vec3(10, 0.1, 10));
	ground->setKinematic(true);
	m_vRigidbodies.push_back(ground);

	const float mass = 1;

	// For testing: create a kind of angry birds game:
	Rigidbody* planks[10]{
		new Rigidbody(&m_SimulationParameters, mass, Vec3(-1, 0.1, 0), Vec3(0.0), Vec3(0.1, 0.2, 0.1)),
		new Rigidbody(&m_SimulationParameters, mass, Vec3(0, 0.1, 0), Vec3(0.0), Vec3(0.1, 0.2, 0.1)),
		new Rigidbody(&m_SimulationParameters, mass, Vec3(-1, 0.5, 0), Vec3(0.0), Vec3(0.1, 0.4, 0.1)),
		new Rigidbody(&m_SimulationParameters, mass, Vec3(0, 0.5, 0), Vec3(0.0), Vec3(0.1, 0.4, 0.1)),
		new Rigidbody(&m_SimulationParameters, mass, Vec3(0.6, 0.4, 0), Vec3(0.0), Vec3(0.1, 0.4, 0.1)),
		new Rigidbody(&m_SimulationParameters, mass, Vec3(1.1, 0.4, 0), Vec3(0.0), Vec3(0.1, 0.4, 0.1)),
		new Rigidbody(&m_SimulationParameters, mass, Vec3(0.6, 0.65, 0), Vec3(0.0), Vec3(0.4, 0.1, 0.1)),
		new Rigidbody(&m_SimulationParameters, mass, Vec3(1.275, 0.65, 0), Vec3(0.0), Vec3(0.65, 0.1, 0.1)),
		new Rigidbody(&m_SimulationParameters, mass, Vec3(-0.5, 0.25, 0), Vec3(0.0), Vec3(1.2, 0.1, 0.1)),
		new Rigidbody(&m_SimulationParameters, mass, Vec3(-0.5, 0.75, 0), Vec3(0.0), Vec3(1.2, 0.1, 0.1)),
	};
	
	for (int i = 0; i < 10; i++) {
		planks[i]->color = Vec3(120, 57, 0) / 255.0f;
		planks[i]->setForce(Vec3(0, -m_fGravity, 0));
		m_vRigidbodies.push_back(planks[i]);
	}
	
	Rigidbody* ground1 = new Rigidbody(&m_SimulationParameters, mass, Vec3(1, 0.1, 0), Vec3(0.0), Vec3(1.2, 0.2, 0.1));
	Rigidbody* ground2 = new Rigidbody(&m_SimulationParameters, mass, Vec3(1.5, 0.4, 0), Vec3(0.0), Vec3(0.2, 0.4, 0.1));
	ground1->setKinematic(true);
	ground2->setKinematic(true);
	m_vRigidbodies.push_back(ground1);
	m_vRigidbodies.push_back(ground2);

	Rigidbody* pigs[5]{
		new Rigidbody(&m_SimulationParameters, mass, Vec3(-0.75, 0.4, 0), Vec3(0.0), Vec3(0.2, 0.2, 0.2)),
		new Rigidbody(&m_SimulationParameters, mass, Vec3(-0.25, 0.4, 0), Vec3(0.0), Vec3(0.2, 0.2, 0.2)),
		new Rigidbody(&m_SimulationParameters, mass, Vec3(-0.5, 0.9, 0), Vec3(0.0), Vec3(0.2, 0.2, 0.2)),
		new Rigidbody(&m_SimulationParameters, mass, Vec3(0.6, 0.8, 0), Vec3(0.0), Vec3(0.2, 0.2, 0.2)),
		new Rigidbody(&m_SimulationParameters, mass, Vec3(1.2, 0.8, 0), Vec3(0.0), Vec3(0.2, 0.2, 0.2)),
	};

	for (int i = 0; i < 5; i++) {
		pigs[i]->color = Vec3(0, 120, 0) / 255.0f;
		pigs[i]->setForce(Vec3(0, -m_fGravity, 0));
		m_vRigidbodies.push_back(pigs[i]);
	}
}*/

void RigidBodySystemSimulator::setupSpringsDemo()
{
	// Create the ground:
	Rigidbody* ground = new Rigidbody(&m_SimulationParameters, 1, Vec3(0, -0.05, 0), Vec3(0, 0, 0), Vec3(10, 0.1, 10));
	ground->setKinematic(true);
	m_vRigidbodies.push_back(ground);

	// Create the platform:
	Rigidbody* platform = new Rigidbody(&m_SimulationParameters, 1, Vec3(0, 1.5, 0), Vec3(20, 45, 10), Vec3(2.4, 0.1, 2.4));
	platform->color = Vec3(120, 57, 0) / 255.0f;
	m_vRigidbodies.push_back(platform);

	// Create pillars:
	Rigidbody* pillar1 = new Rigidbody(&m_SimulationParameters, 1, Vec3(-1.5, 1, -1.5), Vec3(0, 0, 0), Vec3(0.1, 2, 0.1));
	Rigidbody* pillar2 = new Rigidbody(&m_SimulationParameters, 1, Vec3(-1.5, 1, +1.5), Vec3(0, 0, 0), Vec3(0.1, 2, 0.1));
	Rigidbody* pillar3 = new Rigidbody(&m_SimulationParameters, 1, Vec3(+1.5, 1, -1.5), Vec3(0, 0, 0), Vec3(0.1, 2, 0.1));
	Rigidbody* pillar4 = new Rigidbody(&m_SimulationParameters, 1, Vec3(+1.5, 1, +1.5), Vec3(0, 0, 0), Vec3(0.1, 2, 0.1));

	pillar1->setKinematic(true);
	pillar2->setKinematic(true);
	pillar3->setKinematic(true);
	pillar4->setKinematic(true);

	m_vRigidbodies.push_back(pillar1);
	m_vRigidbodies.push_back(pillar2);
	m_vRigidbodies.push_back(pillar3);
	m_vRigidbodies.push_back(pillar4);

	// Create a spring structure to contain the pillars, the platform and the springs between them:
	SpringStructure structure = SpringStructure();
	structure.setExternalForce(Vec3(0, -m_fGravity, 0));

	structure.addRigidbody(platform);
	structure.addRigidbody(pillar1);
	structure.addRigidbody(pillar2);
	structure.addRigidbody(pillar3);
	structure.addRigidbody(pillar4);

	float stiffness = 20;
	float initialLength = 1;
	structure.addSpring(0, 1, Vec3(-0.5, 0, -0.5), Vec3(0, 0.5, 0), initialLength, stiffness);
	structure.addSpring(0, 2, Vec3(-0.5, 0, +0.5), Vec3(0, 0.5, 0), initialLength, stiffness);
	structure.addSpring(0, 3, Vec3(+0.5, 0, -0.5), Vec3(0, 0.5, 0), initialLength, stiffness);
	structure.addSpring(0, 4, Vec3(+0.5, 0, +0.5), Vec3(0, 0.5, 0), initialLength, stiffness);

	m_vSpringStructures.push_back(structure);
}

void RigidBodySystemSimulator::manageCollisions(double timestep)
{
	// First, find the collisions between all the rigidbodies:
	vector<Collision> collisions;
	for (int i = 0; i < m_vRigidbodies.size(); i++) {
		for (int j = i + 1; j < m_vRigidbodies.size(); j++) {
			Rigidbody* r1 = m_vRigidbodies[i];
			Rigidbody* r2 = m_vRigidbodies[j];

			// If both objects are idle, we don't need to recompute the 
			// collision between them (it will be the same that in 
			// the previous frame) !
			if (r1->isIdle() && r2->isIdle())
				continue;

			// Else, we compute the collision between the two objects:
			CollisionInfo collision = Rigidbody::computeCollision(r1, r2);

			if (collision.isValid) {
				collisions.push_back(Collision(i, j, collision));
				r1->addCollider(r2);
				r2->addCollider(r1);
			}
		}
	}

	// Recompute which rigidbodies are in idle state or not. A rigidbody 
	// can stay in idle state only if all the rigidbodies colliding it 
	// are also in idle state:
	for (Rigidbody* r : m_vRigidbodies)
		r->checkKeepIdleState();

	// Now we can compute the impulse of each collision, but there is no need to compute impulses
	// between rigidbodies in idle state:
	for (int i = 0; i < collisions.size(); i++) {
		Rigidbody* r1 = m_vRigidbodies[collisions[i].i1];
		Rigidbody* r2 = m_vRigidbodies[collisions[i].i2];

		if (!r1->isIdle() || !r2->isIdle()) {
			manageCollision(r1, r2, &collisions[i], timestep);
		}
	}

	// Check which rigidbodies are still moving after the impulse:
	for (Rigidbody* r : m_vRigidbodies)
		r->checkIsMooving();

	for (Rigidbody* r : m_vRigidbodies)
		r->checkEnterIdleState();

	// Finally, update the colliders of the rigidbodies:
	for (Rigidbody* r : m_vRigidbodies)
		r->updateColliders();
}

void RigidBodySystemSimulator::manageCollision(Rigidbody* r1, Rigidbody* r2, const Collision* collision, double timestep)
{
	double J = Rigidbody::computeImpulse(r1, r2, m_SimulationParameters.collisionFactor,
		collision->collisionPoint, collision->collisionNormal);

	if (J < m_SimulationParameters.minimumImpulse) {
		if (!r1->isKinematic() && !r2->isKinematic())
			Rigidbody::correctPosition(r1, r2, &m_SimulationParameters, collision->collisionPoint, 
				-collision->collisionNormal, collision->collisionDepth, timestep);

		else if (!r1->isKinematic())
			Rigidbody::correctPosition(r1, &m_SimulationParameters, collision->collisionPoint, 
				collision->collisionNormal, collision->collisionDepth, timestep);

		else if (!r2->isKinematic())
			Rigidbody::correctPosition(r2, &m_SimulationParameters, collision->collisionPoint, 
				-collision->collisionNormal, collision->collisionDepth, timestep);
	}
}

void RigidBodySystemSimulator::fireRigidbody()
{
	// Use different scenarios to test the simulator in different situations:
	
	if (m_iTestScenario == 0) {		// Fire rigidbodies from the cannon
		Rigidbody* bullet = cannon.fireRigidbody(&m_SimulationParameters);
		bullet->addForce(Vec3(0, -m_fGravity, 0));
		m_vRigidbodies.push_back(bullet);
	}

	else if (m_iTestScenario == 1) {	// Small objects with no velocity
		Rigidbody* box = new Rigidbody(&m_SimulationParameters, 1, Vec3(0.5, 0.5, 0), Vec3(10, 45, 10), Vec3(0.04, 0.1, 0.02));
		box->color = Vec3(1, 1, 0);
		box->setForce(Vec3(0, -m_fGravity, 0));
		m_vRigidbodies.push_back(box);
	}

	else if (m_iTestScenario == 2) {	// Throw spinning boxes
		Rigidbody* box = new Rigidbody(&m_SimulationParameters, 1, Vec3(0.5, 0.5, 0), Vec3(0.0), Vec3(0.2, 0.04, 0.2));
		box->color = Vec3(1, 1, 0);
		box->setForce(Vec3(0, -m_fGravity, 0));
		box->setLinearVelocity(Vec3(-10, 0, 0));
		box->setAngularVelocity(Vec3(0, 80, 0));
		m_vRigidbodies.push_back(box);
	}
	
	else if (m_iTestScenario == 3) {	// Stack boxes to make a tower
		Rigidbody* box = new Rigidbody(&m_SimulationParameters, 1, Vec3(0, 2, 0), Vec3(0.0), Vec3(1, 0.1, 1));
		box->color = Vec3(1, 1, 0);
		box->setForce(Vec3(0, -m_fGravity, 0));
		box->setAngularVelocity(Vec3(0, 80, 0));
		m_vRigidbodies.push_back(box);
	}

	else if (m_iTestScenario == 4) {	// For angry birds
		Rigidbody* bird = new Rigidbody(&m_SimulationParameters, 1, Vec3(-3, 0.5, 0), Vec3(0.0), Vec3(0.2, 0.2, 0.2));
		bird->color = Vec3(1, 1, 0);
		bird->setForce(Vec3(0, -m_fGravity, 0));
		bird->setLinearVelocity(Vec3(10, 2, 0));
		m_vRigidbodies.push_back(bird);
	}

	else if (m_iTestScenario == 5) {	// Throw boxes without velocity above the origin
		Vec3 position(0.4 * randFloat(eng) - 0.2, 5, 0.4 * randFloat(eng) - 0.2);
		Vec3 rotation(90 * randFloat(eng), 90 * randFloat(eng), 90 * randFloat(eng));

		Rigidbody* box = new Rigidbody(&m_SimulationParameters, 1, position, rotation, Vec3(0.5));
		box->color = Vec3(0, 0, 1);
		box->setForce(Vec3(0, -m_fGravity, 0));
		m_vRigidbodies.push_back(box);
	}
}
