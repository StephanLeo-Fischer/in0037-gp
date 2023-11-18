#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator(){
	// Data Attributes
	m_fMass = 10.0;
	m_fStiffness = 40.0;
	m_fDamping = 0.0;
	m_iIntegrator = 1;

	// UI Attributes
	m_externalForce = Vec3(0, 0, 0);
	m_mouse = { 0, 0 };
	m_trackmouse = { 0, 0 };
	m_oldtrackmouse = { 0, 0 };
}

const char* MassSpringSystemSimulator::getTestCasesStr(){
	return 
		"Demo 1: one-step,"
		"Demo 2: Euler simulation,"
		"Demo 3: Midpoint simulation,"
		"Demo 4: complex simulation - compare the stability of Euler and Midpoint method,"
		"Demo 5: complex simulation with Leapfrog method only";
}

// Called when we reset the scene, or change the test case:
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC){
	this->DUC = DUC;
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", "Euler,Midpoint,LeapFrog");
	switch (m_iTestCase)
	{
	case 0:break;
	case 1:break;
	case 2:break;
	case 3:
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
	case 4:
		TwAddVarRW(DUC->g_pTweakBar, "Mass",       TW_TYPE_FLOAT,      &m_fMass,       "step=0.1  min=0.1");
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness",  TW_TYPE_FLOAT,      &m_fStiffness,  "step=0.1  min=0.1");
		TwAddVarRW(DUC->g_pTweakBar, "Damping",    TW_TYPE_FLOAT,      &m_fDamping,    "step=0.1  max=1 min=0");
		TwAddVarRW(DUC->g_pTweakBar, "Gravity",    TW_TYPE_BOOLCPP,    &m_bApplyExternalForces, "");
		TwAddVarRW(DUC->g_pTweakBar, "Bounce Factor", TW_TYPE_FLOAT, &m_fWallBounceFactor, "step=0.1  max=3 min=-1");
		break;
	default:break;
	}
}

// Called once, at the beginning of the program:
void MassSpringSystemSimulator::reset(){
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext){
	Vec3 white = Vec3(1, 1, 1);
	Vec3 red =   Vec3(1, 0, 0);
	Vec3 green = Vec3(0, 1, 0);
	Vec3 blue =  Vec3(0, 0, 1);

	// For Demo 1, 2 and 3, add a green line for the velocity and red line for the force:
	if (m_iTestCase < 3) {
		for (auto& p : m_vPoints) {
			DUC->beginLine();
			DUC->drawLine(p.m_vPosition, green, p.m_vPosition + p.m_vVelocity, green);	// velocity
			DUC->drawLine(p.m_vPosition, red, p.m_vPosition / 10 + p.m_vForce, red);	// force
			DUC->endLine();
		}
	}

	// For all the demos, draw the mass points and springs:
	for (auto& p : m_vPoints) {
		DUC->drawSphere(p.m_vPosition, size_of_ball * p.m_fMass);
	}

	for (auto& s : m_vSprings) {
		DUC->beginLine();
		DUC->drawLine(m_vPoints.at(s.m_pPoint1).m_vPosition, white, m_vPoints.at(s.m_pPoint2).m_vPosition, white);
		DUC->endLine();
	}
}

// Called when we reset the scene, or change the test case:
void MassSpringSystemSimulator::notifyCaseChanged(int testCase){
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0: // single timestep
		initTable1();
		cout << "Demo 1, a simple one-step test\n";
		for (auto& p : m_vPoints) {
			cout << p.to_string();
		}
		cout << "timestep_euler(0.1)\n";
		timestep_euler(0.1);
		for (auto& p : m_vPoints) {
			cout << p.to_string();
		}
		initTable1();
		cout << "timestep_midpoint(0.1)\n";
		timestep_midpoint(0.1);
		for (auto& p : m_vPoints) {
			cout << p.to_string();
		}
		break;

	case 1: // euler simulation
		initTable1();
		cout << "Demo 2: a simple Euler simulation - HINT: changing the timestep has no effect on the simulation\n";
		break;
	case 2: // midpoint simulation
		initTable1();
		cout << "Demo 3: a simple Midpoint simulation - HINT: changing the timestep has no effect on the simulation\n";
		break;
	case 3: // complex simulation
		initDemo4();
		cout << "Demo 4: a complex simulation, compare the stability of Euler and Midpoint method\n";
		break;
	case 4: // leapfrog simulation
		initDemo4();
		cout << "Demo 5: a complex simulation, with Leapfrog method only\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

// Called each time the mouse is moved while pressed:
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed){
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

void MassSpringSystemSimulator::simulateTimestep(float timeStep){
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0: // single timestep
		// This case requires only one update, and is thus handled by notifyCaseChanged()
		break;
	case 1: // Euler simulation
		timestep_euler(0.005);
		break;
	case 2: // Midpoint simulation
		timestep_midpoint(0.005);
		break;
	case 3: // Complex simulations
		switch (m_iIntegrator)
		{
		case 0: 
			timestep_euler(timeStep); 
			addBoundaries();
			break;
		case 1: 
			timestep_midpoint(timeStep); 
			addBoundaries();
			break;
		case 2: 
			timestep_leapfrog(timeStep);
			addBoundaries();
			break;
		}
		break;
	case 4: // Leapfrog simulation
		timestep_leapfrog(timeStep);
		addBoundaries();
		break;
	default:
		break;
	}
}

// This function is called when the mouse is clicked, 
// and when the mouse is moved while clicked:
void MassSpringSystemSimulator::onClick(int x, int y){
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// This function is called when the mouse moves, but not clicked:
void MassSpringSystemSimulator::onMouse(int x, int y){
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// Set the mass of the next mass points we will add using addMassPoint()
void MassSpringSystemSimulator::setMass(float mass){
	m_fMass = mass;
}

// Set the stiffness of the next springs we will add using addSpring()
void MassSpringSystemSimulator::setStiffness(float stiffness){
	m_fStiffness = stiffness;
}

// Set the damping factor of all the mass points:
void MassSpringSystemSimulator::setDampingFactor(float damping){
	m_fDamping = damping;
}

// Create a new mass point, add it to the list of points, and return the index of the point:
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed){
	m_vPoints.push_back(Point(position, velocity, isFixed, m_fMass));
	return m_vPoints.size() - 1;
}

// Create a new spring between two points, with a length equal to the distance between the points:
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2) {
	float initialLength = norm(getPositionOfMassPoint(masspoint1) - getPositionOfMassPoint(masspoint2));
	m_vSprings.push_back(Spring(masspoint1, masspoint2, initialLength, m_fStiffness));
}

// Create a new spring between two points, with the given inital length:
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength){
	m_vSprings.push_back(Spring(masspoint1, masspoint2, initialLength, m_fStiffness));
}

int MassSpringSystemSimulator::getNumberOfMassPoints(){
	return m_vPoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings(){
	return m_vSprings.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index){
	return m_vPoints.at(index).m_vPosition;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index){
	return m_vPoints.at(index).m_vVelocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force){
	m_bApplyExternalForces = true;
	m_externalForce = force;
}

void MassSpringSystemSimulator::initTable1(){
	m_vPoints.clear();
	m_vSprings.clear();

	setMass(10);							// Set the mass of the mass points
	setStiffness(40);						// Set the stiffness of the springs
	setDampingFactor(0);					// No damping
	applyExternalForce(Vec3(0, 0, 0));		// No external forces

	Vec3& p_0 = Vec3(0, 0, 0);
	Vec3& v_0 = Vec3(-1, 0, 0);
	Vec3& p_1 = Vec3(0, 2, 0);
	Vec3& v_1 = Vec3(1, 0, 0);
	
	addMassPoint(p_0, v_0, false);
	addMassPoint(p_1, v_1, false);

	addSpring(0, 1, 1);
}

void MassSpringSystemSimulator::initDemo4(){
	m_vPoints.clear();
	m_vSprings.clear();

	setMass(10);							// Set the mass of the mass points
	setStiffness(100);						// Set the stiffness of the springs
	setDampingFactor(1);					// Add damping
	applyExternalForce(Vec3(0, -2, 0));		// Add gravity

	// Create a cylinder:
	const Vec3 startPosition = Vec3(0, 3, 0);
	const Vec3 startSpeed = Vec3(0, 0, 0);
	const int rounds = 10;
	const float radius = 2;
	const float height = 1;

	// Create the mass points:
	addMassPoint(startPosition + Vec3(0, 0, -height / 2), startSpeed, false);
	addMassPoint(startPosition + Vec3(0, 0, +height / 2), startSpeed, false);

	for (int i = 0; i < rounds; i++) {
		double angle = 2 * i * M_PI / rounds;
		addMassPoint(
			startPosition + Vec3(radius * cos(angle), radius * sin(angle), -height / 2),
			startSpeed, false);

		addMassPoint(
			startPosition + Vec3(radius * cos(angle), radius * sin(angle),  height / 2),
			startSpeed, false);
	}

	// Springs between the points:
	addSpring(0, 1);
	for (int i = 2; i <= 2 * rounds; i += 2) {
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


void MassSpringSystemSimulator::resetForces() {
	if (m_bApplyExternalForces) {
		for (auto& p : m_vPoints)
			p.m_vForce = m_externalForce;
	}
	else {
		for (auto& p : m_vPoints)
			p.m_vForce = Vec3(0, 0, 0);
	}
}

void MassSpringSystemSimulator::calculateForces() {
	
	// Start by resetting the forces:
	resetForces();

	for (auto& s : m_vSprings) {
		Point& p1 = m_vPoints.at(s.m_pPoint1);
		Point& p2 = m_vPoints.at(s.m_pPoint2);

		// Vector from p2 to p1:
		Vec3 diff = p1.m_vPosition - p2.m_vPosition;

		float l = norm(diff);
		float L = s.m_fInitialLength;
		float k = m_fStiffness;

		// formula on the slide
		Vec3 f = -k * (l - L) * diff / l;

		p1.m_vForce += f;
		p2.m_vForce -= f;
	}
}

void MassSpringSystemSimulator::calculateDamping(){
	for (auto& p : m_vPoints) {
		p.m_vForce -= m_fDamping * p.m_vVelocity;
	}
}

void MassSpringSystemSimulator::updateCurrentPositions(float timeStep){
	for (auto& p : m_vPoints) {
		if (!p.m_bFixed) {
			p.m_vPosition += p.m_vVelocity * timeStep;
		}
	}
}

void MassSpringSystemSimulator::updateCurrentVelocities(float timeStep){
	for (auto& p : m_vPoints) {
		if (!p.m_bFixed) {
			p.m_vVelocity += p.m_vForce * (timeStep / p.m_fMass);
		}
		else {
			p.m_vVelocity = Vec3(0, 0, 0);
		}
	}
}

void MassSpringSystemSimulator::timestep_euler(float timeStep){
	// calc Forces
	calculateForces();

	// additional damping
	calculateDamping();
	
	// update current positions
	updateCurrentPositions(timeStep);

	// update current velocities
	updateCurrentVelocities(timeStep);
}

void MassSpringSystemSimulator::timestep_midpoint(float timeStep){
	std::vector<Vec3> old_pos;
	std::vector<Vec3> old_vel;
	for (auto& p : m_vPoints) {
		old_pos.push_back(p.m_vPosition);
		old_vel.push_back(p.m_vVelocity);
	}

	// calc Forces
	calculateForces();

	// update current positions /2
	updateCurrentPositions(timeStep / 2);

	// update current velocities /2
	updateCurrentVelocities(timeStep / 2);

	// calc Forces with new positions
	calculateForces();

	// restore old positions
	for (int i = 0; i < m_vPoints.size(); ++i) {
		m_vPoints.at(i).m_vPosition = old_pos.at(i);
	}

	// update current positions
	updateCurrentPositions(timeStep);

	// restore old velocities
	for (int i = 0; i < m_vPoints.size(); ++i) {
		m_vPoints.at(i).m_vVelocity = old_vel.at(i);
	}

	// update current velocities
	updateCurrentVelocities(timeStep);
}

void MassSpringSystemSimulator::timestep_leapfrog(float timeStep){
	// calc Forces
	calculateForces();

	// additional damping
	calculateDamping();

	// update current velocities
	updateCurrentVelocities(timeStep);

	// update current positions
	updateCurrentPositions(timeStep);
}

void MassSpringSystemSimulator::addBoundaries(){
	float max = 10;
	float floorHeight = -1;

	for (Point& p : m_vPoints) {
		if (p.m_vPosition.x > max) {  // x
			p.m_vPosition.x = max;
			p.m_vVelocity.x *= -1 * m_fWallBounceFactor;
		} else if (p.m_vPosition.x < -max) {
			p.m_vPosition.x = -max;
			p.m_vVelocity.x *= -1 * m_fWallBounceFactor;
		}
		if (p.m_vPosition.y > max) {  // y
			p.m_vPosition.y = max;
			p.m_vVelocity.y *= -1 * m_fWallBounceFactor;
		} else if (p.m_vPosition.y < floorHeight) {
			p.m_vPosition.y = -1;
			p.m_vVelocity.y *= -1 * m_fWallBounceFactor;
		}
		if (p.m_vPosition.z > max) {  // z
			p.m_vPosition.z = max;
			p.m_vVelocity.z *= -1 * m_fWallBounceFactor;
		} else if (p.m_vPosition.z < -max) {
			p.m_vPosition.z = -max;
			p.m_vVelocity.z *= -1 * m_fWallBounceFactor;
		}
	}
}

std::string MassSpringSystemSimulator::Point::to_string(){
	std::string s;
	s = "\tp: " + m_vPosition.toString()
		+ "\n\tv: " + m_vVelocity.toString()
		+ "\n\tf: " + m_vForce.toString() + '\n';
		//<< "\n\tm:" << m_fMass << '\n';
	if (m_bFixed) {
		s += "Fixed\n";
	}
	s += '\n';
	return s;
}
