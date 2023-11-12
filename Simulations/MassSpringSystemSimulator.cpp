#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator(){
	// Data Attributes
	m_fMass = 10.0;
	m_fStiffness = 1.0;
	m_fDamping = 1.0;
	m_iIntegrator = 1;
	m_externalForce = Vec3(0, 0, 0);

	// UI Attributes
	m_externalForce;
	m_mouse;
	m_trackmouse;
	m_oldtrackmouse;
}

const char* MassSpringSystemSimulator::getTestCasesStr(){
	return "Demo 1: simple one-step,Demo 2: simple Euler simulation,Demo 3: simple Midpoint simulation,Demo 4: complex simulation - compare the stability of Euler and Midpoint method,Optional Demo 5: additionally implement the Leap-Frog method";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC){
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:break;
	case 1:break;
	case 2:break;
	case 3:
		TwAddVarRW(DUC->g_pTweakBar, "Euler", TW_TYPE_BOOL8, &m_bModeEuler, "");
		TwAddVarRW(DUC->g_pTweakBar, "Midpoint", TW_TYPE_BOOL8, &m_bModeMidpoint, "");
		TwAddVarRW(DUC->g_pTweakBar, "Masspoints", TW_TYPE_INT32, &m_iNumberOfWantedMasspoints, "min=10 max=100 step=1");
		break;
	default:break;
	}
}

void MassSpringSystemSimulator::reset(){
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	// stephans edit
	executed = false;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext){
	Vec3 white = Vec3(1, 1, 1);
	Vec3 red = Vec3(1, 0, 0);
	Vec3 green = Vec3(0, 1, 0);
	Vec3 blue = Vec3(0, 0, 1);
	switch (m_iTestCase)
	{
	case 0: // single timestep
	case 1: // euler simulation
	case 2: // midpoint simulation
	case 3: // complex simulation
		for (auto& p : m_vPoints) {
			DUC->drawSphere(p.m_vPosition, size_of_ball * p.m_fMass);
			DUC->beginLine();
			DUC->drawLine(p.m_vPosition, green, p.m_vPosition + p.m_vVelocity, green);
			DUC->drawLine(p.m_vPosition, red, p.m_vPosition + p.m_vForce, red);
			DUC->endLine();
		}
		for (auto& s : m_vSprings) {
			DUC->beginLine();
			DUC->drawLine(m_vPoints.at(s.m_pPoints.first).m_vPosition, white, m_vPoints.at(s.m_pPoints.second).m_vPosition, white);
			DUC->endLine();
		}
		break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase){
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0: // single timestep
		initTable1();
		cout << "Demo 1, a simple one-step test - HINT: hardcoded for 0.005s timestep\n";
		for (auto& p : m_vPoints) {
			cout << p.to_string();
		}
		cout << "timestep_euler(0.005)\n";
		timestep_euler(0.005);
		for (auto& p : m_vPoints) {
			cout << p.to_string();
		}
		initTable1();
		cout << "timestep_midpoint(0.005)\n";
		timestep_midpoint(0.005);
		for (auto& p : m_vPoints) {
			cout << p.to_string();
		}
		break;

	case 1: // euler simulation
		initTable1();
		cout << "Demo 2: a simple Euler simulation - HINT: Please set timestep manually to 0.005 for this Demo\n";
		for (auto& p : m_vPoints) {
			cout << p.to_string();
		}
		break;

	case 2: // midpoint simulation
		initTable1();
		cout << "Demo 3: a simple Midpoint simulation - HINT: Please set timestep manually to 0.005 for this Demo\n";
		for (auto& p : m_vPoints) {
			cout << p.to_string();
		}
		break;

	case 3: // complex simulation
		initDemo4();
		cout << "Demo 4: a complex simulation, compare the stability of Euler and Midpoint method\n";
		for (auto& p : m_vPoints) {
			cout << p.to_string();
		}
		break;

	case 4:
		cout << "Optional Demo 5: additionally implement the Leap-Frog method\n";
		for (auto& p : m_vPoints) {
			cout << p.to_string();
		}
		break;

	default:
		cout << "Empty Test!\n";
		break;
	}
}

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
		break;

	case 1: // Euler simulation
		timestep_euler(timeStep);
		addBoundaries();
		break;

	case 2: // Midpoint simulation
		timestep_midpoint(timeStep);
		addBoundaries();
		break;
	case 3: // Complex simulations
		if (m_bModeEuler && m_bModeMidpoint) {
			cout << "Please select only one method!\n";
		}
		if (m_bModeEuler) {
			timestep_euler(timeStep);
		}
		if (m_bModeMidpoint) {
			timestep_midpoint(timeStep);
		}
		addBoundaries();
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y){
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y){
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass){
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness){
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping){
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed){
	Point& p = Point();
	p.m_vPosition = position;
	p.m_vVelocity = velocity;
	p.m_bFixed = isFixed;
	
	p.m_fMass = m_fMass;
	p.m_vAcceleration = Vec3(0, 0, 0);
	p.m_vForce = Vec3(0, 0, 0);

	m_vPoints.push_back(p);
	return m_vPoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength){
	Spring& s = Spring();
	s.m_pPoints = pair<int, int>(masspoint1, masspoint2);
	s.m_fInitialLength = initialLength;

	s.m_fStiffness = m_fStiffness;
	s.m_fDamping = m_fDamping;

	m_vSprings.push_back(s);
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

}

void MassSpringSystemSimulator::initTable1(){
	m_vPoints.clear();
	m_vSprings.clear();

	Vec3& p_0 = Vec3(0, 0, 0);
	Vec3& p_1 = Vec3(0, 1, 0);
	Vec3& v_0 = Vec3(-1, 0, 0);
	Vec3& v_1 = Vec3(1, 0, 0);
	
	addMassPoint(p_0, v_0, false);
	addMassPoint(p_1, v_1, false);
	m_vPoints.at(0).m_fMass = 10;
	m_vPoints.at(1).m_fMass = 10;

	addSpring(0, 1, 1);
	m_vSprings.at(0).m_fStiffness = 40;
}

void MassSpringSystemSimulator::initDemo4(){
	m_vPoints.clear();
	m_vSprings.clear();

	std::mt19937 eng;
	std::uniform_real_distribution<float> randLen(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-1.0f, 1.0f);

	int i = addMassPoint(
		Vec3(randPos(eng), randPos(eng), randPos(eng)),
		Vec3(randPos(eng), randPos(eng), randPos(eng)),
		false);

	for (; i < m_iNumberOfWantedMasspoints; ++i) {
		addSpring(
			i,
			addMassPoint(
				Vec3(randPos(eng), randPos(eng), randPos(eng)),
				Vec3(randPos(eng), randPos(eng), randPos(eng)),
				false),
			randLen(eng));
		m_vPoints.at(i).m_fMass = m_fMass;
	}
	addSpring(0, getNumberOfMassPoints() - 1, randLen(eng));
}

float MassSpringSystemSimulator::distance(Point p1, Point p2){
	Vec3& pos1 = p1.m_vPosition;
	Vec3& pos2 = p2.m_vPosition;
	Vec3 diff = pos1 - pos2;
	return sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
}

void MassSpringSystemSimulator::timestep_euler(float timeStep){
	for (auto& p : m_vPoints) {
		p.m_vPosition += timeStep * p.m_vVelocity;
	}
	for (auto& s : m_vSprings) {
		Point& p1 = m_vPoints.at(s.m_pPoints.first);
		Point& p2 = m_vPoints.at(s.m_pPoints.second);
		float d = distance(p1, p2);

		Vec3 force = -s.m_fStiffness * (d - s.m_fInitialLength) * (p1.m_vPosition - p2.m_vPosition) / d;
		p1.m_vForce = force;
		p2.m_vForce = -force;

		addGravity();

		p1.m_vAcceleration = p1.m_vForce / p1.m_fMass;
		p2.m_vAcceleration = p2.m_vForce / p2.m_fMass;

		p1.m_vVelocity += timeStep * p1.m_vAcceleration;
		p2.m_vVelocity += timeStep * p2.m_vAcceleration;

		p1.m_vPosition += timeStep * p1.m_vVelocity;
		p2.m_vPosition += timeStep * p2.m_vVelocity;
	}
}

void MassSpringSystemSimulator::timestep_midpoint(float timeStep){
	timestep_euler(timeStep / 2);
	timestep_euler(timeStep / 2);
}

void MassSpringSystemSimulator::addBoundaries(){
	float max = 1;
	for (Point& p : m_vPoints) {
		if (p.m_vPosition.x > max*5 || p.m_vPosition.x < -max*5){
			p.m_vVelocity.x *= -0.5;
		}
		if (p.m_vPosition.y > max*5 || p.m_vPosition.y < -max) {
			p.m_vVelocity.y *= -0.5;
		}
		if (p.m_vPosition.z > max*5 || p.m_vPosition.z < -max*5) {
			p.m_vVelocity.z *= -0.5;
		}
	}
}

void MassSpringSystemSimulator::addGravity(){
	for (Point& p : m_vPoints) {
		p.m_vForce += Vec3(0, -9.81, 0);
	}
}

std::string MassSpringSystemSimulator::Point::to_string(){
	cout << "\tp: " << m_vPosition.toString()
		<< "\n\tv: " << m_vVelocity.toString()
		<< "\n\ta: " << m_vAcceleration.toString()
		<< "\n\tf: " << m_vForce.toString()
		<< "\n\tm:" << m_fMass << '\n';
	if (m_bFixed) {
		cout << "Fixed\n";
	}
	cout << '\n';
	return std::string();
}
