#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator(){
	// Data Attributes
	m_fMass = 1.0;
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
	Vec3 blue = Vec3(0, 1, 0);
	Vec3 green = Vec3(0, 0, 1);
	switch (m_iTestCase)
	{
	case 0: // single timestep
		// draw spheres
		for (auto& p : m_vPoints_euler) {
			DUC->drawSphere(p.m_vPosition, size_of_ball * p.m_fMass);
		}
		for (auto& p : m_vPoints_midpoint) {
			DUC->drawSphere(p.m_vPosition, size_of_ball * p.m_fMass);
		}
		for (auto s : m_vSprings) {
			int point1_idx = s.m_pPoints.first;
			int point2_idx = s.m_pPoints.second;

			auto &p1_pos_euler = m_vPoints_euler.at(point1_idx).m_vPosition;
			auto &p2_pos_euler = m_vPoints_euler.at(point2_idx).m_vPosition;

			auto &p1_pos_midpoint = m_vPoints_midpoint.at(point1_idx).m_vPosition;
			auto &p2_pos_midpoint = m_vPoints_midpoint.at(point2_idx).m_vPosition;

			DUC->beginLine();
			DUC->drawLine(p1_pos_euler, blue, p2_pos_euler, blue);
			DUC->drawLine(p1_pos_midpoint, green, p2_pos_midpoint, green);
			DUC->endLine();
		}
			break;
	case 1: // euler simulation
		// draw spheres
		for (auto& p : m_vPoints_euler) {
			DUC->drawSphere(p.m_vPosition, size_of_ball * p.m_fMass);
		}

		// draw springs
		for (auto& s : m_vSprings) {
			int point1_idx = s.m_pPoints.first;
			int point2_idx = s.m_pPoints.second;

			auto &p1_pos_euler = m_vPoints_euler.at(point1_idx).m_vPosition;
			auto &p2_pos_euler = m_vPoints_euler.at(point2_idx).m_vPosition;

			DUC->beginLine();
			DUC->drawLine(p1_pos_euler, blue, p2_pos_euler, blue);
			DUC->endLine();
		}
		break;
	case 2: // midpoint simulation
		// draw spheres
		for (auto &p : m_vPoints_midpoint) {
			DUC->drawSphere(p.m_vPosition, size_of_ball * p.m_fMass);
		}

		// draw springs
		for (auto& s : m_vSprings) {
			int point1_idx = s.m_pPoints.first;
			int point2_idx = s.m_pPoints.second;

			auto &p1_pos_midpoint = m_vPoints_midpoint.at(point1_idx).m_vPosition;
			auto &p2_pos_midpoint = m_vPoints_midpoint.at(point2_idx).m_vPosition;

			DUC->beginLine();
			DUC->drawLine(p1_pos_midpoint, green, p2_pos_midpoint, green);
			DUC->endLine();
		}
		break;
	case 3: // complex simulation
		// draw spheres
		for (auto &p : *m_vPoints) {
			DUC->drawSphere(p.m_vPosition, size_of_ball * p.m_fMass);
		}

		// draw springs
		for (auto& s : m_vSprings) {
			int point1_idx = s.m_pPoints.first;
			int point2_idx = s.m_pPoints.second;

			auto &&p1_pos = m_vPoints->at(point1_idx).m_vPosition;
			auto &&p2_pos = m_vPoints->at(point2_idx).m_vPosition;

			auto force = (m_vPoints->at(point1_idx).m_vForce + m_vPoints->at(point2_idx).m_vForce);
			float redly_color = 0;
			redly_color = force.x * force.x;
			redly_color = force.y * force.y;
			redly_color = force.z * force.z;
			redly_color = sqrt(redly_color);
			Vec3 red_color = Vec3(1, 1 - redly_color, 1 - redly_color);

			DUC->beginLine();
			DUC->drawLine(p1_pos, red_color, p2_pos, red_color);
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
		cout << "Demo 1, a simple one-step test - HINT: Please set timestep manually to 0.005 for this Demo\n"
			<< "Initiate Table 1.1:\n";
		for (auto& p : m_vPoints_euler) {
			cout << p.to_string();
		}
		break;

	case 1: // euler simulation
		initTable1();
		cout << "Demo 2: a simple Euler simulation - HINT: Please set timestep manually to 0.005 for this Demo\n"
			<< "Initiate Table 1.1:\n";
		for (auto& p : m_vPoints_euler) {
			cout << p.to_string();
		}
		break;

	case 2: // midpoint simulation
		initTable1();
		cout << "Demo 3: a simple Midpoint simulation - HINT: Please set timestep manually to 0.005 for this Demo\n";
		cout << "Initiate Table 1.1:\n";
		for (auto& p : m_vPoints_midpoint) {
			cout << p.to_string();
		}
		break;

	case 3: // complex simulation
		initDemo4();
		cout << "Demo 4: a complex simulation, compare the stability of Euler and Midpoint method\n";
		for (auto p : *m_vPoints) {
			cout << p.to_string();
		}
		break;

	case 4:
		cout << "Optional Demo 5: additionally implement the Leap-Frog method\n";
		for (auto& p : m_vPoints_euler) {
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
		if (!executed) {
			timestep_euler(timeStep);
			cout << "After execution of a single Euler timestep:\n";
			for (auto p : m_vPoints_euler) {
				p.to_string();
			}

			timestep_midpoint(timeStep);
			cout << "After execution of a single Midpoint timestep:\n";
			for (auto p : m_vPoints_midpoint) {
				p.to_string();
			}
			executed = true; // for single timestep
		}
		break;

	case 1: // Euler simulation
		timestep_euler(timeStep);
		for (auto p : m_vPoints_euler) {
			cout << p.to_string();
		}
		break;

	case 2: // Midpoint simulation
		timestep_midpoint(timeStep);
		for (auto p : m_vPoints_midpoint) {
			cout << p.to_string();
		}
		break;
	case 3: // Complex simulations
		if (m_bModeEuler && m_bModeMidpoint) {
			if(m_vPoints == &m_vPoints_euler){
				m_bModeEuler = false;
				m_bModeMidpoint = true;
				m_vPoints_midpoint = m_vPoints_euler;
			}
			else if(m_vPoints == &m_vPoints_midpoint){
				m_bModeEuler =true;
				m_bModeMidpoint = false;
				m_vPoints_euler = m_vPoints_midpoint;
			}
		}
		if (m_bModeEuler && !m_bModeMidpoint) {
			m_vPoints = &m_vPoints_euler;
			timestep_euler(timeStep);
			for (auto &p : *m_vPoints) {
				cout << p.to_string();
			}
		}
		else if(m_bModeMidpoint && !m_bModeEuler){
			m_vPoints = &m_vPoints_midpoint;
			timestep_midpoint(timeStep);
			for (auto &p : *m_vPoints) {
				cout << p.to_string();
			}
		}
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

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed){
	Point point;
	point.m_vPosition = position;
	point.m_vVelocity = Velocity;
	point.m_vAcceleration = 0;
	point.m_vForce = 0;
	point.m_bFixed = isFixed;
	m_vPoints->push_back(point);
	return m_vPoints->size()-1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength){
	Spring spring;
	spring.m_pPoints.first = masspoint1;
	spring.m_pPoints.second = masspoint2;
	spring.m_fInitialLength = initialLength;
	spring.m_fDamping = m_fDamping;
	spring.m_fStiffness = m_fStiffness;
	m_vSprings.push_back(spring);
}

int MassSpringSystemSimulator::getNumberOfMassPoints(){
	return m_vPoints->size();
}

int MassSpringSystemSimulator::getNumberOfSprings(){
	return m_vSprings.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index){
	return m_vPoints->at(index).m_vPosition;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index){
	return m_vPoints->at(index).m_vVelocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force){

}

void MassSpringSystemSimulator::initTable1(){
	executed = false; 
	m_vSprings.clear();
	m_vPoints->clear();
	addSpring(
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false),
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false),
		1);
	m_vPoints->at(0).m_fMass = 40;
	m_vPoints->at(1).m_fMass = 40;
	setDampingFactor(0);
	m_bModeEuler = true;
	m_bModeMidpoint = false;
	m_vPoints_midpoint = m_vPoints_euler = *m_vPoints;
}

void MassSpringSystemSimulator::initDemo4(){
	m_vSprings.clear();
	m_vPoints_euler.clear();
	m_vPoints_midpoint.clear();
	std::mt19937 eng;
	std::uniform_real_distribution<float> randLen(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-1.0f, 1.0f);

	int i = addMassPoint(
		Vec3(randPos(eng), randPos(eng), randPos(eng)),
		Vec3(randPos(eng), randPos(eng), randPos(eng)),
		false);

	for(; i < m_iNumberOfWantedMasspoints; ++i) {
		addSpring(
			i,
			addMassPoint(
				Vec3(randPos(eng), randPos(eng), randPos(eng)),
				Vec3(randPos(eng), randPos(eng), randPos(eng)),
				false),
			randLen(eng));
		m_vPoints->at(i).m_fMass = randLen(eng);
	}
	addSpring(0, getNumberOfMassPoints()-1, randLen(eng));
}

float MassSpringSystemSimulator::distance_normalized(Point p1, Point p2){
	Vec3& pos1 = p1.m_vPosition;
	Vec3& pos2 = p2.m_vPosition;
	Vec3 diff = pos1 - pos2;
	return sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
}

void MassSpringSystemSimulator::timestep_euler(float timeStep){
	for (auto& s : m_vSprings) {
		Point& p1 = m_vPoints_euler.at(s.m_pPoints.first);
		Point& p2 = m_vPoints_euler.at(s.m_pPoints.second);
		Vec3 diff = p1.m_vPosition - p2.m_vPosition;
		float dist_normalized = distance_normalized(p1, p2);
		Vec3 diff_normalized = diff / dist_normalized;

		p1.m_vForce = -s.m_fStiffness * (dist_normalized - s.m_fInitialLength) * diff_normalized;
		p2.m_vForce = -s.m_fStiffness * (dist_normalized - s.m_fInitialLength) * -diff_normalized;
	}

	for (auto& p : m_vPoints_euler) {
		p.m_vAcceleration = p.m_vForce / p.m_fMass;
		p.m_vVelocity += timeStep * p.m_vAcceleration;
		p.m_vPosition += timeStep * p.m_vVelocity;
	}
}

void MassSpringSystemSimulator::timestep_midpoint(float timeStep){
	for (auto& s : m_vSprings) {
		// for euler points
		Point& p1 = m_vPoints_midpoint.at(s.m_pPoints.first);
		Point& p2 = m_vPoints_midpoint.at(s.m_pPoints.second);
		Vec3 diff = p1.m_vPosition - p2.m_vPosition;
		float dist_normalized = distance_normalized(p1, p2);
		Vec3 diff_normalized = diff / dist_normalized;

		p1.m_vForce = -s.m_fStiffness * (dist_normalized - s.m_fInitialLength) * diff_normalized;
		p2.m_vForce = -s.m_fStiffness * (dist_normalized - s.m_fInitialLength) * -diff_normalized;
	}

	for (auto& p : m_vPoints_midpoint) {
		p.m_vAcceleration = p.m_vForce / p.m_fMass;
		auto velocity_midpoint =  timeStep/2 * p.m_vAcceleration;
		p.m_vVelocity += velocity_midpoint/2 + (timeStep * p.m_vAcceleration)/2;
		p.m_vPosition += timeStep * p.m_vVelocity;
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
