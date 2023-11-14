#include "MassSpringSystemSimulator.h"


MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	// Data Attributes
	m_fMass = 1.0;
	m_fStiffness = 1.0;
	m_fDamping = 1.0;
	m_iIntegrator = 1;

	// UI Attributes
	m_externalForce;
	m_mouse = Point2D();
	m_trackmouse = Point2D();
	m_oldtrackmouse = Point2D();
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Euler,Leapfrog,Mitpoint";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:break;
	case 1:break;
	case 2:break;
	default:break;
	}
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0:  // euler 

		for (Point p : _points) {
			DUC->drawSphere(p.getPosition(), 0.12);  // magic number size
		}

		for (Spring s : _springs) {
			Vec3 pos1 = getPositionOfMassPoint(s.getIndexFirstConnectedPoint());
			Vec3 pos2 = getPositionOfMassPoint(s.getIndexSecondConnectedPoint());
			DUC->beginLine();
			DUC->drawLine(pos1, Vec3(0, 0, 0), pos2, Vec3(1, 1, 1));  // random colors
			DUC->endLine();
		}
		break;
	case 1: break;
	case 2: break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Test Case 1!\n";
		initDemo1();

		break;
	case 1:
		cout << "Test Case 2!\n";

		break;
	case 2:
		cout << "Test Case 3!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
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
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		//m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		//m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}
}

void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}
void MassSpringSystemSimulator::setStiffness(float stiffness) {
	m_fStiffness = stiffness;
}
void MassSpringSystemSimulator::setDampingFactor(float damping) {
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed) {
	Point point = Point{ position, velocity, Vec3(), Vec3(), m_fMass, isFixed };
	_points.push_back(point);  // adding it to list of all points
	return _points.size() - 1;  // gibt index in / größe der Liste zurück 
}


void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	Spring& spring = Spring{ masspoint1, masspoint2, initialLength, m_fStiffness, m_fDamping };
	_springs.push_back(spring);  // adding it to list of all springs
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return _points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return _springs.size();
}


Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) 
{
	return _points.at(index).getPosition();
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return _points.at(index).getVelocity();
}


void MassSpringSystemSimulator::initDemo1() 
{
	setMass(10);
	setStiffness(40);
	setDampingFactor(1);  // 1 sollte nichts dampen
	int p1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	int p2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
	addSpring(p1, p2, 1);  // L = 1
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	int i = 0;  // index des jeweiligen punktes
	// update current setup for each frame
	switch (m_iTestCase)
	{// handling different cases
	case 0:  // Euler timestep
		//single step
		if (_frameElapsed) break;
		printPoints();
		eulerSimulation(timeStep);
		printPoints();
		_frameElapsed = true;
		
		break;
	case 1:  // midpoint
		break;
	case 2:  // leapfrog
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::printPoints() {
	std::cout << "\n";
	for (Point& p : _points) {
		std::cout << "\npos " << p.getPosition() << "\tvel " << p.getVelocity() << "\tacc " << p._acc << "\tforce " << p._force;
	}

}

void MassSpringSystemSimulator::eulerSimulation(float timeStep) {
	// get forces from springs
	for (Spring& spring : _springs) {
		Point& p1 = _points.at(spring.getIndexFirstConnectedPoint());
		Point& p2 = _points.at(spring.getIndexSecondConnectedPoint());

		// hooke's law: -k * (l - L) * normalizedDirection
		// l distance
		Vec3 p1Pos = p1.getPosition();
		Vec3 p2Pos = p2.getPosition();
		float l = sqrt(pow(p1Pos.x - p2Pos.x, 2) + pow(p1Pos.y - p2Pos.y, 2) + pow(p1Pos.z - p2Pos.z, 2));
		Vec3 dir = p2Pos - p1Pos;
		Vec3 p1Force = -m_fStiffness * (l - spring.getInitialLength()) * (dir / l);

		printPoints();
		p1._force += Vec3(1,1,1);//p1Force;  // wenn man mehr springs hat, werden die werte aber überschrieben. TODO
		p2._force += Vec3(1, 1, 1);// -p1Force;  // trick 17
		printPoints();
	}

	// apply forces to masses
	for (Point& point : _points)
	{
		point.setAcceleration(point._force / point._mass);
		point.setVelocity(point._vel + timeStep * point._acc);
		point.setPosition(point._pos + timeStep * point._vel);
	}

}