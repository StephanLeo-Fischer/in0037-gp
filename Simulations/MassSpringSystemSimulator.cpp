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
	case 0:  // euler single step
		if (_frameElapsed) break;

		for (Point p : _points) {
			DUC->drawSphere(p.getPosition(), 0.05);  // magic number 
		}

		for (Spring s : _springs) {
			Vec3 pos1 = getPositionOfMassPoint(s.getIndexFirstConnectedPoint());
			Vec3 pos2 = getPositionOfMassPoint(s.getIndexSecondConnectedPoint());
			std::cout << pos1 << pos2;
			DUC->drawLine(pos1, Vec3(0, 0, 0), pos2, Vec3(1, 1, 1));  // random colors
		}
		_frameElapsed = true;
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
	Point point{ position, velocity, Vec3(), Vec3(), m_fMass, isFixed };
	_points.push_back(point);  // adding it to list of all points
	return _points.size() - 1;  // gibt index in / größe der Liste zurück 
}


void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	Spring spring{ masspoint1, masspoint2, initialLength, m_fStiffness, m_fDamping };
	_springs.push_back(spring);  // adding it to list of all springs
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return _points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return _springs.size();
}


Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	// help from https://stackoverflow.com/questions/16747591/how-to-get-an-element-at-specified-index-from-c-list
	auto pointsFront = _points.begin();
	std::advance(pointsFront, index);
	return (*pointsFront).getPosition();
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	// help from https://stackoverflow.com/questions/16747591/how-to-get-an-element-at-specified-index-from-c-list
	auto pointsFront = _points.begin();
	std::advance(pointsFront, index);
	return (*pointsFront).getVelocity();
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
		for (Point point : _points) {
			//std::cout << i++ << '\t' << ("%s", point.to_string());
		}

		/*m_vfRotate.x += timeStep;
		if (m_vfRotate.x > 2 * M_PI) m_vfRotate.x -= 2.0f * (float)M_PI;
		m_vfRotate.y += timeStep;
		if (m_vfRotate.y > 2 * M_PI) m_vfRotate.y -= 2.0f * (float)M_PI;
		m_vfRotate.z += timeStep;
		if (m_vfRotate.z > 2 * M_PI) m_vfRotate.z -= 2.0f * (float)M_PI;*/
		
		break;
	case 1:  // midpoint
		break;
	case 2:  // leapfrog
		break;
	default:
		break;
	}
}
