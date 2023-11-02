#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
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

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Euler,Leapfrog,Midpoint";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
		TwAddVarRW(DUC->g_pTweakBar, "Mass p_0", TW_TYPE_FLOAT, &m_vPoints[0].m_fMass, "min=1 max=100");
		TwAddVarRW(DUC->g_pTweakBar, "pos.x p_0", TW_TYPE_FLOAT, &m_vPoints[0].m_vPosition.x, "min=-100 max=100");
		TwAddVarRW(DUC->g_pTweakBar, "pos.y p_0", TW_TYPE_FLOAT, &m_vPoints[0].m_vPosition.y, "min=-100 max=100");
		TwAddVarRW(DUC->g_pTweakBar, "pos.z p_0", TW_TYPE_FLOAT, &m_vPoints[0].m_vPosition.z, "min=-100 max=100");

		TwAddVarRW(DUC->g_pTweakBar, "Mass p_1", TW_TYPE_FLOAT, &m_vPoints[1].m_fMass, "min=1 max=100");
		TwAddVarRW(DUC->g_pTweakBar, "pos.x p_1", TW_TYPE_FLOAT, &m_vPoints[1].m_vPosition.x, "min=-100 max=100");
		TwAddVarRW(DUC->g_pTweakBar, "pos.y p_1", TW_TYPE_FLOAT, &m_vPoints[1].m_vPosition.y, "min=-100 max=100");
		TwAddVarRW(DUC->g_pTweakBar, "pos.z p_1", TW_TYPE_FLOAT, &m_vPoints[1].m_vPosition.z, "min=-100 max=100");

		break;
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
	Vec3 white = Vec3(1, 1, 1);
	Vec3 red = Vec3(1, 0, 0);
	switch (m_iTestCase)
	{
	case 0: 
		for (int i = 0; i < getNumberOfMassPoints(); ++i) {
			DUC->drawSphere(getPositionOfMassPoint(i), size_of_ball * m_vPoints.at(i).m_fMass);
		}
		for (int i = 0; i < getNumberOfSprings(); ++i) {
			int point1_idx = m_vSprings.at(i).m_pPoints.first;
			int point2_idx = m_vSprings.at(i).m_pPoints.second;
			//does not work
			//DUC->drawLine(m_vPoints.at(point1_idx).m_vPosition, white, m_vPoints.at(point2_idx).m_vPosition, red);
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
		cout << "Test Case 1: Euler!\n";
		initTable1();
		break;
	case 1:
		cout << "Test Case 2: Leapfrog!\n";

		break;
	case 2:
		cout << "Test Case 3: Midpoint!\n";
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

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0: // Euler timestep
		for (auto &p : m_vPoints){
			p.m_vPosition.x += timeStep;
		}
		break;
	default:
		break;
	}
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

void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	Point point;
	point.m_vPosition = position;
	point.m_vVelocity = Velocity;
	point.m_vAcceleration = 0;
	point.m_vForce = 0;
	point.m_bFixed = isFixed;
	m_vPoints.push_back(point);
	return m_vPoints.size()-1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring spring;
	spring.m_pPoints.first = masspoint1;
	spring.m_pPoints.second = masspoint2;
	spring.m_fInitialLength = initialLength;
	spring.m_fDamping = m_fDamping;
	spring.m_fStiffness = m_fStiffness;
	m_vSprings.push_back(spring);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_vPoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_vSprings.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return m_vPoints.at(index).m_vPosition;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return m_vPoints.at(index).m_vVelocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
}

void MassSpringSystemSimulator::initTable1()
{
	m_vPoints.clear();
	m_vSprings.clear();
	addSpring(
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false),
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false),
		1);
	m_vPoints.at(0).m_fMass = 40;
	m_vPoints.at(1).m_fMass = 40;
	setDampingFactor(0);
}
