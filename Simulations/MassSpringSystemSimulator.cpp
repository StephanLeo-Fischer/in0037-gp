#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator() {

	containment = 5.0;
	floor = -1.0;

	// Data Attributes
	m_fMass = 10.0;
	m_fStiffness = 40.0;
	m_fDamping = 0.0;
	m_iIntegrator = 1;

	m_iIntegrationKind = EULER;

	// UI Attributes
	m_externalForce;
	m_mouse;
	m_trackmouse;
	m_oldtrackmouse;

	
}

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Demo 1,Demo 2,Demo 3,Demo 4,Demo 5";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	DUC->setUpLighting(Vec3(1, 1, 1), Vec3(0, 0, 1), 1, Vec3(0, 0, 1));

	
	switch (m_iTestCase) {
	case 0:break;
	case 1:break;
	case 2:break;
	case 3: {
		TwButtonCallback;
		TwType TW_TYPE_TESTCASE = TwDefineEnumFromString("Integration", "Euler,Midpoint");
		TwAddVarRW(DUC->g_pTweakBar, "Integration", TW_TYPE_TESTCASE, &m_iIntegrationKind, "");
		break;
	}
	case 4:break;
	default:break;
	}
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	switch (m_iTestCase) {
	case 0:break;
	default:
		for (MassPoint& p : masspoints) {
			DUC->drawSphere(p.position, Vec3(1, 1, 1) * 0.01);
		}

		DUC->beginLine();
		for (Spring s : springs) {
			DUC->drawLine(masspoints.at(s.point1).position, Vec3(0, 1, 0), masspoints.at(s.point2).position, Vec3(0, 1, 0));
		}
		DUC->endLine();
		break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
	switch (m_iTestCase) 	{
	case 0: {
		cout << "Demo 1!\n";

		gravity = Vec3(0, 0, 0);
		m_externalForce = Vec3(0, 0, 0);

		masspoints.clear();
		springs.clear();

		int point1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		int point2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(point1, point2, 1.0);

		inittemp(masspoints, temppoints);

		eulerStep(0.1);
		cout << "Euler Step:" << endl;
		for (MassPoint &p : masspoints) {
			cout << "Position: " << p.position << ", Velocity: " << p.velocity << "\n" << endl;
		}

		inittemp(temppoints, masspoints);
		midpointStep(0.1);

		cout << "Midpoint Step:" << endl;
		for (MassPoint &p : masspoints) {
			cout << "Position: " << p.position << ", Velocity: " << p.velocity << "\n" << endl;
		}

		break;
	}
	case 1: {
		cout << "Demo 2!\n";
		gravity = Vec3(0, 0, 0);
		m_externalForce = Vec3(0, 0, 0);

		masspoints.clear();
		springs.clear();

		int point1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		int point2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(point1, point2, 1.0);

		break;
	}
	case 2: {
		cout << "Demo 3!\n";
		gravity = Vec3(0, 0, 0);
		m_externalForce = Vec3(0, 0, 0);

		masspoints.clear();
		springs.clear();

		int point1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		int point2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(point1, point2, 1.0);

		inittemp(masspoints, temppoints);
		break;
	}
	case 3: {
		cout << "Demo 4!\n";
		gravity = Vec3(0, -9.81, 0);

		masspoints.clear();
		springs.clear();

		int masspoint0 = addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);
		int masspoint1 = addMassPoint(Vec3(0, 2, 0), Vec3(0, 0, 0), false);
		int masspoint2 = addMassPoint(Vec3(1, 0, 0), Vec3(0, 0, 0), false);
		int masspoint3 = addMassPoint(Vec3(0, 1, 0), Vec3(0, 0, 0), false);
		int masspoint4 = addMassPoint(Vec3(0, 0, 1), Vec3(0, 0, 0), false);
		int masspoint5 = addMassPoint(Vec3(1, 1, 0), Vec3(0, 0, 0), false);
		int masspoint6 = addMassPoint(Vec3(1, 1, 1), Vec3(0, 0, 0), false);
		int masspoint7 = addMassPoint(Vec3(2, 0, 1), Vec3(0, 0, 0), false);
		int masspoint8 = addMassPoint(Vec3(0, 0, 2), Vec3(0, 0, 0), false);
		int masspoint9 = addMassPoint(Vec3(2, 0, 0), Vec3(0, 0, 0), false);

		addSpring(masspoint0, masspoint1, 1.0);
		addSpring(masspoint0, masspoint3, 1.0);
		addSpring(masspoint0, masspoint5, 1.0);
		addSpring(masspoint0, masspoint7, 1.0);
		addSpring(masspoint0, masspoint9, 1.0);

		addSpring(masspoint1, masspoint2, 2.0);
		addSpring(masspoint2, masspoint3, 1.0);
		addSpring(masspoint3, masspoint4, 6.0);
		addSpring(masspoint4, masspoint5, 4.0);
		addSpring(masspoint5, masspoint6, 1.0);
		addSpring(masspoint6, masspoint7, 1.0);
		addSpring(masspoint7, masspoint8, 0.5);
		addSpring(masspoint8, masspoint9, 2.0);
		addSpring(masspoint9, masspoint0, 1.4);

		break;
	}
	case 4: {
		cout << "Demo 5!\n";
		gravity = Vec3(0, -9.81, 0);

		masspoints.clear();
		springs.clear();

		int masspoint0 = addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);
		int masspoint1 = addMassPoint(Vec3(0, 2, 0), Vec3(0, 0, 0), false);
		int masspoint2 = addMassPoint(Vec3(1, 0, 0), Vec3(0, 0, 0), false);
		int masspoint3 = addMassPoint(Vec3(0, 1, 0), Vec3(0, 0, 0), false);
		int masspoint4 = addMassPoint(Vec3(0, 0, 1), Vec3(0, 0, 0), false);
		int masspoint5 = addMassPoint(Vec3(1, 1, 0), Vec3(0, 0, 0), false);
		int masspoint6 = addMassPoint(Vec3(1, 1, 1), Vec3(0, 0, 0), false);
		int masspoint7 = addMassPoint(Vec3(2, 0, 1), Vec3(0, 0, 0), false);
		int masspoint8 = addMassPoint(Vec3(0, 0, 2), Vec3(0, 0, 0), false);
		int masspoint9 = addMassPoint(Vec3(2, 0, 0), Vec3(0, 0, 0), false);

		addSpring(masspoint0, masspoint1, 1.0);
		addSpring(masspoint0, masspoint3, 1.0);
		addSpring(masspoint0, masspoint5, 1.0);
		addSpring(masspoint0, masspoint7, 1.0);
		addSpring(masspoint0, masspoint9, 1.0);

		addSpring(masspoint1, masspoint2, 2.0);
		addSpring(masspoint2, masspoint3, 1.0);
		addSpring(masspoint3, masspoint4, 6.0);
		addSpring(masspoint4, masspoint5, 4.0);
		addSpring(masspoint5, masspoint6, 1.0);
		addSpring(masspoint6, masspoint7, 1.0);
		addSpring(masspoint7, masspoint8, 0.5);
		addSpring(masspoint8, masspoint9, 2.0);
		addSpring(masspoint9, masspoint0, 1.4);

		break;
	}
	default:
		cout << "Empty Test!\n";
		break;
	}

	firstTime = true;
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0) {
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

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	// update current setup for each frame
	switch (m_iTestCase) {
	// handling different cases
	case 1:
		eulerStep(0.005);
		break;
	case 2: {
		midpointStep(0.005);
		break;
	}
	case 3:
		switch (m_iIntegrationKind) {
		case 0:
			eulerStep(timeStep);
			break;
		case 1:
			inittemp(masspoints, temppoints);
			midpointStep(timeStep);
			break;
		}
		break;
	case 4: 
		leapfrogStep(timeStep);
		break;
	default:
		break;
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

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
	masspoints.push_back(MassPoint(position, Velocity, isFixed));
	return masspoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	springs.push_back(Spring(masspoint1, masspoint2, initialLength));
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return masspoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return masspoints.at(index).position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return masspoints.at(index).velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	applyExternalForce(masspoints, force);
}

void MassSpringSystemSimulator::applyExternalForce(vector<MassPoint> &points, Vec3 force) {
	for (MassPoint& p : points) {
		p.force += force;
	}
}

void MassSpringSystemSimulator::addRandomPoint() {
	
}

void MassSpringSystemSimulator::eulerStep(float timeStep) {
	for (Spring s : springs) {
		float currentLength = sqrt(masspoints.at(s.point1).position.squaredDistanceTo(masspoints.at(s.point2).position));
		Vec3 force = -m_fStiffness * (currentLength - s.length) * ((masspoints.at(s.point1).position - masspoints.at(s.point2).position) / currentLength);

		masspoints.at(s.point1).force += force;
		masspoints.at(s.point2).force -= force;
	}

	applyExternalForce(m_externalForce + gravity);

	for (MassPoint& p : masspoints) {

		p.force /= m_fMass;

		if(!p.isFixed) p.position += p.velocity * timeStep;

		p.velocity += p.force * timeStep;

		p.force = Vec3(0, 0, 0);

	}
	containmentBreach();
}

void MassSpringSystemSimulator::midpointStep(float timeStep) {
	for (Spring s : springs) {
		float currentLength = sqrt(masspoints.at(s.point1).position.squaredDistanceTo(masspoints.at(s.point2).position));
		Vec3 force = -m_fStiffness * (currentLength - s.length) * ((masspoints.at(s.point1).position - masspoints.at(s.point2).position) / currentLength);

		temppoints.at(s.point1).force += force;
		temppoints.at(s.point2).force -= force;
	}

	applyExternalForce(temppoints, m_externalForce + gravity);

	for (MassPoint& p : temppoints) {
		p.force += m_externalForce;
		p.force /= m_fMass;

		p.position += (timeStep / 2) * p.velocity;
		p.velocity += (timeStep / 2) * p.force;

		p.force = Vec3(0, 0, 0);
	}

	for (Spring s : springs) {
		float currentLength = sqrt(temppoints.at(s.point1).position.squaredDistanceTo(temppoints.at(s.point2).position));
		Vec3 force = -m_fStiffness * (currentLength - s.length) * ((temppoints.at(s.point1).position - temppoints.at(s.point2).position) / currentLength);

		masspoints.at(s.point1).force += force;
		masspoints.at(s.point2).force -= force;
	}

	applyExternalForce(m_externalForce + gravity);

	for (int i = 0; i < masspoints.size(); i++) {
		MassPoint& p = masspoints.at(i);

		p.force /= m_fMass;

		if (!p.isFixed) p.position += temppoints.at(i).velocity * timeStep;

		p.velocity += p.force * timeStep;

		p.force = Vec3(0, 0, 0);
	}
	containmentBreach();
}

void MassSpringSystemSimulator::leapfrogStep(float timeStep) {
	for (Spring s : springs) {
		float currentLength = sqrt(masspoints.at(s.point1).position.squaredDistanceTo(masspoints.at(s.point2).position));
		Vec3 force = -m_fStiffness * (currentLength - s.length) * ((masspoints.at(s.point1).position - masspoints.at(s.point2).position) / currentLength);

		masspoints.at(s.point1).force += force;
		masspoints.at(s.point2).force -= force;
	}

	applyExternalForce(m_externalForce + gravity);

	for (MassPoint& p : masspoints) {
		p.force /= m_fMass;

		p.velocity += p.force * timeStep;

		if (!p.isFixed) p.position += p.velocity * timeStep;

		p.force = Vec3(0, 0, 0);
	}

	containmentBreach();
}

void MassSpringSystemSimulator::containmentBreach() {
	for (MassPoint& p : masspoints) {
		if (p.position.x > containment) p.position.x = containment;
		else if (p.position.x < -containment) p.position.x = -containment;
		if (p.position.y > containment) p.position.y = containment;
		else if (p.position.y < floor) p.position.y = floor;
		if (p.position.z > containment) p.position.z = containment;
		else if (p.position.z < -containment) p.position.z = -containment;
	}
}

void MassSpringSystemSimulator::inittemp(vector<MassPoint>& main, vector<MassPoint>& temp) {
	temp.clear();
	for (int i = 0; i < main.size(); i++) {
		MassPoint p = main.at(i);
		temp.push_back(MassPoint(p.position, p.velocity, p.isFixed));
	}
}

