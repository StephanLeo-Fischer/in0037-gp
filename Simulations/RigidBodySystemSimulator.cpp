#pragma once

#include "RigidBodySystemSimulator.h"

#define demo1 "Demo-1: A simple one-step test"
#define demo2 "Demo-2: Simple single-body simulation"
#define demo3 "Demo-3: Two-rigid-body collision scene"
#define demo4 "Demo-4: Complex Simulation"

// #########################################################
//           CONSTRUCTOR/DESCRUCTOR
// #########################################################
RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 1;
	m_pRigidBodySystem = new RigidBodySystem();
}



// #########################################################
//           FUNCTIONS
// #########################################################
const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo-1,Demo-2,Demo-3,Demo-4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}


void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	Vec3 colors[3] = { Vec3(0.5,0.1,0.1), Vec3(0.1,0.5,0.1), Vec3(0.1,0.1,0.5) };
	int i = 0;
	for (RigidBody& rigidBody : m_pRigidBodySystem->m_vRigidBodies)
	{
		DUC->setUpLighting(Vec3(0, 0, 0), 1 * Vec3(1, 1, 1), 2000.0, colors[i % 3]);
		DUC->drawRigidBody(rigidBody.getObj2World());
		++i;
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (testCase) 
	{
	case 0:
		cout << demo1 << endl;
		break;
	case 1:
		cout << demo2 << endl;
		break;
	case 2:
		cout << demo3 << endl;
		break;
	case 3:
		cout << demo4 << endl;
		break;
	default:
		cout << "Empty Test!" << endl;
		break;
	}
	m_pRigidBodySystem->scene(m_iTestCase);
}

void RigidBodySystemSimulator::externalForcesCalculations(float elapsedTime)
{
	Vec3 pullforce(0, 0, 0);
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 forceView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 forceWorld = worldViewInv.transformVectorNormal(forceView);
		float forceScale = 0.2f;
		pullforce = pullforce + (forceWorld * forceScale);
	}
	//pullforce -=  pullforce * 5.0f * timeElapsed;

	m_externalForce = pullforce;

}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 0: // case 0 do nothing
		break;

	case 1:
		m_pRigidBodySystem->addGlobalFrameForce(m_externalForce);
		m_pRigidBodySystem->update(timeStep);
		break;

	case 2:
		if (DXUTIsKeyDown(VK_LBUTTON))
			m_pRigidBodySystem->moveTowards();
		m_pRigidBodySystem->addGlobalFrameForce(m_externalForce);
		m_pRigidBodySystem->update(timeStep);
		break;

	default:
		break;
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// #########################################################
//           EXTRA FUNCTIONS
// #########################################################

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_pRigidBodySystem->m_vRigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return m_pRigidBodySystem->m_vRigidBodies[i].getCenter();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_pRigidBodySystem->m_vRigidBodies[i].getVelocity();
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_pRigidBodySystem->m_vRigidBodies[i].getAngularV();
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_pRigidBodySystem->m_vRigidBodies[i].addForceWorld(force, loc);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	m_pRigidBodySystem->m_vRigidBodies.emplace_back(position, size, mass);
	m_pRigidBodySystem->m_vRigidBodies.back().update(0.0f);

}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	m_pRigidBodySystem->m_vRigidBodies[i].setRotation(orientation);

}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_pRigidBodySystem->m_vRigidBodies[i].setVelocity(velocity);
}
