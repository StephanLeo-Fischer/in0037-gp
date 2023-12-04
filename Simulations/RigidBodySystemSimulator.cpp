#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 0;
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo 1,Demo 2,Demo 3,Demo 4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:break;
	case 1:break;
	case 2:
		TwAddVarRW(DUC->g_pTweakBar, "Coeff. o. Rest.", TW_TYPE_FLOAT, &cor, "min=0.1 step=0.1 max=1");
		break;
	case 3:break;
	default:break;
	}
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase) {
	case 0:break;
	default:
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		Mat4 camera = DUC->g_camera.GetWorldMatrix();

		for (Rigidbody& r : rigidbodies) {
			DUC->drawRigidBody(r.toWorldMatrix() * camera);
		}
		break;
	}
	
	/*
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
		m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}
	*/
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	rigidbodies.clear();
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Demo 1!\n";
		initDemo1();
		eulerStep(2.0);
		printResults();
		break;
	case 1:
		cout << "Demo 2!\n";
		initDemo1();
		break;
	case 2:
		cout << "Demo 3!\n";
		initDemo3();
		break;
	case 3:
		cout << "Demo 4!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase) {
	case 1:
		eulerStep(0.01);
		break;
	case 2:
		eulerStep(timeStep);
		collisionHandling();
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

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return 0;
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return Vec3();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return Vec3();
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return Vec3();
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	rigidbodies.at(i).addForce(loc, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	rigidbodies.push_back(Rigidbody(position, size, (float)mass));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
}

void RigidBodySystemSimulator::initDemo1()
{
	addRigidBody(Vec3(), Vec3(1, 0.6, 0.5), 2, Vec3(0, 0, 90));
	applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
}

void RigidBodySystemSimulator::initDemo3()
{
	addRigidBody(Vec3(), Vec3(1), 1, Vec3());
	addRigidBody(Vec3(0, 3, 0), Vec3(1), 1, Vec3(0, 45, 45));

	rigidbodies.at(0).linearVelocity = Vec3(0, 0.5, 0);
	rigidbodies.at(1).linearVelocity = Vec3(0, -0.5, 0);
}

void RigidBodySystemSimulator::printResults()
{
	Rigidbody& r = rigidbodies.at(0);
	cout << "Linear Velocity: " << r.linearVelocity << endl;
	cout << "Angular Velocity: " << r.angularVelocity << endl;
	cout << "World Velocity of Point (-0.3, -0.5, -0.25): " << r.worldVelocityOfPoint(Vec3(-0.3, -0.5, -0.25)) << endl;
	cout << "World Velocity of Point (0.3, 0.5, 0.25): " << r.worldVelocityOfPoint(Vec3(0.3, 0.5, 0.25)) << endl;
}

void RigidBodySystemSimulator::eulerStep(float timeStep)
{
	for (Rigidbody& r : rigidbodies) {
		r.linearEulerStep(timeStep);
		r.angularEulerStep(timeStep);
		r.clearForces();
	}	
}

void RigidBodySystemSimulator::collisionHandling()
{
	for (int i = 0; i < rigidbodies.size(); i++) {

		Rigidbody& a = rigidbodies.at(i);

		for (int j = i + 1; j < rigidbodies.size(); j++) {

			Rigidbody& b = rigidbodies.at(j);
			auto info = checkCollisionSAT(a.toWorldMatrix(), b.toWorldMatrix());
			if (!info.isValid) return;

			Vec3 vrel = a.worldVelocityOfPoint(info.collisionPointWorld - a.position) - b.worldVelocityOfPoint(info.collisionPointWorld - b.position);
			if (dot(vrel, info.normalWorld) >= 0) return;

			float impulse = calculateImpulse(vrel, a, b, info.collisionPointWorld, info.normalWorld);
			a.handleCollision(impulse, info.collisionPointWorld, info.normalWorld);
			b.handleCollision(-impulse, info.collisionPointWorld, info.normalWorld);
		}
	}	
}

float RigidBodySystemSimulator::calculateImpulse(Vec3 vrel, Rigidbody a, Rigidbody b, Vec3 collisionPoint, Vec3 normal)
{
	auto top = dot(-(1.0 + cor) * vrel, normal);
	auto bottomMasses = (1.0 / a.mass) + (1.0 / b.mass);

	Vec3 aCollisionPoint = collisionPoint - a.position;
	auto aInertiaThingy = cross(a.calculateInertia().transformVector(cross(aCollisionPoint, normal)), aCollisionPoint);
	
	Vec3 bCollisionPoint = collisionPoint - b.position;
	auto bInertiaThingy = cross(b.calculateInertia().transformVector(cross(bCollisionPoint, normal)), bCollisionPoint);
	

	return top / (bottomMasses + dot(aInertiaThingy + bInertiaThingy, normal));
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass, Vec3 rotation)
{
	rigidbodies.push_back(Rigidbody(position, size, (float)mass, rotation));
}
