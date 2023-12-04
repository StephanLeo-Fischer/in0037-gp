#pragma once
#include "util/quaternion.h"
#include "collisionDetect.h"

using namespace GamePhysics;

class Rigidbody
{
public:
	Vec3 position;
	Vec3 size;
	float mass;

	Quat orientation;
	Mat4 inertiaTensor;
	Vec3 momentum;

	Vec3 linearVelocity;
	Vec3 angularVelocity;

	Vec3 force;
	Vec3 torque;

	Rigidbody(Vec3 position, Vec3 size, float mass);
	Rigidbody(Vec3 position, Vec3 size, float mass, Vec3 rotation);

	void linearEulerStep(float timeStep);
	void angularEulerStep(float timeStep);

	Mat4 calculateInertia();

	void addForce(Vec3 loc, Vec3 force);
	void clearForces();

	void handleCollision(float impulse, Vec3 point, Vec3 normal);

	Mat4 toWorldMatrix();
	Vec3 worldPositionOfPoint(Vec3 point);
	Vec3 worldVelocityOfPoint(Vec3 point);
};

