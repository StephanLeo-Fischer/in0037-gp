#pragma once
#include "util/vectorbase.h"

using namespace GamePhysics;

class Rigidbody
{
public:
	Vec3 position;
	Vec3 size;
	float mass;

	Vec3 linearVelocity;
	Vec3 angularVelocity;

	Vec3 force;
	Vec3 torque;

	Rigidbody(Vec3 position, Vec3 size, float mass);

	void linearEulerStep(float timeStep);

	void addForce(Vec3 loc, Vec3 force);
	void clearForces();
};

