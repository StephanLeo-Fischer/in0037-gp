#include "Rigidbody.h"

Rigidbody::Rigidbody(Vec3 position, Vec3 size, float mass)
{
	this->position = position;
	this->size = size;
	this->mass = mass;
}

void Rigidbody::linearEulerStep(float timeStep)
{
	position += timeStep * linearVelocity;
	linearVelocity += timeStep * (force / mass); 
}

void Rigidbody::addForce(Vec3 loc, Vec3 force)
{
	this->force += force;
	torque += cross(loc, force);
}

void Rigidbody::clearForces()
{
	force = Vec3();
	torque = Vec3();
}
