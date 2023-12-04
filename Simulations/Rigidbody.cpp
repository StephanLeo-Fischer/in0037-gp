#include "Rigidbody.h"

Rigidbody::Rigidbody(Vec3 position, Vec3 size, float mass)
{
	Rigidbody(position, size, mass, Vec3());
}

Rigidbody::Rigidbody(Vec3 position, Vec3 size, float mass, Vec3 rotation)
{
	this->position = position;
	this->size = size;
	this->mass = mass;

	Mat4 rot;
	rot.initRotationXYZ(rotation.x, rotation.y, rotation.z);
	orientation = Quat(rot);

	Vec3 i = ((float)mass / 12.0) * Vec3(size.y * size.y + size.z * size.z, size.x * size.x + size.z * size.z, size.x * size.x + size.y * size.y);
	inertiaTensor = Mat4(i.x, 0, 0, 0, 0, i.y, 0, 0, 0, 0, i.z, 0, 0, 0, 0, 1);
	inertiaTensor.inverse();
}

void Rigidbody::linearEulerStep(float timeStep)
{
	position += timeStep * linearVelocity;
	linearVelocity += timeStep * (force / mass); 
}

void Rigidbody::angularEulerStep(float timeStep)
{
	orientation += (timeStep / 2.0) * Quat(angularVelocity.x, angularVelocity.y, angularVelocity.z, 0) * orientation;
	orientation.unit();

	momentum += torque * timeStep;
	angularVelocity = calculateInertia().transformVector(momentum);
}

Mat4 Rigidbody::calculateInertia()
{
	Mat4 rotMat = orientation.getRotMat();
	rotMat.transpose();

	return rotMat * inertiaTensor * orientation.getRotMat();
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

void Rigidbody::handleCollision(float impulse, Vec3 point, Vec3 normal)
{
	linearVelocity += (impulse * normal) / mass;
	momentum += cross(point - position, impulse * normal);
}

Mat4 Rigidbody::toWorldMatrix()
{
	Mat4 translatMat, scaleMat;
	translatMat.initTranslation(position.x, position.y, position.z);
	scaleMat.initScaling(size.x, size.y, size.z);

	return scaleMat * orientation.getRotMat() * translatMat;
}

Vec3 Rigidbody::worldPositionOfPoint(Vec3 point)
{
	return position + orientation.getRotMat().transformVector(point);
}

Vec3 Rigidbody::worldVelocityOfPoint(Vec3 point)
{
	return linearVelocity + cross(angularVelocity, point);
}
