#include "Rigidbody.h"

Rigidbody::Rigidbody(float mass, Vec3 position, Vec3 rotation, Vec3 scale) : 
	m_fMass(mass), m_vLinearVelocity(0.0), m_vAngularVelocity(0.0), color(1) {

	setPosition(position);
	setRotation(rotation);
	setScale(scale);

	clearForces();
}

void Rigidbody::draw(DrawingUtilitiesClass* DUC) const
{
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, color);
	DUC->drawRigidBody(m_mScaleMat * m_mRotMat * m_mTranslatMat);
}

void Rigidbody::timestepEuler(float timestep)
{
	m_vPosition += timestep * m_vLinearVelocity;
	m_vLinearVelocity += timestep * m_vSumForces / m_fMass;

	Quat w = Quat(0, m_vAngularVelocity.x, m_vAngularVelocity.y, m_vAngularVelocity.z);
	m_qRotation = (m_qRotation + 0.5 * timestep * w * m_qRotation).unit();

	// Update the translation and rotation matrices:
	m_mTranslatMat.initTranslation(m_vPosition.x, m_vPosition.y, m_vPosition.z);
	m_mRotMat = m_qRotation.getRotMat();

	// Update the angular momentum:
	m_vAngularMomentum += timestep * computeTorque();

	// Compute the inverse of the rotation matrix (which is also it's transposition):
	Mat4 invRotMat = m_mRotMat;
	invRotMat.transpose();

	m_vAngularVelocity = (m_mRotMat * m_mInvInertialTensor * invRotMat).transformVector(m_vAngularMomentum);
}

void Rigidbody::applyTorque(Vec3 location, Vec3 force) {
	pair<Vec3, Vec3> torque;
	torque.first = location;
	torque.second = force;

	this->m_vSumForces += force;
	this->m_vTorques.push_back(torque);
}

void Rigidbody::applyForce(Vec3 force) {
	this->m_vSumForces += force;
}

void Rigidbody::clearForces() {
	this->m_vTorques.clear();
	this->m_vSumForces = 0;
}

Vec3 Rigidbody::getPosition() const { return m_vPosition; }
void Rigidbody::setPosition(Vec3 position) { 
	this->m_vPosition = position;
	this->m_mTranslatMat.initTranslation(position.x, position.y, position.z);
}

Quat Rigidbody::getRotation() const { return m_qRotation; }
void Rigidbody::setRotation(Vec3 rotation) {
	// Convert angles from degrees to radians:
	const float factor = M_PI / 180;

	this->m_qRotation = Quat(factor * rotation.x, factor * rotation.y, factor * rotation.z);
	this->m_mRotMat.initRotationXYZ(rotation.x, rotation.y, rotation.z);
}
void Rigidbody::setRotation(Quat rotation) { 
	this->m_qRotation = rotation;
	this->m_mRotMat = m_qRotation.getRotMat();
}

Vec3 Rigidbody::getScale() const { return m_vScale; }
void Rigidbody::setScale(Vec3 scale) { 
	this->m_vScale = scale;
	this->m_mScaleMat.initScaling(scale.x, scale.y, scale.z);

	// Update the inertial tensors, which depends on the scale of the object:
	updateInertialTensors();
}

Vec3 Rigidbody::getLinearVelocity() const { return m_vLinearVelocity; }
void Rigidbody::setLinearVelocity(Vec3 linearVelocity) { this->m_vLinearVelocity = linearVelocity; }

Vec3 Rigidbody::getAngularVelocity() const { return m_vAngularVelocity; }

void Rigidbody::setAngularVelocity(Vec3 angularVelocity) {
	this->m_vAngularVelocity = angularVelocity;

	// Update the angular momentum, since we changed the angular velocity of the rigidbody:
	this->m_vAngularMomentum = m_mInertialTensor.transformVector(angularVelocity);
}

Vec3 Rigidbody::getVelocityOfPoint(Vec3 position) const {
	return m_vLinearVelocity + m_vAngularVelocity * (position - m_vPosition);
}

Mat4 Rigidbody::getTransformMatrix() const
{
	return m_mScaleMat * m_mRotMat * m_mTranslatMat;
}

void Rigidbody::updateInertialTensors()
{
	// Compute the inertial tensor and it's inverse:
	float A = m_fMass * (m_vScale.y * m_vScale.y + m_vScale.z * m_vScale.z) / 12;
	float B = m_fMass * (m_vScale.x * m_vScale.x + m_vScale.z * m_vScale.z) / 12;
	float C = m_fMass * (m_vScale.x * m_vScale.x + m_vScale.y * m_vScale.y) / 12;

	this->m_mInertialTensor = Mat4(
		A, 0, 0, 0,
		0, B, 0, 0,
		0, 0, C, 0,
		0, 0, 0, 1
	);

	this->m_mInvInertialTensor = Mat4(
		1.0/A,	0,		0,		0,
		0,		1.0/B,	0,		0,
		0,		0,		1.0/C,	0,
		0,		0,		0,		1
	);

	// Update the angular momentum, since we changed the inertial tensor of the rigidbody:
	this->m_vAngularMomentum = m_mInertialTensor.transformVector(m_vAngularVelocity);
}

Vec3 Rigidbody::computeTorque() {
	Vec3 sum;

	for (pair<Vec3, Vec3>& torque : m_vTorques)
		sum += cross(torque.first - m_vPosition, torque.second);

	return sum;
}
