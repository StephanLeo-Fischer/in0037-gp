#include "Rigidbody.h"
#include "collisionDetect.h"

#define RADIANS(deg) (deg * M_PI / 180)

Rigidbody::Rigidbody(float mass, Vec3 position, Vec3 rotation, Vec3 scale) : 
	m_fMass(mass),
	m_vPosition(position),
	m_qRotation(RADIANS(rotation.x), RADIANS(rotation.y), RADIANS(rotation.z)),
	m_vScale(scale.x, scale.y, scale.z),

	m_vLinearVelocity(0.0), 
	m_vAngularVelocity(0.0), 
	color(0.2) {

	updateTransformMatrices();
	updateInertialTensors();

	clearForces();
}

void Rigidbody::draw(DrawingUtilitiesClass* DUC) const
{
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, color);
	DUC->drawRigidBody(m_mTransformMatrix);

	// For debug:
	DUC->beginLine();
	DUC->drawLine(m_vPosition, Vec3(0, 0, 0), m_vPosition + m_vLinearVelocity, Vec3(0, 0, 1));
	DUC->drawLine(m_vPosition, Vec3(0, 0, 0), m_vPosition + m_vAngularVelocity, Vec3(1, 0, 0));
	DUC->drawLine(m_vPosition, Vec3(0, 0, 0), m_vPosition + m_vAngularMomentum, Vec3(1, 1, 0));
	DUC->drawLine(testCollisionCenter, Vec3(0, 0, 0), testCollisionCenter + testNormalCollision, Vec3(0, 1, 0));
	DUC->endLine();
}

void Rigidbody::timestepEuler(float timestep)
{
	m_vPosition += timestep * m_vLinearVelocity;
	m_vLinearVelocity += timestep * m_vSumForces / m_fMass;

	Quat w = Quat(m_vAngularVelocity.x, m_vAngularVelocity.y, m_vAngularVelocity.z, 0);
	m_qRotation = (m_qRotation + 0.5 * timestep * w * m_qRotation).unit();

	// Update the rotation and transformation matrices, since we changed the rotation and position of the rigidbody:
	updateTransformMatrices();

	// Update the angular momentum:
	m_vAngularMomentum += timestep * computeTorque();

	// Update the current inertial tensor, and use it to update the angular velocity:
	m_mCurrentInvInertialTensor = computeCurrentInvInertialTensor();

	m_vAngularVelocity = m_mCurrentInvInertialTensor.transformVector(m_vAngularMomentum);
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

float Rigidbody::getMass() const { return m_fMass; }

void Rigidbody::setMass(float mass) {
	this->m_fMass = mass;
	updateInertialTensors();
}

Vec3 Rigidbody::getPosition() const { return m_vPosition; }
void Rigidbody::setPosition(Vec3 position) { 
	this->m_vPosition = position;
	updateTransformMatrices();
}

Quat Rigidbody::getRotation() const { return m_qRotation; }
void Rigidbody::setRotation(Vec3 rotation) {
	setRotation(Quat(RADIANS(rotation.x), RADIANS(rotation.y), RADIANS(rotation.z)));
}
void Rigidbody::setRotation(Quat rotation) { 
	this->m_qRotation = rotation;
	updateTransformMatrices();
	updateCurrentInertialTensor();
}

Vec3 Rigidbody::getScale() const { return m_vScale; }
void Rigidbody::setScale(Vec3 scale) { 
	this->m_vScale = scale;
	updateTransformMatrices();
	updateInertialTensors();
}

Vec3 Rigidbody::getLinearVelocity() const { return m_vLinearVelocity; }
void Rigidbody::setLinearVelocity(Vec3 linearVelocity) { this->m_vLinearVelocity = linearVelocity; }

Vec3 Rigidbody::getAngularVelocity() const { return m_vAngularVelocity; }

void Rigidbody::setAngularVelocity(Vec3 angularVelocity) {
	this->m_vAngularVelocity = angularVelocity;

	// Update the angular momentum, since we changed the angular velocity of the rigidbody:
	this->m_vAngularMomentum = computeCurrentInertialTensor().transformVector(angularVelocity);
}

Vec3 Rigidbody::getVelocityOfPoint(Vec3 position) const {
	return m_vLinearVelocity + m_vAngularVelocity * (position - m_vPosition);
}

boolean Rigidbody::manageCollision(Rigidbody* other, float c)
{
	Mat4 transformA = m_mTransformMatrix;
	Mat4 transformB = other->m_mTransformMatrix;

	CollisionInfo collision = checkCollisionSAT(transformA, transformB);

	if (collision.isValid) {
		// Change the color of the rigidbodies:
		this->color = Vec3(1, 0, 0);
		other->color = Vec3(1, 0, 0);

		// Compute the impulse to update both rigidbodies:

		Vec3 position = collision.collisionPointWorld;
		Vec3 n = collision.normalWorld;
		testNormalCollision = n * 100;
		testCollisionCenter = position;

		float Ma = m_fMass;
		float Mb = other->m_fMass;

		Vec3 xa = position - m_vPosition;
		Vec3 xb = position - other->m_vPosition;

		Vec3 va = m_vLinearVelocity;
		Vec3 vb = other->m_vLinearVelocity;

		Vec3 wa = m_vAngularVelocity;
		Vec3 wb = other->m_vAngularVelocity;

		Vec3 vr = va + cross(wa, xa) - vb - cross(wb, xb);

		// If the collision was already managed, then return:
		if (dot(vr, n) >= 0)
			return true;

		Mat4 invIa = m_mCurrentInvInertialTensor;
		Mat4 invIb = other->m_mCurrentInvInertialTensor;

		Vec3 A = cross(invIa.transformVector(cross(xa, n)), xa);
		Vec3 B = cross(invIb.transformVector(cross(xb, n)), xb);

		float num = -(1 + c) * dot(vr, n);
		float den = (1 / Ma + 1 / Mb + dot(A + B, n));

		// Result of the impulse:
		float J = num / den;

		// Update va and vb for both rigidbodies:
		this->m_vLinearVelocity  = va + J * n / Ma;
		other->m_vLinearVelocity = vb - J * n / Mb;

		// Update La and Lb for both rigidbodies:
		Vec3 La = m_vAngularMomentum;
		Vec3 Lb = other->m_vAngularMomentum;

		this->m_vAngularMomentum  = La + cross(xa, J * n);
		other->m_vAngularMomentum = Lb - cross(xb, J * n);

		// Update: Shouldn't we also update the angular velocity of the rigidbodies ?
		this->m_vAngularVelocity  = invIa.transformVector(this->m_vAngularMomentum);
		other->m_vAngularVelocity = invIb.transformVector(other->m_vAngularMomentum);
	}
	else {
		this->color = Vec3(0.4);
		other->color = Vec3(0.4);
	}

	// Return if there was a collision between the two rigidbodies
	return collision.isValid;
}

void Rigidbody::updateInertialTensors()
{
	// Compute the initial inertial tensor and it's inverse:
	float A = m_fMass * (m_vScale.y * m_vScale.y + m_vScale.z * m_vScale.z) / 12;
	float B = m_fMass * (m_vScale.x * m_vScale.x + m_vScale.z * m_vScale.z) / 12;
	float C = m_fMass * (m_vScale.x * m_vScale.x + m_vScale.y * m_vScale.y) / 12;

	this->m_mInertialTensor0 = Mat4(
		A, 0, 0, 0,
		0, B, 0, 0,
		0, 0, C, 0,
		0, 0, 0, 1
	);

	this->m_mInvInertialTensor0 = Mat4(
		1.0/A,	0,		0,		0,
		0,		1.0/B,	0,		0,
		0,		0,		1.0/C,	0,
		0,		0,		0,		1
	);

	updateCurrentInertialTensor();
}

void Rigidbody::updateCurrentInertialTensor()
{
	// Update the current inertial tensor of the rigidbody, depending on it's orientation:
	this->m_mCurrentInvInertialTensor = computeCurrentInvInertialTensor();

	// Update the angular momentum, since we changed the inertial tensor of the rigidbody:
	this->m_vAngularMomentum = computeCurrentInertialTensor().transformVector(m_vAngularVelocity);
}

Mat4 Rigidbody::computeCurrentInertialTensor() const {
	Mat4 invRotMat = m_mRotMat;
	invRotMat.transpose();

	// return m_mRotMat * m_mInertialTensor0 * invRotMat;
	return invRotMat * m_mInertialTensor0 * m_mRotMat;
}

Mat4 Rigidbody::computeCurrentInvInertialTensor() const {
	Mat4 invRotMat = m_mRotMat;
	invRotMat.transpose();

	//return m_mRotMat * m_mInvInertialTensor0 * invRotMat;
	return invRotMat * m_mInvInertialTensor0 * m_mRotMat;
}

void Rigidbody::updateTransformMatrices()
{
	// Update the rotation matrix:
	this->m_mRotMat = m_qRotation.getRotMat();

	// Update the transformation matrix (scaleMat * rotMat * translatMat):
	Mat4 tmp;

	// Scaling:
	tmp.initScaling(m_vScale.x, m_vScale.y, m_vScale.z);
	this->m_mTransformMatrix = tmp * m_mRotMat;

	// Translation:
	tmp.initTranslation(m_vPosition.x, m_vPosition.y, m_vPosition.z);
	this->m_mTransformMatrix = m_mTransformMatrix * tmp;
}

Vec3 Rigidbody::computeTorque() {
	Vec3 sum;

	for (pair<Vec3, Vec3>& torque : m_vTorques)
		sum += cross(torque.first - m_vPosition, torque.second);

	return sum;
}
