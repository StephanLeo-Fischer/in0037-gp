#include "Rigidbody.h"
#include "collisionDetect.h"

#define RADIANS(deg) (deg * M_PI / 180)

Rigidbody::Rigidbody(SimulationParameters * params, float mass, Vec3 position, Vec3 rotation, Vec3 scale) : 
	Rigidbody(params, mass, position, Quat(RADIANS(rotation.x), RADIANS(rotation.y), RADIANS(rotation.z)), scale) {}

Rigidbody::Rigidbody(SimulationParameters* params, float mass, Vec3 position, Quat rotation, Vec3 scale) :
	m_pParams(params),
	m_fMass(mass),
	m_vPosition(position),
	m_qRotation(rotation),
	m_vScale(scale.x, scale.y, scale.z),

	m_bIsKinematic(false),

	m_vLinearVelocity(0.0),
	m_vPrevLinearVelocity(0.0),
	m_vAngularVelocity(0.0),
	m_vPrevAngularVelocity(0.0),
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
	DUC->endLine();
}

void Rigidbody::timestepEuler(float timestep)
{
	// This is used to perform micro collisions:
	m_vPrevLinearVelocity = m_vLinearVelocity;
	m_vPrevAngularVelocity = m_vAngularVelocity;

	Quat w = Quat(m_vAngularVelocity.x, m_vAngularVelocity.y, m_vAngularVelocity.z, 0);
	
	m_vPosition += timestep * m_vLinearVelocity;
	m_qRotation = (m_qRotation + 0.5 * timestep * w * m_qRotation).unit();

	// Update the rotation and transformation matrices, since we changed the rotation and position of the rigidbody:
	updateTransformMatrices();

	if (!m_bIsKinematic) {
		// Update the linear velocity, and angular momentum:
		m_vLinearVelocity += timestep * m_vSumForces / m_fMass;
		m_vAngularMomentum += timestep * computeTorque();

		// Add friction to the linear velocity and angular momentum, to prevent the Euler method from creating energy:
		m_vLinearVelocity *= m_pParams->linearFriction;
		m_vAngularMomentum *= m_pParams->angularFriction;

		// Update the current inertial tensor, and use it to update the angular velocity:
		m_mCurrentInvInertialTensor = computeCurrentInvInertialTensor();
		m_vAngularVelocity = m_mCurrentInvInertialTensor.transformVector(m_vAngularMomentum);
	}
}

void Rigidbody::applyTorque(Vec3 location, Vec3 force) {
	// If the object is kinematic, ignore the forces applied to it:
	if (!m_bIsKinematic) {
		pair<Vec3, Vec3> torque;
		torque.first = location;
		torque.second = force;

		this->m_vSumForces += force;
		this->m_vTorques.push_back(torque);
	}
}

void Rigidbody::applyForce(Vec3 force) {
	// If the object is kinematic, ignore the forces applied to it:
	if(!m_bIsKinematic)
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

void Rigidbody::setParams(SimulationParameters* params) { this->m_pParams = params; }

void Rigidbody::setKinematic(boolean isKinematic) { 
	this->m_bIsKinematic = true;

	if (isKinematic) {
		// If the object is kinematic, ignore the forces applied to it:
		m_mCurrentInvInertialTensor = 0;
		m_vAngularMomentum = 0;
		clearForces();
	}
	else {
		// Else, update the current inertial tensor and angular momentum of the rigidbody,
		// based on it's orientation and angular speed:
		m_mCurrentInvInertialTensor = computeCurrentInertialTensor();
		m_vAngularMomentum = computeCurrentInertialTensor().transformVector(m_vAngularVelocity);
	}
}
boolean Rigidbody::isKinematic() const { return m_bIsKinematic; }

Vec3 Rigidbody::getLinearVelocity() const { return m_vLinearVelocity; }
void Rigidbody::setLinearVelocity(Vec3 linearVelocity) { 
	this->m_vLinearVelocity = linearVelocity;
	this->m_vPrevLinearVelocity = linearVelocity;
}

Vec3 Rigidbody::getAngularVelocity() const { return m_vAngularVelocity; }
void Rigidbody::setAngularVelocity(Vec3 angularVelocity) {
	this->m_vAngularVelocity = angularVelocity;
	this->m_vPrevAngularVelocity = angularVelocity;

	// If the rigidbody is not kinematic, update it's angular momentum, since we changed 
	// it's angular velocity:
	if(!m_bIsKinematic)
		this->m_vAngularMomentum = computeCurrentInertialTensor().transformVector(angularVelocity);
}

Vec3 Rigidbody::getVelocityOfPoint(Vec3 position) const {
	return m_vLinearVelocity + m_vAngularVelocity * (position - m_vPosition);
}

boolean Rigidbody::manageCollision(Rigidbody* other)
{
	Mat4 transformA = m_mTransformMatrix;
	Mat4 transformB = other->m_mTransformMatrix;

	CollisionInfo collision = checkCollisionSAT(transformA, transformB);

	if (collision.isValid) {
		// If both objects are kinematic, ignore the collision:
		if (m_bIsKinematic && other->m_bIsKinematic)
			return true;

		// Compute the impulse to update both rigidbodies:
		float c = m_pParams->collisionFactor;

		Vec3 position = collision.collisionPointWorld;
		Vec3 n = collision.normalWorld;

		Vec3 xa = position - m_vPosition;
		Vec3 xb = position - other->m_vPosition;

		Vec3 va = m_vLinearVelocity;
		Vec3 vb = other->m_vLinearVelocity;

		Vec3 wa = m_vAngularVelocity;
		Vec3 wb = other->m_vAngularVelocity;

		Vec3 vr = va + cross(wa, xa) - vb - cross(wb, xb);

		// Compute the numerator of the impulse J:
		float numJ;
		if (!m_pParams->enableMicroCollisions) {
			numJ = -(1 + c) * dot(vr, n);
		}
		else {
			// Using the previous values for va, wa, vb and wb, compute the previous vrel:
			Vec3 pVa = m_vPrevLinearVelocity;
			Vec3 pVb = other->m_vPrevLinearVelocity;
			Vec3 pWa = m_vPrevAngularVelocity;
			Vec3 pWb = other->m_vPrevAngularVelocity;

			Vec3 prev_vr = pVa + cross(pWa, xa) - pVb - cross(pWb, xb);

			numJ = -dot(vr + c * prev_vr, n);
		}


		if (m_bIsKinematic) {
			// Little improvement: update the position of the rigidbodies directly, to manage the
			// situation when rigidbodies are just colliding with vrel = 0:
			// other->m_vPosition -= collision.depth * n;

			// If the collision was already managed, then return:
			if (dot(vr, n) >= 0)
				return true;


			// Ma -> +infinity, so invA = 0
			float Mb = other->m_fMass;

			Mat4 invIb = other->m_mCurrentInvInertialTensor;
			Vec3 B = cross(invIb.transformVector(cross(xb, n)), xb);

			// Result of the impulse:
			float J = numJ / (1/Mb + dot(B, n));

			// Update the linear velocity of the other rigidbody:
			other->m_vLinearVelocity = vb - J * n / Mb;

			// Update the angular momentum and velocity of the other rigidbody:
			other->m_vAngularMomentum = other->m_vAngularMomentum - cross(xb, J * n);
			other->m_vAngularVelocity = invIb.transformVector(other->m_vAngularMomentum);
		}
		else if (other->m_bIsKinematic) {
			// Little improvement: update the position of the rigidbodies directly, to manage the
			// situation when rigidbodies are just colliding with vrel = 0:
			// this->m_vPosition += collision.depth * n;

			// If the collision was already managed, then return:
			if (dot(vr, n) >= 0)
				return true;

			// Mb -> +infinity, so invB = 0
			float Ma = m_fMass;

			Mat4 invIa = m_mCurrentInvInertialTensor;
			Vec3 A = cross(invIa.transformVector(cross(xa, n)), xa);

			// Result of the impulse:
			float J = numJ / (1/Ma + dot(A, n));
			
			// Update the linear velocity of this rigidbody:
			this->m_vLinearVelocity = va + J * n / Ma;

			// Update the angular momentum and velocity of this rigidbody:
			this->m_vAngularMomentum = m_vAngularMomentum + cross(xa, J * n);
			this->m_vAngularVelocity = invIa.transformVector(this->m_vAngularMomentum);
		}
		else {
			float Ma = m_fMass;
			float Mb = other->m_fMass;

			// Little improvement: update the position of the rigidbodies directly, to manage the
			// situation when rigidbodies are just colliding with vrel = 0:
			// this->m_vPosition += Mb * collision.depth * n / (Ma + Mb);
			// other->m_vPosition -= Ma * collision.depth * n / (Ma + Mb);

			// If the collision was already managed, then return:
			if (dot(vr, n) >= 0)
				return true;

			Mat4 invIa = m_mCurrentInvInertialTensor;
			Mat4 invIb = other->m_mCurrentInvInertialTensor;

			Vec3 A = cross(invIa.transformVector(cross(xa, n)), xa);
			Vec3 B = cross(invIb.transformVector(cross(xb, n)), xb);

			// Result of the impulse:
			float J = numJ / (1/Ma + 1/Mb + dot(A + B, n));

			// Update va and vb for both rigidbodies:
			this->m_vLinearVelocity = va + J * n / Ma;
			other->m_vLinearVelocity = vb - J * n / Mb;

			// Update La and Lb for both rigidbodies:
			Vec3 La = m_vAngularMomentum;
			Vec3 Lb = other->m_vAngularMomentum;

			this->m_vAngularMomentum = La + cross(xa, J * n);
			other->m_vAngularMomentum = Lb - cross(xb, J * n);

			// Update the angular velocity of both rigidbodies:
			this->m_vAngularVelocity = invIa.transformVector(this->m_vAngularMomentum);
			other->m_vAngularVelocity = invIb.transformVector(other->m_vAngularMomentum);
		}
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
	// If the rigidbody is kinematic, the inverse of it's inertial tensor is null, and thus 
	// it's angular momentum is also null, so we can ignore this:
	if (!m_bIsKinematic) {
		// Update the current inertial tensor of the rigidbody, depending on it's orientation:
		this->m_mCurrentInvInertialTensor = computeCurrentInvInertialTensor();

		// Update the angular momentum, since we changed the inertial tensor of the rigidbody:
		this->m_vAngularMomentum = computeCurrentInertialTensor().transformVector(m_vAngularVelocity);
	}
}

// This should be called only if the rigidbody is not kinematic:
Mat4 Rigidbody::computeCurrentInertialTensor() const {
	Mat4 invRotMat = m_mRotMat;
	invRotMat.transpose();

	// return m_mRotMat * m_mInertialTensor0 * invRotMat;
	return invRotMat * m_mInertialTensor0 * m_mRotMat;
}

// This should be called only if the rigidbody is not kinematic:
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

// This should be called only if the rigidbody is not kinematic:
Vec3 Rigidbody::computeTorque() {
	Vec3 sum;

	for (pair<Vec3, Vec3>& torque : m_vTorques)
		sum += cross(torque.first - m_vPosition, torque.second);

	return sum;
}
