#include "Rigidbody.h"


Rigidbody::Rigidbody(float mass, Vec3 pos, Quat orientation, Vec3 scale) :
	m_fMass(mass),
	m_vPosition(pos),
	m_qOrientation(orientation),
	m_vScale(scale.x, scale.y, scale.z),
	m_vLinearVelocity(0.0),
	m_vAngularVelocity(0.0),
	m_bIsKinematic(false),
	color(0.2)
{
	updateTransformMatrices();
	updateInertialTensor0s();
	clearForces();
}

void Rigidbody::timestepEuler(float timeStep)
{
	m_vPosition += timeStep * m_vLinearVelocity;
	m_vLinearVelocity += timeStep * m_vSumForces / m_fMass;
	m_qOrientation += (timeStep / 2) * Quat(m_vAngularVelocity.x, m_vAngularVelocity.y, m_vAngularVelocity.z, 0) * m_qOrientation;
	m_vAngularMomentum += timeStep * m_vTorque;
	Mat4 transposedRotation = m_mRotMat;
	transposedRotation.transpose();
	Mat4 inertiaTensorInverse = m_mRotMat * m_mInvInertialTensor0 * transposedRotation;
	m_vAngularVelocity = inertiaTensorInverse * m_vAngularMomentum;
}



void Rigidbody::addForce(Vec3 loc, Vec3 force)
{
	m_vSumForces += force;
	m_vTorque += cross(loc, force);
}
void Rigidbody::clearForces()
{
	m_vSumForces = Vec3();
	m_vTorque = Vec3();
}

void Rigidbody::updateInertialTensor0s()
{
	// Compute the initial inertial tensor and it's inverse:
	float A = m_fMass * (m_vScale.y * m_vScale.y + m_vScale.z * m_vScale.z) / 12;
	float B = m_fMass * (m_vScale.x * m_vScale.x + m_vScale.z * m_vScale.z) / 12;
	float C = m_fMass * (m_vScale.x * m_vScale.x + m_vScale.y * m_vScale.y) / 12;

	this->m_mInertialTensor0 = Mat4(
		A, 0, 0, 0,
		0, B, 0, 0,
		0, 0, C, 0,
		0, 0, 0, 0  // theo has a 1 in last 
	);

	this->m_mInvInertialTensor0 = Mat4(
		1.0 / A, 0, 0, 0,
		0, 1.0 / B, 0, 0,
		0, 0, 1.0 / C, 0,
		0, 0, 0, 0  // theo has a 1 in last 
	);

	updateCurrentInertialTensor();
}



void Rigidbody::updateCurrentInertialTensor()
{
	// If the rigidbody is kinematic, the inverse of it's inertial tensor is null, and thus 
	// it's angular momentum is also null, so we can ignore this:
	if (!m_bIsKinematic) {
		Mat4 invRotMat = m_mRotMat;
		invRotMat.transpose();

		// Update the current inertial tensor of the rigidbody, depending on it's orientation:
		this->m_mCurrentInvInertialTensor = invRotMat * m_mInertialTensor0 * m_mRotMat;

		// Update the angular momentum, since we changed the inertial tensor of the rigidbody:
		this->m_vAngularMomentum = (invRotMat * m_mInvInertialTensor0 * m_mRotMat).transformVector(m_vAngularVelocity);
	}
}

std::string Rigidbody::toString() {
	std::string s;
	s = "\tp: " + m_vPosition.toString()
		+ "\n\tv: " + m_vLinearVelocity.toString()
		+ "\n\tf: " + m_vSumForces.toString() + '\n';
	//<< "\n\tm:" << m_fMass << '\n';
	s += '\n';
	return s;
}
