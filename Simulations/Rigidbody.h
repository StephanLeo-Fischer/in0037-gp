#ifndef RIGIDBODY_h
#define RIGIDBODY_h
#include "Simulator.h"

class Rigidbody {
public:
	Rigidbody(float mass, Vec3 position, Vec3 rotation, Vec3 scale);

	void draw(DrawingUtilitiesClass * DUC) const;

	void timestepEuler(float timestep);

	void applyTorque(Vec3 location, Vec3 force);
	void applyForce(Vec3 force);
	void clearForces();

	// Add some getters and setters:
	Vec3 getPosition() const;
	void setPosition(Vec3 position);

	Quat getRotation() const;
	void setRotation(Vec3 rotation);
	void setRotation(Quat rotation);

	Vec3 getScale() const;
	void setScale(Vec3 scale);

	Vec3 getLinearVelocity() const;
	void setLinearVelocity(Vec3 linearVelocity);

	Vec3 getAngularVelocity() const;
	void setAngularVelocity(Vec3 angularVelocity);

	// Compute the velocity of the given position in global space, if it was part of the rigidbody:
	Vec3 getVelocityOfPoint(Vec3 position) const;

	Mat4 getTransformMatrix() const;

	// Color of the rigidbody (used for drawing)
	Vec3 color;

private:
	void updateInertialTensors();
	Vec3 computeTorque();

	float m_fMass;

	// Inertial tensor of the rigidbody, as well as it's inverse:
	Mat4 m_mInertialTensor;
	Mat4 m_mInvInertialTensor;

	// Translation, rotation and scale matrices (used for drawing):
	Mat4 m_mTranslatMat;
	Mat4 m_mRotMat;
	Mat4 m_mScaleMat;

	Vec3 m_vPosition;
	Quat m_qRotation;
	Vec3 m_vScale;

	Vec3 m_vLinearVelocity;
	Vec3 m_vAngularVelocity;

	Vec3 m_vSumForces;						// Sum of the forces applied to this Rigidbody
	vector<pair<Vec3, Vec3>> m_vTorques;	// All the torques (position, force) applied to this Rigidbody

	Vec3 m_vAngularMomentum;
};
#endif