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
	float getMass() const;
	void setMass(float mass);

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

	boolean manageCollision(Rigidbody* other, float c);

	// Color of the rigidbody (used for drawing)
	Vec3 color;

private:
	void updateTransformMatrices();		// We need to call this when we update the position, rotation or scale of the rigidbody
	void updateInertialTensors();		// We need to call this when we update the mass or the scale of the rigidbody
	void updateCurrentInertialTensor();	// We need to call this when we changed the rotation of the rigidbody
	
	Mat4 computeCurrentInertialTensor() const;
	Mat4 computeCurrentInvInertialTensor() const;
	Vec3 computeTorque();

	float m_fMass;

	Vec3 m_vPosition;
	Quat m_qRotation;
	Vec3 m_vScale;

	Vec3 m_vLinearVelocity;
	Vec3 m_vAngularVelocity;

	// Rotation and transformation matrices:
	Mat4 m_mRotMat;
	Mat4 m_mTransformMatrix;	// scaleMat * rotMat * translatMat

	// Initial inertial tensor of the rigidbody, as well as it's inverse:
	Mat4 m_mInertialTensor0;
	Mat4 m_mInvInertialTensor0;
	Mat4 m_mCurrentInvInertialTensor;	// R*inv(I)*inv(R)

	Vec3 m_vAngularMomentum;

	Vec3 m_vSumForces;						// Sum of the forces applied to this Rigidbody
	vector<pair<Vec3, Vec3>> m_vTorques;	// All the torques (position, force) applied to this Rigidbody

	// TODO: Delete this !
	Vec3 testCollisionCenter;
	Vec3 testNormalCollision;
};
#endif