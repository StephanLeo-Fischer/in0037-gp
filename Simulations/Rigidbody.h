#ifndef RIGIDBODY_h
#define RIGIDBODY_h
#include "Simulator.h"
#include "collisionDetect.h"

// Define a structure to put all the parameters describing the physics parameters of the rigidbody;
struct SimulationParameters {
	double collisionFactor;

	// Two factors added to avoid creating energy with the Euler integration method:
	double linearFriction;
	double angularFriction;
};

class Rigidbody {
public:
	// Color of the rigidbody (used for drawing)
	Vec3 color;

	// Create a rigidbody with an orientation using Euler angles (in degrees):
	Rigidbody(SimulationParameters* params, double mass, Vec3 position, Vec3 rotation, Vec3 scale);

	// Create a rigidbody with an orientation defined with a quaternion:
	Rigidbody(SimulationParameters* params, double mass, Vec3 position, Quat rotation, Vec3 scale);

	void draw(DrawingUtilitiesClass * DUC, int debugLine) const;

	void timestepEuler(double timestep);

	void addTorque(Vec3 location, Vec3 force);
	void addForce(Vec3 force);
	void setForce(Vec3 force);
	void clearForces();

	// Add some getters and setters:
	double getMass() const;
	void setMass(double mass);

	Vec3 getPosition() const;
	void setPosition(Vec3 position);

	Quat getRotation() const;
	void setRotation(Vec3 rotation);
	void setRotation(Quat rotation);

	Vec3 getScale() const;
	void setScale(Vec3 scale);

	void setParams(SimulationParameters* params);

	void setKinematic(boolean isKinematic);
	boolean isKinematic() const;

	Vec3 getLinearVelocity() const;
	void setLinearVelocity(Vec3 linearVelocity);

	Vec3 getAngularVelocity() const;
	void setAngularVelocity(Vec3 angularVelocity);

	// Compute the velocity of the given position in global space, if it was part of the rigidbody:
	Vec3 getVelocityOfPoint(Vec3 position) const;

	void manageCollision(Rigidbody* other, bool correctPos);
	void correctPosition(CollisionInfo* collisionInfo);
	void computeCollisionInfo(Rigidbody* other);

	// Return the local axes of the rigidbody:
	inline Vec3 right() const;			// X-axis
	inline Vec3 up() const;				// Y-axis
	inline Vec3 forward() const;		// Z-axis

private:
	void updateTransformMatrices();		// We need to call this when we update the position, rotation or scale of the rigidbody
	void updateInertialTensors();		// We need to call this when we update the mass or the scale of the rigidbody
	void updateCurrentInertialTensor();	// We need to call this when we changed the rotation of the rigidbody
	
	Mat4 computeCurrentInertialTensor() const;
	Mat4 computeCurrentInvInertialTensor() const;
	Vec3 computeTorque();

	// Return the axis between -up(), up(), -right(), right(), -forward() and forward() with
	// the highest dot product with the given vector
	Vec3 getAxisAlong(Vec3* vector) const;

	// Pointer to a set of parameters for the simulation
	SimulationParameters * m_pParams;

	double m_fMass;

	Vec3 m_vPosition;
	Quat m_qRotation;
	Vec3 m_vScale;

	// If the rigidbody is kinematic, it behaves like if it had an infinite mass:
	boolean m_bIsKinematic;

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

	// TODO: Delete this:
	Vec3 _DEBUG_CONTACT_POINT = 0.0;

	Vec3 m_vSumForces;						// Sum of the forces applied to this Rigidbody
	vector<pair<Vec3, Vec3>> m_vTorques;	// All the torques (position, force) applied to this Rigidbody
};
#endif