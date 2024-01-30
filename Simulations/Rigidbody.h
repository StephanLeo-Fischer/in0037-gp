#ifndef RIGIDBODY_h
#define RIGIDBODY_h
#include <unordered_set>

#include "Simulator.h"
#include "collisionDetect.h"

// Define a structure to put all the parameters describing the physics parameters of the simulation:
struct SimulationParameters {
	// The collision factor used to compute the impulse between two colliding rigidbodies:
	double collisionFactor;

	// Each time we do a timestepEuler(), we multiply the linear velocity and angular 
	// momentum of the rigidbody by (1-airFriction) to simulate friction with air:
	double airFriction;

	// This friction is used to reduce the speed of an object sliding on the ground 
	// for example:
	double objectFriction;

	// If the impulse is lower than this value, we will also use position correction on the rigidbody:
	double minimumImpulse;

	// Minimum number of frames an immobile object has to wait before going into idle state:
	int minFramesBeforeIdle;

	// When two rigidbodies are in each other, if the impulse is not big enough,
	// we may have to update directly the position of the rigidbodies (instead of the speed),
	// to manage the intersection. However, the changes shouldn't be too big (we don't want
	// to "teleport" objects to different positions). Thus, we define a maximum linear and
	// angular correction speed:
	double maxLinearCorrectionSpeed;
	double maxAngularCorrectionSpeed;

	// When correcting the position of rigidbodies, we still want objects to keep colliding (if we are at the
	// limit between collision, and no collision, the state of the colliding objects will always change, which
	// is bad). The depthTarget is the target value of the collision depth between colliding objects (should be small):
	double depthTarget;

	// If the linear and angular velocities of a rigidbody is below these values, this 
	// object can go in idle state. In this state, the rigidbody stops beign updated, 
	// util it's collided by a new rigidbody:
	double sqMinimumLinearVelocity;		// Square of the min (avoid computing square roots)
	double sqMinimumAngularVelocity;	// Square of the min (avoid computing square roots)
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

	void setKinematic(bool isKinematic);
	bool isKinematic() const;

	Vec3 getLinearVelocity() const;
	void setLinearVelocity(Vec3 linearVelocity);

	Vec3 getAngularVelocity() const;
	void setAngularVelocity(Vec3 angularVelocity);

	// Compute the velocity of the given position in global space, if it was part of the rigidbody:
	Vec3 getVelocityOfPoint(Vec3 position) const;

	// Return the local axes of the rigidbody:
	inline Vec3 right() const;			// X-axis
	inline Vec3 up() const;				// Y-axis
	inline Vec3 forward() const;		// Z-axis

	Vec3 transformLocalToGlobal(Vec3 localPosition);

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	// Functions about idle state of rigidbodies:

	bool isIdle() const;

	// Add a new rigidbody to the list of rigidbodies colliding this one
	void addCollider(Rigidbody* rigidbody);

	void allowIdleState(bool allow);

	void exitIdleState();

	void checkIsMooving();


	// New functions:
	bool hasChangedState();

	// Get all the objects colliding this rigidbody, but also the objects colliding them, etc...
	void getFullNeighborhood(std::unordered_set<Rigidbody*>* target);

	void checkIdleState();

	void nextFrame();


	//////////////////////////////////////////////////////////////////////////////////////////////////////

	static CollisionInfo computeCollision(Rigidbody* rigidbodyA, Rigidbody* rigidbodyB);

	// Compute the impulse between two rigidbodies, apply this impulse to the rigidbodies, and return
	// the computed impulse:
	static double computeImpulse(Rigidbody* rigidbodyA, Rigidbody* rigidbodyB, double collisionFactor, Vec3 collisionPoint, Vec3 collisionNormal);

	// Correct the position of the given rigidbody, given the collision info with another rigidbody that is supposed 
	// to be fixed. We can use this function when the impulse between two rigidbodies is small (i.e. collision with ground)
	// The collisionNormal should be from the fixed rigidbody to the given rigidbody:
	static void correctPosition(Rigidbody* r, const SimulationParameters* params, 
		Vec3 collisionPoint, Vec3 collisionNormal, double collisionDepth, double timestep);

	// Correct the position of both rigidbodies, given the collision info between them.
	// We can use this function when the impulse between the rigidbodies is small.
	// The collision normal should be from A to B:
	static void correctPosition(Rigidbody* rigidbodyA, Rigidbody* rigidbodyB, const SimulationParameters* params, 
		Vec3 collisionPoint, Vec3 collisionNormal, double collisionDepth, double timestep);

private:
	void updateTransformMatrices();		// We need to call this when we update the position, rotation or scale of the rigidbody
	void updateInertiaTensors();		// We need to call this when we update the mass or the scale of the rigidbody
	void updateCurrentInertiaTensor();	// We need to call this when we changed the rotation of the rigidbody
	
	Mat4 computeCurrentInertiaTensor() const;
	Mat4 computeCurrentInvInertiaTensor() const;
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

	// Used for collision detection: this is just max(m_vScale):
	double m_fBoundingSphereRadius;

	// If the rigidbody is kinematic, it behaves like if it had an infinite mass:
	bool m_bIsKinematic;

	// If the rigidbody is idle, this means that it doesn't need to be updated (it's velocity is null,
	// and the impulses it's getting from other rigidbodies is almost zero). A rigidbody will exit the
	// idle state as soon as the state of the rigidbodies that is colliding it changes
	bool m_bIsIdle;

	// We can prevent some objects from going to idle state with this:
	bool m_bAllowIdleState;

	// If the velocity of the rigidbody is above the thresholds defined in the simulation parameters:
	bool m_bIsMooving;

	// This flag becomes true if the state of the rigidbody has changed since the last frame. This is
	// the case when the set of colliding objects have changed fo example:
	bool m_bStateChanged;

	// Number of frames this object still has to wait before being able to go in idle state:
	int m_iRemainingFramesBeforeIdle;

	// List of the rigidbodies that were colliding this object in the previous and current frame:
	vector<Rigidbody*> m_vPrevColliders;
	vector<Rigidbody*> m_vCurrColliders;

	Vec3 m_vLinearVelocity;
	Vec3 m_vAngularVelocity;

	// TESTING:
	Vec3 m_vFilteredAngularVelocity;

	// Rotation and transformation matrices:
	Mat4 m_mRotMat;
	Mat4 m_mTransformMatrix;	// scaleMat * rotMat * translatMat

	// Initial inertia tensor of the rigidbody, as well as it's inverse:
	Mat4 m_mInertiaTensor0;
	Mat4 m_mInvInertiaTensor0;
	Mat4 m_mCurrentInvInertiaTensor;	// R*inv(I)*inv(R)

	Vec3 m_vAngularMomentum;

	Vec3 m_vSumForces;						// Sum of the forces applied to this Rigidbody
	vector<pair<Vec3, Vec3>> m_vTorques;	// All the torques (position, force) applied to this Rigidbody
};

#endif