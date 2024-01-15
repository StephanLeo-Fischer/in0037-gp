#include "Rigidbody.h"
#include "collisionDetect.h"

#define RADIANS(deg) (deg * M_PI / 180)
#define DEGREES(rad) (rad * 180 / M_PI)

Rigidbody::Rigidbody(SimulationParameters * params, double mass, Vec3 position, Vec3 rotation, Vec3 scale) : 
	Rigidbody(params, mass, position, Quat(RADIANS(rotation.x), RADIANS(rotation.y), RADIANS(rotation.z)), scale) {}

Rigidbody::Rigidbody(SimulationParameters* params, double mass, Vec3 position, Quat rotation, Vec3 scale) :
	m_pParams(params),
	m_fMass(mass),
	m_vPosition(position),
	m_qRotation(rotation),
	m_vScale(scale.x, scale.y, scale.z),
	m_fBoundingSphereRadius(max(max(scale.x, scale.y), scale.z)),

	m_bIsKinematic(false),
	m_bIsIdle(false),
	m_bAllowIdleState(true),
	m_bIsMooving(true),

	m_vLinearVelocity(0.0),
	m_vAngularVelocity(0.0),
	m_vFilteredAngularVelocity(0.0),
	color(0.2) {

	updateTransformMatrices();
	updateInertiaTensors();

	clearForces();
}

void Rigidbody::draw(DrawingUtilitiesClass* DUC, int debugLine) const
{
	// Use a red tint for idle rigidbodies:
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, m_bIsIdle ? Vec3(color.x, color.y/2, color.z/2) : color);
	DUC->drawRigidBody(m_mTransformMatrix);

	// For debug:
	const Vec3 red	 = Vec3(1, 0, 0);
	const Vec3 green = Vec3(0, 1, 0);
	const Vec3 blue  = Vec3(0, 0, 1);

	DUC->beginLine();

	switch (debugLine) {
	case 1:
		DUC->drawLine(m_vPosition, red, m_vPosition + m_vLinearVelocity, red);
		break;
	case 2:
		DUC->drawLine(m_vPosition, red, m_vPosition + m_vAngularVelocity, red);
		break;
	case 3:
		DUC->drawLine(m_vPosition, red, m_vPosition + m_vFilteredAngularVelocity, red);
		break;
	case 4:
		DUC->drawLine(m_vPosition, green, m_vPosition + m_vAngularMomentum, green);
		break;
	case 5:
		DUC->drawLine(m_vPosition, blue, m_vPosition + m_vSumForces, blue);
		break;
	}

	// TEST:
	// DUC->drawLine(m_vPosition, red, m_vPosition + right(), red);
	// DUC->drawLine(m_vPosition, green, m_vPosition + up(), green);
	// DUC->drawLine(m_vPosition, blue, m_vPosition + forward(), blue);

	DUC->endLine();
}

void Rigidbody::timestepEuler(double timestep) {
	m_vFilteredAngularVelocity = 0.9 * m_vFilteredAngularVelocity + 0.1 * m_vAngularVelocity;

	if (m_bIsKinematic || m_bIsIdle)
		return;

	Quat w = Quat(m_vAngularVelocity.x, m_vAngularVelocity.y, m_vAngularVelocity.z, 0);
	
	m_vPosition += timestep * m_vLinearVelocity;
	m_qRotation = (m_qRotation + 0.5 * timestep * w * m_qRotation).unit();

	// Update the rotation and transformation matrices, since we changed the rotation and position of the rigidbody:
	updateTransformMatrices();

	// Update the linear velocity, and angular momentum:
	m_vLinearVelocity += timestep * m_vSumForces / m_fMass;
	m_vAngularMomentum += timestep * computeTorque();

	// Add friction to the linear velocity and angular momentum, to stabilize the system:
	m_vLinearVelocity *= (1 - m_pParams->airFriction);
	m_vAngularMomentum *= (1 - m_pParams->airFriction);

	// Update the current inertia tensor, and use it to update the angular velocity:
	m_mCurrentInvInertiaTensor = computeCurrentInvInertiaTensor();
	m_vAngularVelocity = m_mCurrentInvInertiaTensor.transformVector(m_vAngularMomentum);
}

void Rigidbody::addTorque(Vec3 location, Vec3 force) {
	// If the object is kinematic, ignore the forces applied to it:
	if (!m_bIsKinematic) {
		pair<Vec3, Vec3> torque;
		torque.first = location;
		torque.second = force;

		this->m_vSumForces += force;
		this->m_vTorques.push_back(torque);
	}
}

void Rigidbody::addForce(Vec3 force) {
	// If the object is kinematic, ignore the forces applied to it:
	if(!m_bIsKinematic)
		this->m_vSumForces += force;
}

void Rigidbody::setForce(Vec3 force) {
	// If the object is kinematic, ignore the forces applied to it:
	if (!m_bIsKinematic)
		this->m_vSumForces = force;
}

void Rigidbody::clearForces() {
	this->m_vTorques.clear();
	this->m_vSumForces = Vec3(0, 0, 0);
}

void Rigidbody::addCollider(Rigidbody* rigidbody) {
	m_vCurrColliders.push_back(rigidbody);
}

void Rigidbody::updateColliders() {
	m_vPrevColliders = m_vCurrColliders;
	m_vCurrColliders.clear();

	// Testing: we know that idle objects are not moving. Thus,
	// the collisions between idle objects will be the same in
	// the next frame. We don't need to recompute them !
	if (m_bIsIdle) {
		for (auto& c : m_vPrevColliders)
			if (c->m_bIsIdle)
				m_vCurrColliders.push_back(c);
	}
}

double Rigidbody::getMass() const { return m_fMass; }
void Rigidbody::setMass(double mass) {
	this->m_fMass = mass;
	updateInertiaTensors();
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
	updateCurrentInertiaTensor();
}

Vec3 Rigidbody::getScale() const { return m_vScale; }
void Rigidbody::setScale(Vec3 scale) { 
	this->m_vScale = scale;
	this->m_fBoundingSphereRadius = max(max(scale.x, scale.y), scale.z);

	updateTransformMatrices();
	updateInertiaTensors();
}

void Rigidbody::setParams(SimulationParameters* params) { this->m_pParams = params; }

void Rigidbody::setKinematic(bool isKinematic) { 
	this->m_bIsKinematic = isKinematic;
	this->m_bIsIdle = isKinematic;

	if (isKinematic) {
		// If the object is kinematic, ignore the forces applied to it, and set 
		// its velocity to zero:
		m_mCurrentInvInertiaTensor = 0;
		m_vAngularMomentum = 0;
		m_vLinearVelocity = 0;
		m_vAngularVelocity = 0;
		clearForces();
	}
	else {
		// Else, update the current inertia tensor and angular momentum of the rigidbody,
		// based on it's orientation and angular speed:
		m_mCurrentInvInertiaTensor = computeCurrentInertiaTensor();
		m_vAngularMomentum = computeCurrentInertiaTensor().transformVector(m_vAngularVelocity);
	}
}
bool Rigidbody::isKinematic() const { return m_bIsKinematic; }

bool Rigidbody::isIdle() const { return m_bIsIdle; }

void Rigidbody::allowIdleState(bool allow) {
	m_bAllowIdleState = allow;

	if (!allow && !m_bIsKinematic)
		m_bIsIdle = false;
}

void Rigidbody::exitIdleState()
{
	// Kinematic objects are always in idle state, and shouldn't be affected by this function
	// For non kinematic object, changing their idle state will also change the state of all
	// the rigidbodies colliding this object:
	if (!m_bIsKinematic && m_bIsIdle) {
		m_bIsIdle = false;

		for (auto& r : m_vCurrColliders)
			r->exitIdleState();
	}
}

void Rigidbody::checkIsMooving() {
	double sqLinearVelocity = normNoSqrt(m_vLinearVelocity);
	double sqAngularVelocity = normNoSqrt(m_vFilteredAngularVelocity);

	// The object is not mooving if its linear and angular velocities are below the thresholds
	// defined in the simulation parameters:
	m_bIsMooving = sqLinearVelocity >= m_pParams->sqMinimumLinearVelocity
		|| sqAngularVelocity >= m_pParams->sqMinimumAngularVelocity;
}

Vec3 Rigidbody::getLinearVelocity() const { return m_vLinearVelocity; }
void Rigidbody::setLinearVelocity(Vec3 linearVelocity) { 
	this->m_vLinearVelocity = linearVelocity;
}

Vec3 Rigidbody::getAngularVelocity() const { return m_vAngularVelocity; }
void Rigidbody::setAngularVelocity(Vec3 angularVelocity) {
	this->m_vAngularVelocity = angularVelocity;

	// If the rigidbody is not kinematic, update it's angular momentum, since we changed 
	// it's angular velocity:
	if(!m_bIsKinematic)
		this->m_vAngularMomentum = computeCurrentInertiaTensor().transformVector(angularVelocity);
}

Vec3 Rigidbody::getVelocityOfPoint(Vec3 position) const {
	return m_vLinearVelocity + m_vAngularVelocity * (position - m_vPosition);
}

// X-axis
inline Vec3 Rigidbody::right() const {
	return Vec3(m_mTransformMatrix.value[0][0], m_mTransformMatrix.value[0][1], m_mTransformMatrix.value[0][2]);
}

// Y-axis
inline Vec3 Rigidbody::up() const {
	return Vec3(m_mTransformMatrix.value[1][0], m_mTransformMatrix.value[1][1], m_mTransformMatrix.value[1][2]);
}

// Z-axis
inline Vec3 Rigidbody::forward() const {
	return Vec3(m_mTransformMatrix.value[2][0], m_mTransformMatrix.value[2][1], m_mTransformMatrix.value[2][2]);
}

Vec3 Rigidbody::transformLocalToGlobal(Vec3 localPosition) {
	return m_mTransformMatrix.transformVector(localPosition);
}

// A rigidbody can stay in idle state only if all the rigidbodies colliding it are also in idle state.
// The rigidbody also looses immediately this state if the set of colliding objects has changed
// since the last frame:
void Rigidbody::checkKeepIdleState()
{
	// As soon as one colliding object is not in idle state, this object also looses its idle state.
	// If we are already not in idle state, there is nothing to do:
	if (m_bIsIdle) {
		for (int i = 0; i < m_vCurrColliders.size(); i++) {
			if (!m_vCurrColliders[i]->m_bIsIdle) {
				exitIdleState();
				return;
			}
		}

		// We also loose the idle state if the set of colliders has changed:
		if (m_vCurrColliders.size() != m_vPrevColliders.size())
			exitIdleState();
		else {
			sort(m_vPrevColliders.begin(), m_vPrevColliders.end());
			sort(m_vCurrColliders.begin(), m_vCurrColliders.end());

			if (m_vPrevColliders != m_vCurrColliders)
				exitIdleState();
		}
	}
}

void Rigidbody::checkEnterIdleState() {
	// We can enter idle state if this rigidbody, and all the colliding rigidbodies 
	// have a small velocity:

	if (m_bIsIdle || !m_bAllowIdleState)
		return;

	if (m_bIsMooving) {
		m_iRemainingFramesBeforeIdle = 5;
		return;
	}

	for (auto& r : m_vCurrColliders)
		if (r->m_bIsMooving)
			return;

	if (--m_iRemainingFramesBeforeIdle <= 0) {
		m_bIsIdle = true;
		m_vLinearVelocity = 0;
		m_vAngularVelocity = 0;
		m_vAngularMomentum = 0;
	}
}

void Rigidbody::updateInertiaTensors()
{
	// Compute the initial inertia tensor and it's inverse:
	double A = m_fMass * (m_vScale.y * m_vScale.y + m_vScale.z * m_vScale.z) / 12;
	double B = m_fMass * (m_vScale.x * m_vScale.x + m_vScale.z * m_vScale.z) / 12;
	double C = m_fMass * (m_vScale.x * m_vScale.x + m_vScale.y * m_vScale.y) / 12;

	this->m_mInertiaTensor0 = Mat4(
		A, 0, 0, 0,
		0, B, 0, 0,
		0, 0, C, 0,
		0, 0, 0, 1
	);

	this->m_mInvInertiaTensor0 = Mat4(
		1.0/A,	0,		0,		0,
		0,		1.0/B,	0,		0,
		0,		0,		1.0/C,	0,
		0,		0,		0,		1
	);

	updateCurrentInertiaTensor();
}

void Rigidbody::updateCurrentInertiaTensor()
{
	// If the rigidbody is kinematic, the inverse of it's inertia tensor is null, and thus 
	// it's angular momentum is also null, so we can ignore this:
	if (!m_bIsKinematic) {
		// Update the current inertia tensor of the rigidbody, depending on it's orientation:
		this->m_mCurrentInvInertiaTensor = computeCurrentInvInertiaTensor();

		// Update the angular momentum, since we changed the inertia tensor of the rigidbody:
		this->m_vAngularMomentum = computeCurrentInertiaTensor().transformVector(m_vAngularVelocity);
	}
}

// This should be called only if the rigidbody is not kinematic:
Mat4 Rigidbody::computeCurrentInertiaTensor() const {
	Mat4 invRotMat = m_mRotMat;
	invRotMat.transpose();

	// return m_mRotMat * m_mInertiaTensor0 * invRotMat;
	return invRotMat * m_mInertiaTensor0 * m_mRotMat;
}

// This should be called only if the rigidbody is not kinematic:
Mat4 Rigidbody::computeCurrentInvInertiaTensor() const {
	Mat4 invRotMat = m_mRotMat;
	invRotMat.transpose();

	return invRotMat * m_mInvInertiaTensor0 * m_mRotMat;
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

// Return the axis of the rigidbody that is the most aligned with the given vector:
Vec3 Rigidbody::getAxisAlong(Vec3* vector) const
{
	Vec3 right = this->right();
	Vec3 up = this->up();
	Vec3 forward = this->forward();

	double d1 = dot(right / norm(right), *vector);
	double ad1 = abs(d1);

	double d2 = dot(up / norm(up), *vector);
	double ad2 = abs(d2);

	double d3 = dot(forward / norm(forward), *vector);
	double ad3 = abs(d3);

	if (ad1 >= ad2 && ad1 >= ad3)
		return d1 > 0 ? right : -right;
	
	else if (ad2 >= ad3)
		return d2 > 0 ? up : -up;

	return d3 > 0 ? forward : -forward;
}

CollisionInfo Rigidbody::computeCollision(Rigidbody* rigidbodyA, Rigidbody* rigidbodyB) {
	
	// First check if the bounding spheres of the rigidbodies are colliding: if not, we know that there is no collision:
	double sumRadius = rigidbodyA->m_fBoundingSphereRadius + rigidbodyB->m_fBoundingSphereRadius;
	if (normNoSqrt(rigidbodyA->m_vPosition - rigidbodyB->m_vPosition) > sumRadius * sumRadius) {
		CollisionInfo collision;
		collision.isValid = false;

		return collision;
	}
	
	return checkCollisionSAT(rigidbodyA->m_mTransformMatrix, rigidbodyB->m_mTransformMatrix);
}

double Rigidbody::computeImpulse(Rigidbody* rigidbodyA, Rigidbody* rigidbodyB, double collisionFactor, Vec3 collisionPoint, Vec3 collisionNormal) {
	
	// If both rigidbodies are kinematic, ignore collisions:
	if (rigidbodyA->m_bIsKinematic && rigidbodyB->m_bIsKinematic)
		return 0;

	// In this situation, this object is not kinematic, and the other is kinematic:
	if (rigidbodyB->m_bIsKinematic)
		return computeImpulse(rigidbodyB, rigidbodyA, collisionFactor, collisionPoint, -collisionNormal);

	// There are thus only two remaining possibilities:
	// - Either rigidbodyA is kinematic, and rigidbodyB is not
	// - Or both objects are not kinematic
	
	// Compute the impulse to update both rigidbodies:
	Vec3 position = collisionPoint;
	Vec3 n = collisionNormal;

	Vec3 xa = position - rigidbodyA->m_vPosition;
	Vec3 xb = position - rigidbodyB->m_vPosition;

	Vec3 va = rigidbodyA->m_vLinearVelocity;
	Vec3 vb = rigidbodyB->m_vLinearVelocity;

	Vec3 wa = rigidbodyA->m_vAngularVelocity;
	Vec3 wb = rigidbodyB->m_vAngularVelocity;

	Vec3 vr = va + cross(wa, xa) - vb - cross(wb, xb);

	// First case: rigidbodyA is kinematic (and rigidbodyB is not):
	if (rigidbodyA->m_bIsKinematic) {
		// If the collision was already managed, then return:
		if (dot(vr, n) >= 0)
			return 0;

		// Ma -> +infinity, so invA = 0
		double Mb = rigidbodyB->m_fMass;

		Mat4 invIb = rigidbodyB->m_mCurrentInvInertiaTensor;
		Vec3 B = cross(invIb.transformVector(cross(xb, n)), xb);

		// Result of the impulse:
		double J = -(1 + collisionFactor) * dot(vr, n) / (1 / Mb + dot(B, n));
		
		// Update the linear velocity of rigidbodyB:
		rigidbodyB->m_vLinearVelocity = vb - J * n / Mb;

		// Update the angular momentum and velocity of rigidbodyB:
		rigidbodyB->m_vAngularMomentum = rigidbodyB->m_vAngularMomentum - cross(xb, J * n);
		rigidbodyB->m_vAngularVelocity = invIb.transformVector(rigidbodyB->m_vAngularMomentum);

		return J;
	}

	// Second case: both objects aren't kinematic:
	else {
		double Ma = rigidbodyA->m_fMass;
		double Mb = rigidbodyB->m_fMass;

		// If the collision was already managed, then return:
		if (dot(vr, n) >= 0)
			return 0;

		Mat4 invIa = rigidbodyA->m_mCurrentInvInertiaTensor;
		Mat4 invIb = rigidbodyB->m_mCurrentInvInertiaTensor;

		Vec3 A = cross(invIa.transformVector(cross(xa, n)), xa);
		Vec3 B = cross(invIb.transformVector(cross(xb, n)), xb);

		// Result of the impulse:
		double J = -(1 + collisionFactor) * dot(vr, n) / (1 / Ma + 1 / Mb + dot(A + B, n));

		// Update va and vb for both rigidbodies:
		rigidbodyA->m_vLinearVelocity = va + J * n / Ma;
		rigidbodyB->m_vLinearVelocity = vb - J * n / Mb;

		// Update La and Lb for both rigidbodies:
		Vec3 La = rigidbodyA->m_vAngularMomentum;
		Vec3 Lb = rigidbodyB->m_vAngularMomentum;

		rigidbodyA->m_vAngularMomentum = La + cross(xa, J * n);
		rigidbodyB->m_vAngularMomentum = Lb - cross(xb, J * n);

		// Update the angular velocity of both rigidbodies:
		rigidbodyA->m_vAngularVelocity = invIa.transformVector(rigidbodyA->m_vAngularMomentum);
		rigidbodyB->m_vAngularVelocity = invIb.transformVector(rigidbodyB->m_vAngularMomentum);

		return J;
	}
}

void Rigidbody::correctPosition(Rigidbody* r, const SimulationParameters* params, Vec3 collisionPoint, Vec3 collisionNormal, double collisionDepth, double timestep)
{
	Vec3 n1 = collisionNormal;				// Vector from the fixed rigidbody to the given rigidbody
	Vec3 n2 = r->getAxisAlong(&n1);
	double rigidbodyHeight = norm(n2);		// Height of the rigidbody along n2
	n2 /= rigidbodyHeight;

	// Add friction to the part of the linear velocity orthogonal to the normal:
	Vec3 velocityAlongNormal = dot(r->m_vLinearVelocity, n1) * n1;
	r->m_vLinearVelocity = velocityAlongNormal + (1-params->objectFriction) * (r->m_vLinearVelocity - velocityAlongNormal);

	// For the angular velocity, the friction should be applied to the normal part:
	velocityAlongNormal = dot(r->m_vAngularVelocity, n1) * n1;
	r->m_vAngularVelocity = (1-params->objectFriction) * velocityAlongNormal + (r->m_vAngularVelocity - velocityAlongNormal);

	double dot_n1_n2 = dot(n1, n2);

	// If n1 and n2 are already aligned, we can only play on the position of the rigidbody to
	// correct the intersection:
	if (abs(dot_n1_n2 - 1) < 1e-5) {
		float posCorrection = min(collisionDepth, params->maxLinearCorrectionSpeed * timestep);
		r->m_vPosition += posCorrection * n1;
	}

	// Else, we have to rotate the rigidbody first, in order to minimize the collision depth, and
	// we can then correct the remaining error by translating the rigidbody:
	else {
		// This is the unit vector around which we have to turn
		Vec3 k = cross(n2, n1);
		k /= norm(k);

		// Vector from the rigidbody center to the collision point, without the part
		// along the rotation axis k:
		Vec3 OM = collisionPoint - r->m_vPosition;
		OM = OM - dot(OM, k) * k;

		double dot_OM_n1 = dot(OM, n1);

		// Compute the maximum value for h, that can be corrected only by rotating the rigidbody:
		double hMax = -rigidbodyHeight / 2 - dot_OM_n1;

		// If collisionDepth > hMax, we can't prevent the intersection just by rotating the 
		// rigidbody. We also need to translate it:
		if (collisionDepth > hMax) {
			// Rotating by this angle would align the rigidbody with the normal:
			double alphaMax = acos(dot_n1_n2);

			// Prevent the rotation from beign too big:
			alphaMax = min(alphaMax, params->maxAngularCorrectionSpeed * timestep);

			// Rotate the rigidbody of alphaMax around k:
			r->m_qRotation = (Quat(k, alphaMax) * r->m_qRotation).unit();

			// Then, correct the remaining error by translating the rigidbody:
			double posCorrection = min(collisionDepth - hMax, params->maxLinearCorrectionSpeed * timestep);
			r->m_vPosition += posCorrection * n1;
		}

		// Else, we can correct the collision just with a rotation of the rigidbody:
		else {
			double normOM = norm(OM);
			double alpha = acos(dot_OM_n1 / normOM) - acos((collisionDepth + dot_OM_n1) / normOM);

			// Prevent the rotation from beign too big:
			alpha = min(alpha, params->maxAngularCorrectionSpeed * timestep);

			if (dot(OM, cross(n1, k)) < 0)
				alpha = -alpha;

			r->m_qRotation = (Quat(k, alpha) * r->m_qRotation).unit();
		}
	}

	r->updateTransformMatrices();
	r->updateCurrentInertiaTensor();
}

void Rigidbody::correctPosition(Rigidbody* rigidbodyA, Rigidbody* rigidbodyB, const SimulationParameters* params, 
	Vec3 collisionPoint, Vec3 collisionNormal, double collisionDepth, double timestep)
{
	// Instead of correcting the position of one rigidbody, keeping the second one fixed, we correct the position
	// of both rigidbodies, depending on their mass:

	double sumMass = rigidbodyA->m_fMass + rigidbodyB->m_fMass;

	// The lightest rigidbody should be the one that is the most corrected, so if A.mass > B.mass, then h2 > h1:
	double h1 = rigidbodyB->m_fMass * collisionDepth / sumMass;
	double h2 = rigidbodyA->m_fMass * collisionDepth / sumMass;

	correctPosition(rigidbodyA, params, collisionPoint, -collisionNormal, h1, timestep);
	correctPosition(rigidbodyB, params, collisionPoint, collisionNormal, h2, timestep);
}
