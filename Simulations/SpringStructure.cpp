#include "SpringStructure.h"

void SpringStructure::drawSprings(DrawingUtilitiesClass* DUC) {
	const Vec3 color = Vec3(1, 1, 0);

	DUC->beginLine();
	for (auto& s : m_vSprings) {
		// Compute the global position of the attach points of the spring:
		Vec3 globalAttach1 = m_vRigidbodies[s.i1]->transformLocalToGlobal(s.attach1);
		Vec3 globalAttach2 = m_vRigidbodies[s.i2]->transformLocalToGlobal(s.attach2);

		DUC->drawLine(globalAttach1, color, globalAttach2, color);
	}
	DUC->endLine();
}

void SpringStructure::addRigidbody(Rigidbody* rigidbody) {
	rigidbody->allowIdleState(false);
	m_vRigidbodies.push_back(rigidbody);
}

void SpringStructure::addSpring(int i1, int i2, Vec3 localAttach1, Vec3 localAttach2, float stiffness) {
	Vec3 globalAttach1 = m_vRigidbodies[i1]->transformLocalToGlobal(localAttach1);
	Vec3 globalAttach2 = m_vRigidbodies[i2]->transformLocalToGlobal(localAttach2);
	
	float length = norm(globalAttach1 - globalAttach2);
	addSpring(i1, i2, localAttach1, localAttach2, length, stiffness);
}

void SpringStructure::addSpring(int i1, int i2, Vec3 localAttach1, Vec3 localAttach2, float initialLength, float stiffness) {
	m_vSprings.push_back(Spring(i1, i2, localAttach1, localAttach2, initialLength, stiffness));
}

void SpringStructure::setExternalForce(Vec3 force) {
	m_vExternalForce = force;
}

void SpringStructure::updateForces() {
	// Clear the forces applied on the rigidbodies of the structure, and apply the 
	// external force to them:
	for (auto& r : m_vRigidbodies) {
		r->clearForces();
		r->addForce(m_vExternalForce);
	}

	// Compute the internal forces of the structure (from the springs):
	for (auto& s : m_vSprings) {
		Rigidbody* r1 = m_vRigidbodies[s.i1];
		Rigidbody* r2 = m_vRigidbodies[s.i2];

		// Compute the global position of the attach points of the spring:
		Vec3 globalAttach1 = r1->transformLocalToGlobal(s.attach1);
		Vec3 globalAttach2 = r2->transformLocalToGlobal(s.attach2);

		// Compute the force of the spring:
		Vec3 v = globalAttach2 - globalAttach1;
		float distance = norm(v);

		// Compute the force applied by the spring to r1:
		Vec3 force = s.m_fStiffness * (distance - s.m_fInitialLength) * v / distance;

		r1->addTorque(globalAttach1, force);
		r2->addTorque(globalAttach2, -force);
	}
}
