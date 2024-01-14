#ifndef SPRING_STRUCTURE_h
#define SPRING_STRUCTURE_h
#include "Rigidbody.h"

// A spring structure is a structure containing a list of rigidbodies, 
// and a list of springs between them
class SpringStructure {
public:
	// SpringStructure();

private:
	struct Spring {
		// Indices (in the structure) of the rigidbodies attached by 
		// this spring:
		int i1, i2;

		// Coordinate of the attach point in the referential of the
		// rigidbodies:
		Vec3 attach1;
		Vec3 attach2;

		// Parameters of the spring:
		float m_fInitialLength;
		float m_fStiffness;

		Spring(int i1, int i2, Vec3 attach1, Vec3 attach2, float initialLength, float stiffness) :
			i1(i1),
			i2(i2),
			attach1(attach1),
			attach2(attach2),
			m_fInitialLength(initialLength),
			m_fStiffness(stiffness) {}
	};

	// External force applied to all the rigidbodies:
	Vec3 m_vExternalForce;

	vector<Rigidbody> rigidbodies;
	vector<Spring> springs;
};

#endif