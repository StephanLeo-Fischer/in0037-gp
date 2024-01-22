#ifndef RIGIDBODY_CANNON_h
#define RIGIDBODY_CANNON_h
#include "Rigidbody.h"

// This class is just used to represent a cannon to fire rigidbodies in the scene:
class RigidbodyCannon {
public:

	RigidbodyCannon(double startTime, double stopTime);

	void update(double timestep);

	void draw(DrawingUtilitiesClass* DUC);

	void reset();

	// Fire a new rigidbody from the cannon position, and return a pointer to it:
	Rigidbody* fireRigidbody(SimulationParameters* params);

	void addBezierPoint(Vec3 point);

	void computeBezierCurve();

	~RigidbodyCannon() {
		delete [] binomialCoefficients;
	}

private:
	Vec3 color;	// Color of the cannon

	double currentTime;
	const double startTime;
	const double stopTime;

	Mat4 transformMatrix;

	Mat4 cannonBasis;
	Mat4 cannonPillar;
	Mat4 cannonBody;
	Mat4 cannonBack;

	Vec3 fireCenter;
	Vec3 fireDirection;

	// List of binomial coefficients, used to compute the bezier curve:
	int* binomialCoefficients;

	// Bezier points representing the curve followed by the cannon:
	vector<Vec3> bezierPoints;

	// Update the transform matrix, given t in [0; 1]:
	void updateTransform(double t);

	// Compute the list of binomial coefficients (allocated on the heap):
	void computeBinomialCoefficients(int n);

	// Compute the position of the bezierCurve at t (in [0; 1]), as well as the angle 
	// between the derivative of the curve and the x-axis:
	void bezier(double t, Vec3* position, double* angle);

	void RigidbodyCannon::createTransform(
		double positionX, double positionY, double positionZ,
		double rotationX, double rotationY, double rotationZ,
		double scaleX, double scaleY, double scaleZ, Mat4* transform);
};

#endif