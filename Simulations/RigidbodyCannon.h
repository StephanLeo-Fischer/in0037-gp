#ifndef RIGIDBODY_CANNON_h
#define RIGIDBODY_CANNON_h
#include "Rigidbody.h"

#define CANNON_ROTATION_SPEED 0.01
#define CANNON_MAX_ANGLE M_PI/2
#define CANNON_MIN_ANGLE 0.0
#define MIN_TIME_BETWEEN_FIRE 0.05

// This class is just used to represent a cannon to fire rigidbodies in the scene:
class RigidbodyCannon {
public:

	RigidbodyCannon(double startTime, double stopTime);

	void update(double timestep);

	void RotateUp();
	void RotateDown();

	void draw(DrawingUtilitiesClass* DUC);

	void reset();

	// Fire a new rigidbody from the cannon position, and return a pointer to it:
	Rigidbody* fireRigidbody(SimulationParameters* params);

	void addBezierPoint(Vec3 point, Vec3 derivative);

private:
	Vec3 color;	// Color of the cannon

	double currentTime;
	const double startTime;
	const double stopTime;

	Mat4 m_mTransformMatrix;

	Mat4 cannonBasis;
	Mat4 cannonPillar;
	Mat4 cannonBody;
	Mat4 cannonBack;

	float m_fLastTimeFire;
	float m_fFireAngle;
	Vec3 m_vFireCenter;
	Vec3 m_vFireDirection;

	// Bezier points and derivatives of the points, representing the curve followed by the cannon:
	vector<Vec3> bezierPoints;
	vector<Vec3> bezierDerivatives;

	// Update the transform matrix, given t in [0; 1]:
	void updateTransform(double t);

	// Compute the position of the bezierCurve at t (in [0; 1]), as well as the angle 
	// between the derivative of the curve and the x-axis:
	void bezier(double t, Vec3* position, double* angle);

	void RigidbodyCannon::createTransform(
		double positionX, double positionY, double positionZ,
		double rotationX, double rotationY, double rotationZ,
		double scaleX, double scaleY, double scaleZ, Mat4* transform);
};

#endif