#include "RigidbodyCannon.h"

RigidbodyCannon::RigidbodyCannon(double startTime, double stopTime) :
	color(0.2),
	currentTime(0), startTime(startTime), stopTime(stopTime) {

	// Create the cannon model:
	cannonBasis.initScaling(0.5, 0.05, 0.5);
	createTransform(0, 0.15, 0, 0, 0, 0, 0.1, 0.3, 0.1, &cannonPillar);
	createTransform(0, 0.3, 0, 70, 0, 0, 0.15, 0.4, 0.15, &cannonBody);
	createTransform(0, 0.279, -0.057, 70, 0, 0, 0.2, 0.2, 0.2, &cannonBack);
}

void RigidbodyCannon::draw(DrawingUtilitiesClass* DUC)
{
	// Draw the cannon:
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, color);
	DUC->drawRigidBody(cannonBasis * transformMatrix);
	DUC->drawRigidBody(cannonPillar * transformMatrix);
	DUC->drawRigidBody(cannonBody * transformMatrix);
	DUC->drawRigidBody(cannonBack * transformMatrix);

	// Draw the path followed by the cannon:
	Vec3 lineColor = Vec3(1, 0, 0);

	DUC->beginLine();

	// TESTING:
	DUC->drawLine(fireCenter, Vec3(1, 1, 0), fireCenter + fireDirection, Vec3(1, 1, 0));

	double angle;
	Vec3 current;
	Vec3 previous = bezierPoints[0];
	for (int i = 1; i <= 100; i++) {
		bezier(i / 100.0f, &current, NULL);

		DUC->drawLine(previous, lineColor, current, lineColor);
		previous = current;
	}

	DUC->endLine();
}

void RigidbodyCannon::update(double timestep) {
	currentTime += timestep;
	
	if (currentTime >= startTime && currentTime <= stopTime)
		updateTransform((currentTime - startTime) / (stopTime - startTime));
}

void RigidbodyCannon::reset() {
	currentTime = 0;
	updateTransform(0);
}

Rigidbody* RigidbodyCannon::fireRigidbody(SimulationParameters* params)
{
	Rigidbody* bullet = new Rigidbody("Cannon_bullet", params, 1, fireCenter, Vec3(0.0), Vec3(0.1));
	bullet->setLinearVelocity(10 * fireDirection);
	bullet->color = Vec3(1, 0, 0);

	return bullet;
}

void RigidbodyCannon::addBezierPoint(Vec3 point, Vec3 derivative) {
	bezierPoints.push_back(point);
	bezierDerivatives.push_back(derivative);
}

void RigidbodyCannon::updateTransform(double t) {
	
	// Get the position and angle of the cannon:
	Vec3 position;
	double angle;
	bezier(t, &position, &angle);

	// Update the transformation matrix of the cannon:
	Mat4 translation;
	translation.initTranslation(position.x, position.y, position.z);

	transformMatrix.initRotationY(-angle * 180 / M_PI);
	transformMatrix *= translation;

	const float fireAngle = 0.35;

	angle += M_PI_2;
	fireCenter = position + Vec3(0.27 * cos(angle), 0.4, 0.27 * sin(angle));
	fireDirection = Vec3(cos(angle)*cos(fireAngle), sin(fireAngle), sin(angle) * cos(fireAngle));
}

void RigidbodyCannon::bezier(double t, Vec3* position, double* angle) {
	int n = bezierPoints.size() - 1;
	int i = min((int) (t * n), n-1);
	t = n * t - i;

	// Create a bezier curve between these 4 points:
	Vec3 points[] = {bezierPoints[i], bezierPoints[i] + bezierDerivatives[i],
					bezierPoints[i+1] - bezierDerivatives[i+1], bezierPoints[i + 1]};

	// Binomial coefficients used to compute the bezier curve:
	int binom[] = { 1, 3, 3, 1 };

	if (position != NULL) {
		// Compute the position on the bezierCurve:
		*position = 0;
		for (int k = 0; k <= 3; k++) {
			double factor = binom[k] * pow(t, k) * pow(1 - t, 3 - k);
			*position += factor * points[k];
		}
	}

	if (angle != NULL) {
		// Compute the derivative of the curve:
		Vec3 derivative = 0.0;
		for (int k = 0; k < 3; k++) {
			double factor = (k + 1) * binom[k + 1] * pow(t, k) * pow(1 - t, 2 - k);
			derivative += factor * (points[k + 1] - points[k]);
		}

		// Ignore the part of the derivative along the y-axis:
		derivative.y = 0;

		// Compute the angle between the derivative of the curve and the x-axis:
		*angle = acos(derivative.x / norm(derivative));

		if (derivative.z < 0)
			*angle = 2 * M_PI - *angle;
	}
}

// Create a transformation matrix (scaleMat * rotMat * translatMat):
void RigidbodyCannon::createTransform(
	double positionX, double positionY, double positionZ, 
	double rotationX, double rotationY, double rotationZ, 
	double scaleX, double scaleY, double scaleZ, Mat4* transform)
{
	Mat4 tmp;

	// Scaling:
	transform->initScaling(scaleX, scaleY, scaleZ);
	
	// Rotation:
	tmp.initRotationXYZ(rotationX, rotationY, rotationZ);
	*transform *= tmp;

	// Translation:
	tmp.initTranslation(positionX, positionY, positionZ);
	*transform *= tmp;
}
