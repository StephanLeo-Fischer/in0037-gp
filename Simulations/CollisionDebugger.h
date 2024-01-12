#ifndef COLLISION_DEBUGGER_h
#define COLLISION_DEBUGGER_h
#include "Simulator.h"
#include "collisionDetect.h"

class CollisionDebugger {
public:
	// Constructor:
	CollisionDebugger() {
		setTransform(&transformA, Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(10, 0.1, 10));
		setTransform(&transformB, Vec3(0, 2, 0), Vec3(0, 0, 0), Vec3(1, 1, 1));

		collision.isValid = false;
	}
	
	void update() {
		Mat4 tmp;
		tmp.initId();

		const float MOVE_SPEED = 0.01f;
		const float ROT_SPEED = 1;

		bool wasUpdated = true;

		// Keycodes: 'A' - 'Z' (0x41 - 0x5A)
		if (DXUTIsKeyDown(0x51))
			tmp.initTranslation(-MOVE_SPEED, 0, 0);		// Q
		else if (DXUTIsKeyDown(0x44))
			tmp.initTranslation(+MOVE_SPEED, 0, 0);		// D
		else if (DXUTIsKeyDown(0x53))
			tmp.initTranslation(0, 0, -MOVE_SPEED);		// S
		else if (DXUTIsKeyDown(0x5A))
			tmp.initTranslation(0, 0, +MOVE_SPEED);		// Z
		else if (DXUTIsKeyDown(0x57))
			tmp.initTranslation(0, -MOVE_SPEED, 0);		// W
		else if (DXUTIsKeyDown(0x58))
			tmp.initTranslation(0, +MOVE_SPEED, 0);		// X

		else if (DXUTIsKeyDown(VK_DOWN))
			tmp.initRotationX(ROT_SPEED);
		else if (DXUTIsKeyDown(VK_UP))
			tmp.initRotationX(-ROT_SPEED);
		else if (DXUTIsKeyDown(VK_LEFT))
			tmp.initRotationY(ROT_SPEED);
		else if (DXUTIsKeyDown(VK_RIGHT))
			tmp.initRotationY(-ROT_SPEED);
		else
			wasUpdated = false;

		transformB = tmp * transformB;

		if (wasUpdated) {
			collision = checkCollisionSAT(transformA, transformB);
		}
	}

	void draw(DrawingUtilitiesClass* DUC) {
		update();
		
		if(collision.isValid)
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(0.8, 0, 0));
		else
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(0.5));

		DUC->drawRigidBody(transformA);
		DUC->drawRigidBody(transformB);

		// Draw the intersection:
		if (collision.isValid) {
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(1, 1, 0));
			DUC->drawSphere(collision.collisionPointWorld, Vec3(0.01));

			DUC->beginLine();
			DUC->drawLine(
				collision.collisionPointWorld, Vec3(0.0), 
				collision.collisionPointWorld + collision.normalWorld, Vec3(1, 0, 0));
			DUC->endLine();
		}
	}

private:
	Mat4 transformA;
	Mat4 transformB;

	CollisionInfo collision;

	// Set the position, rotation and scale of the given transform:
	void setTransform(Mat4* transform, Vec3 position, Vec3 rotation, Vec3 scale) {
		// transformMat = scaleMat * rotMat * translatMat:
		Mat4 tmp;

		// Scaling:
		transform->initScaling(scale.x, scale.y, scale.z);

		// Rotation:
		tmp.initRotationXYZ(rotation.x, rotation.y, rotation.z);
		*transform = *transform * tmp;

		// Translation:
		tmp.initTranslation(position.x, position.y, position.z);
		*transform = *transform * tmp;
	}
};
#endif