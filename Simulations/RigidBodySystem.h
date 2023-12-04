#pragma once

#include "RigidBody.h"
#include <vector>
#include <AntTweakBar.h>
#include <iostream>
#include <cmath>

class RigidBodySystem
{
public:

	// #########################################################
	//           CONSTRUCTOR/DESCRUCTOR
	// #########################################################

	RigidBodySystem();
	virtual ~RigidBodySystem();

	// #########################################################
	//           METHODS
	// #########################################################

	void initTweakBar(TwBar* tweakBar);
	void scene(int sceneflag);
	void moveTowards();
	void addGlobalFrameForce(const GamePhysics::Vec3 force);
	void update(float deltaTime);

	// #########################################################
	//           ATTRIBUTES
	// #########################################################

	std::vector<RigidBody> m_vRigidBodies;
};