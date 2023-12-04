#pragma once

#include <iostream>
#include <vector>
#include "util/vectorbase.h"
#include "util/matrixbase.h"
#include "util/quaternion.h"

using namespace DirectX;

class RigidBody
{
public:

	// #########################################################
	//           CONSTRUCTOR/DESCRUCTOR
	// #########################################################

	RigidBody();
	RigidBody(const GamePhysics::Vec3 center, const GamePhysics::Vec3 size, float mass);
	virtual ~RigidBody();

	// #########################################################
	//           METHODS
	// #########################################################

	static bool collide(RigidBody& body1, RigidBody& body2);
	
	void              addForce(const GamePhysics::Vec3 force, const GamePhysics::Vec3 where);
	void              addForceWorld(const GamePhysics::Vec3 force, const GamePhysics::Vec3 where);
	void              update(float deltaTime);
	GamePhysics::Mat4 computeInertiaTensorInverse(const GamePhysics::Vec3 size, float massInverse);
	std::string 	  toString();

	// #########################################################
	//           SETTERS
	// #########################################################

	void setCenter(const GamePhysics::Vec3  center)                  {m_vCenter = center;}
	void setVelocity(const GamePhysics::Vec3  velocity)              {m_vVelocity = velocity;}
	void setRotation(const GamePhysics::Quat  rotation)              {m_qRotation = rotation;}
	void setAngularVelocity(const GamePhysics::Vec3 angularVelocity) {m_vAngularVelocity = angularVelocity;}

	// #########################################################
	//           GETTERS
	// #########################################################

	std::vector<DirectX::XMVECTOR> getCorners();
	const GamePhysics::Vec3        getCenter()    const {return m_vCenter;}
	const GamePhysics::Vec3        getVelocity()  const {return m_vVelocity;}
	const GamePhysics::Vec3        getAngularV()  const {return m_vAngularVelocity;}
	const GamePhysics::Mat4        getWorld2Obj() const {return m_mWorldToObj;}
	const GamePhysics::Mat4        getObj2World() const {return m_mObjToWorld;}
	static const GamePhysics::Vec3 s_corners[8];

	// #########################################################
	//           ATTRIBUTES
	// #########################################################

	GamePhysics::Vec3 m_vCollisonPoint;
	GamePhysics::Vec3 m_vCollisioNormal;
	GamePhysics::Vec3 m_vTotalVelocity;
	GamePhysics::Vec3 m_vRelVelocity;

	GamePhysics::Vec3 m_vCenter;
	GamePhysics::Quat m_qRotation;
	GamePhysics::Vec3 m_vScale;
	GamePhysics::Vec3 m_vAngularVelocity;

	GamePhysics::Vec3 m_vVelocity;
	GamePhysics::Vec3 m_vMomentum;
	GamePhysics::Mat4 m_mInertiaTensorInverse;
	float             m_fMassInverse;

	GamePhysics::Mat4 m_mObjToWorld;
	GamePhysics::Mat4 m_mWorldToObj;
	GamePhysics::Mat4 m_mScaledObjToWorld;
	GamePhysics::Mat4 m_mWorldToScaledObj;

	GamePhysics::Vec3 m_vFrameForce;
	GamePhysics::Vec3 m_vFrameTorque;
};

