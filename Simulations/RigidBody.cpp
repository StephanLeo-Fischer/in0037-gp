#pragma once

#include "RigidBody.h"

bool RigidBody::collide(RigidBody& body0, RigidBody& body1)
{
	return false;
}

std::vector <DirectX::XMVECTOR> RigidBody::getCorners()
{
	//const DirectX::XMVECTOR centerWorld = XMVector3Transform(DirectX::XMVectorZero(), m_mObjToWorld.toDirectXMatrix());
	//const int edge_size = 3;
	//std::vector<DirectX::XMVECTOR> edges[edge_size];
	//for (int i = 0; i < edge_size; ++i)
	//	edges[i]; //= normalze;
	std::vector<DirectX::XMVECTOR> results;
	//results.push_back(centerWorld - edges[0] - edges[1] - edges[2]);
	//results.push_back(centerWorld + edges[0] - edges[1] - edges[2]);
	//results.push_back(centerWorld - edges[0] + edges[1] - edges[2]);
	//results.push_back(centerWorld + edges[0] + edges[1] - edges[2]); 
	//results.push_back(centerWorld - edges[0] - edges[1] + edges[2]);
	//results.push_back(centerWorld + edges[0] - edges[1] + edges[2]); 
	//results.push_back(centerWorld - edges[0] + edges[1] + edges[2]); 
	//results.push_back(centerWorld + edges[0] + edges[1] + edges[2]); 
	return results;
}

RigidBody::RigidBody() :
	m_vCenter(),
	m_qRotation(0, 0, 0, 1),
	m_vScale(GamePhysics::Vec3(1,1,1)),
	m_vVelocity(GamePhysics::Vec3(0,0,0)),
	m_vMomentum(GamePhysics::Vec3(0,0,0)),
	m_fMassInverse(1.0f),
	m_mInertiaTensorInverse(computeInertiaTensorInverse(m_vScale, m_fMassInverse)),
	m_mObjToWorld(),
	m_mScaledObjToWorld(),
	m_mWorldToScaledObj(),
	m_vFrameForce(),
	m_vFrameTorque()
{
}

RigidBody::RigidBody(const GamePhysics::Vec3 center, const GamePhysics::Vec3 size, float mass) :
	m_vCenter(center),
	m_qRotation(0, 0, 0, 1),
	m_vScale(size),
	m_vVelocity(),
	m_vMomentum(),
	m_fMassInverse(1.0f / mass),
	m_mInertiaTensorInverse(computeInertiaTensorInverse(m_vScale, m_fMassInverse)),
	m_mObjToWorld(),
	m_mScaledObjToWorld(),
	m_mWorldToScaledObj(),
	m_vFrameForce(),
	m_vFrameTorque()
{
}

RigidBody::~RigidBody()
{
}

void RigidBody::addForce(const GamePhysics::Vec3 force, const GamePhysics::Vec3 where)
{
	m_vFrameForce += force;
	m_vFrameTorque += GamePhysics::cross(where, force);
}

void RigidBody::addForceWorld(const GamePhysics::Vec3 force, const GamePhysics::Vec3 where)
{
	addForce(force, where - m_vCenter);
}

void RigidBody::update(float deltaTime)
{
	m_vCenter += deltaTime * m_vVelocity;
	m_vVelocity += deltaTime * m_vFrameForce * m_fMassInverse;
	m_vMomentum += deltaTime * m_vFrameTorque;

	GamePhysics::Mat4 rotation = m_qRotation.getRotMat();
	GamePhysics::Mat4 rotationTranspose(rotation);
	rotationTranspose.transpose();

	GamePhysics::Mat4 currentInertiaTensorInverse = rotation * m_mInertiaTensorInverse * rotationTranspose;

	m_vAngularVelocity = currentInertiaTensorInverse.transformVector(m_vMomentum);

	GamePhysics::Quat m_vAngularVelocity_tmp(m_vAngularVelocity.x, m_vAngularVelocity.y, m_vAngularVelocity.z, 0);
	m_qRotation += deltaTime / 2.0f * m_vAngularVelocity_tmp * m_qRotation;
	m_qRotation = m_qRotation.unit();

	m_vFrameForce = DirectX::XMVectorZero();
	m_vFrameTorque = DirectX::XMVectorZero();

	GamePhysics::Mat4 matrix;
	matrix.initTranslation(m_vCenter.x, m_vCenter.y, m_vCenter.z);
	m_mScaledObjToWorld = rotation * matrix;
	m_mWorldToScaledObj = m_mScaledObjToWorld.inverse();
	matrix.initScaling(m_vScale.x, m_vScale.y, m_vScale.z);
	m_mObjToWorld = matrix * m_mScaledObjToWorld;
	m_mWorldToObj = m_mObjToWorld.inverse();
}

GamePhysics::Mat4 RigidBody::computeInertiaTensorInverse(const GamePhysics::Vec3 size, float massInverse)
{
	// assumption: homogenous cuboid
	const float x = size.x;
	const float y = size.y;
	const float z = size.z;
	const float xSquared = x * x;
	const float ySquared = y * y;
	const float zSquared = z * z;

	return GamePhysics::Mat4(
		12.0f * massInverse / (ySquared + zSquared), 0.0f, 0.0f, 0.0f,
		0.0f, 12.0f * massInverse / (xSquared + zSquared), 0.0f, 0.0f,
		0.0f, 0.0f, 12.0f * massInverse / (xSquared + ySquared), 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

std::string RigidBody::toString()
{
	std::string result = "";
	result += "Center:           " + m_vCenter.toString() + "\n";
	result += "Velocity:         " + m_vVelocity.toString() + "\n";
	result += "Angular Velocity: " + m_vAngularVelocity.toString() + "\n";
	result += "Scale:            " + m_vScale.toString() + "\n";
	result += "FrameForce:       " + m_vFrameForce.toString() + "\n";
	result += "FrameTorque:      " + m_vFrameTorque.toString() + "\n";
	result += "Mass Inverse:     " + std::to_string(m_fMassInverse) + "\n";
	return result;
}

const GamePhysics::Vec3 RigidBody::s_corners[8] =
{
	GamePhysics::Vec3(-0.5f, -0.5f, -0.5f),
	GamePhysics::Vec3(0.5f, -0.5f, -0.5f),
	GamePhysics::Vec3(-0.5f, 0.5f, -0.5f),
	GamePhysics::Vec3(0.5f, 0.5f, -0.5f),
	GamePhysics::Vec3(-0.5f, -0.5f, 0.5f),
	GamePhysics::Vec3(0.5f, -0.5f, 0.5f),
	GamePhysics::Vec3(-0.5f, 0.5f, 0.5f),
	GamePhysics::Vec3(0.5f, 0.5f, 0.5f)
};