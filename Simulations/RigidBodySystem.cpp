#include "RigidBodySystem.h"

RigidBodySystem::RigidBodySystem() :
	m_vRigidBodies()
{
}

RigidBodySystem::~RigidBodySystem()
{
}

void RigidBodySystem::scene(int sceneflag)
{
	// remove all rigid bodies
	m_vRigidBodies.clear();

	if (sceneflag == 0 || sceneflag == 1)
	{
		// Box center:(0,0,0)T, size:(1, 0.6, 0.5), mass: 2,
		m_vRigidBodies.emplace_back(GamePhysics::Vec3(0.0f, 0.0f, 0.0f), GamePhysics::Vec3(1.0f, 0.6f, 0.5f), 2.0f);

		// Initial orientation: rotated around Z by 90 degrees
		GamePhysics::Quat rotation(GamePhysics::Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5f);
		m_vRigidBodies.back().setRotation(rotation);
		m_vRigidBodies.back().update(0.0f);

		// The initial linear and angular velocity are both zero.
		m_vRigidBodies.back().setVelocity(GamePhysics::Vec3(0.0f, 0.0f, 0.0f));
		m_vRigidBodies.back().setAngularVelocity(GamePhysics::Vec3(0.0f, 0.0f, 0.0f));

		/*
		Simulate the rigid body for 1 step with an external force f = (1,1,0)T applied at world space
		position (0.3, 0.5, 0.25)^T. Compute the resulting linear and angular velocity and the world space
		velocity of point (-0.3, -0.5, -0.25)^T. (Ignore that the force position is not on the body.)
		*/
		GamePhysics::Vec3 force =  GamePhysics::Vec3(1.0f, 1.0f, 0.0f);
		GamePhysics::Vec3 fwhere = GamePhysics::Vec3(-0.3f, -0.5f, -0.25f);

		m_vRigidBodies.back().addForceWorld(force, fwhere);

		for (auto rb : m_vRigidBodies)
		{
			std::cout << "Initiate Table 2.1:\n" << rb.toString() << "\n";
		}

		// Demo 1: A simple one-step test
		if (sceneflag == 0) {
			update(2.0f);
			for (auto rb : m_vRigidBodies)
			{
				std::cout << rb.toString() << "\n";
			}
		}

		// Demo 2: Simple single-body simulation
		else if (sceneflag == 1) { 
			update(0.01);
		}

	}

	// Demo 3: Two-rigid-body collision scene
	else if (sceneflag == 2)
	{
		m_vRigidBodies.emplace_back(GamePhysics::Vec3(-0.1f, -0.2f, 0.1f), GamePhysics::Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		m_vRigidBodies.back().update(0.0f);
		m_vRigidBodies.emplace_back(GamePhysics::Vec3(0.0f, 0.2f, 0.0f), GamePhysics::Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		// Quat normal axis not used here
		m_vRigidBodies.back().setRotation(GamePhysics::Quat(GamePhysics::Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.25f));
		m_vRigidBodies.back().setVelocity(GamePhysics::Vec3(0.0f, -0.1f, 0.05f));
		m_vRigidBodies.back().update(0.0f);
	}
	
	/* Demo 4: Complex simulation
	 o Set up a simulation with at least four boxes.
	 o Provide interaction methods, e.g.by including extra forces.You can also add a ground floor(or
	   walls) and enable gravity so that all rigid bodies will collide with it. (Note that bodies will not
	   fully come to rest with this basic simulation algorithm.)
	*/
	else if (sceneflag == 3)
	{
		m_vRigidBodies.emplace_back(GamePhysics::Vec3(-0.1f, -0.2f, 0.1f), GamePhysics::Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		m_vRigidBodies.back().update(0.0f);
		m_vRigidBodies.emplace_back(GamePhysics::Vec3(0.0f, 0.2f, 0.0f), GamePhysics::Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		// Quat normal axis not used here
		m_vRigidBodies.back().setRotation(GamePhysics::Quat(GamePhysics::Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.25f));
		m_vRigidBodies.back().setVelocity(GamePhysics::Vec3(0.0f, -0.1f, 0.05f));
		m_vRigidBodies.back().update(0.0f);
		m_vRigidBodies.emplace_back(GamePhysics::Vec3(0.0f, 0.2f, 0.0f), GamePhysics::Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		// Quat normal axis not used here
		m_vRigidBodies.back().setRotation(GamePhysics::Quat(GamePhysics::Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.25f));
		m_vRigidBodies.back().setVelocity(GamePhysics::Vec3(0.0f, -0.1f, 0.05f));
		m_vRigidBodies.back().update(0.0f);
		m_vRigidBodies.emplace_back(GamePhysics::Vec3(0.0f, 0.2f, 0.0f), GamePhysics::Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		// Quat normal axis not used here
	}
}

void RigidBodySystem::moveTowards()
{
	auto rb_size = m_vRigidBodies.size();
	for (int i = 0; i < rb_size - 1; ++i)
	{
		GamePhysics::Vec3 vel = m_vRigidBodies[i + 1].getCenter() - m_vRigidBodies[i].getCenter();
		m_vRigidBodies[i].setVelocity(vel * 0.1f);
		m_vRigidBodies[i + 1].setVelocity(vel * -0.1f);
	}
}

void RigidBodySystem::addGlobalFrameForce(const GamePhysics::Vec3 force)
{
	for (RigidBody& rigidBody : m_vRigidBodies)
	{
		rigidBody.addForceWorld(force, DirectX::XMVectorZero());
	}
}

void RigidBodySystem::update(float deltaTime)
{

	for (RigidBody& rigidBody : m_vRigidBodies)
	{
		rigidBody.update(deltaTime);
	}
	for (size_t i = 0; i < m_vRigidBodies.size(); ++i)
	{
		for (size_t j = i + 1; j < m_vRigidBodies.size(); ++j)
		{
			if (RigidBody::collide(m_vRigidBodies[i], m_vRigidBodies[j]))
			{
			}
		}
	}
}