#include "Simulator.h"
class Rigidbody
{
public:
    Rigidbody(float mass, Vec3 pos, Quat orientation, Vec3 scale);
    void timestepEuler(float timeStep);

    Mat4 calculateInertia();

    void applyTorque(Vec3 location, Vec3 force);
    void applyForce(Vec3 force);
    void addForce(Vec3 loc, Vec3 force);
    void clearForces();

    void updateTransformMatrices();
    void updateInertialTensor0s();
    void updateCurrentInertialTensor();
    std::string toString();
    bool manageCollision(Rigidbody* other, float c);

    Vec3 color;


    float m_fMass;
    Vec3 m_vPosition;
    Quat m_qOrientation;
    Mat4 m_mRotMat;
    Vec3 m_vScale;
    bool m_bIsKinematic;  // kinematic ~~ infinite mass
    Vec3 m_vLinearVelocity;
    Vec3 m_vAngularVelocity; 
    Vec3 m_vAngularMomentum;

    Vec3 m_vSumForces;
    Vec3 m_vTorque;

    Mat4 m_mTransformMatrix;  // scaleMat * rotMat * translatMat

    Mat4 m_mInertialTensor0;
    Mat4 m_mInvInertialTensor0;
    Mat4 m_mCurrentInvInertialTensor;
};

