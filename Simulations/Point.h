#include "Simulator.h"

class Point
{
public:
    // Constructor
    Point(Vec3 pos, Vec3 vel, Vec3 acc, Vec3 force, float mass, bool isFixed);

    Vec3 getPosition();
    Vec3 getVelocity();
    void setForce(Vec3 force);
    void setAcceleration(Vec3 acc);
    void setVelocity(Vec3 vel);
    void setPosition(Vec3 pos);
    std::string to_string();

//private: //nvmd ich muss eh alle getten und setten
    // vars
    Vec3 _pos;
    Vec3 _vel;
    Vec3 _acc;
    Vec3 _force;
    float _mass;
    bool _isFixed;
};

