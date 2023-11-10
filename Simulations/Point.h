#include "Simulator.h"

class Point
{
public:
    // Constructor
    Point(Vec3 pos, Vec3 vel, Vec3 acc, Vec3 force, float mass, bool isFixed);

    Vec3 getPosition();

private:
    // vars
    Vec3 _pos;
    Vec3 _vel;
    Vec3 _acc;
    Vec3 _force;
    float _mass;
    bool _isFixed;
};

