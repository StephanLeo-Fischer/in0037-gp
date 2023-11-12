#include "Point.h"

Point::Point(Vec3 pos, Vec3 vel, Vec3 acc, Vec3 force, float mass, bool isFixed) {
    this->_pos = pos;
    this->_vel = vel;
    this->_acc = acc;
    this->_force = force;
    this->_mass = mass;
    this->_isFixed = isFixed;
}

Vec3 Point::getPosition() {
    return _pos;
}

Vec3 Point::getVelocity() {
    return _vel;
}


void Point::setForce(Vec3 force) {
    _force = force;
}
void Point::setAcceleration(Vec3 acc) {
    _acc = acc;
}
void Point::setVelocity(Vec3 vel) {
    _vel = vel;
}
void Point::setPosition(Vec3 pos) {
    _pos = pos;
}

std::string Point::to_string() {
    std::string str = (_pos.toString() + "\t" + _vel.toString() + "\n");  // TODO: richtig implementieren, wenn nötig
    return str;
}
