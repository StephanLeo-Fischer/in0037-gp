#include "Spring.h"

Spring::Spring(int firstConnectedPoint, int secondedConnectedPoint, float initialLength, float stiffness, float damping) {
    _firstConnectedPoint = firstConnectedPoint;
    _secondedConnectedPoint = secondedConnectedPoint;
    _initialLength = initialLength;
    _stiffness = stiffness;
    _damping = damping;
}

int Spring::getIndexFirstConnectedPoint()
{
    return _firstConnectedPoint;
}

int Spring::getIndexSecondConnectedPoint()
{
    return _secondedConnectedPoint;
}

float Spring::getInitialLength()
{
    return _initialLength;
}

