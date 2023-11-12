#include "Simulator.h"


class Spring
{
public:
    // Constructor
    Spring(int firstConnectedPoint, int secondedConnectedPoint, float initialLength, float stiffness, float damping);

    int getIndexFirstConnectedPoint();
    int getIndexSecondConnectedPoint();
    float getInitialLength();

private:
    // vars
    int _firstConnectedPoint;
    int _secondedConnectedPoint;
    float _initialLength;
    float _stiffness;
    float _damping;
};

