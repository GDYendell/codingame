#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

/**
 * Save the Planet.
 * Use less Fossil Fuel.
 **/
 
#define GRAVITY -3.711;
#define PI 3.14159265
 
typedef struct Demand {
    int power;
    int rotation;
} Demand;


class PIDController {

public:
    PIDController(int targetX, int targetY, int dT, double kP, double kI, double kD);
    Demand calculateDemand(int positionX, int positionY);

private:
    std::vector<int> _target;
    double _kP;
    double _kI;
    double _kD;
    double _dT;
    std::vector<int> _previous_error;
    std::vector<int> _integral_error;
};


PIDController::PIDController(int targetX, int targetY, int dT, double kP, double kI, double kD) :
    _target({targetX, targetY}), _kP(kP), _kI(kI), _kD(kD), _dT(dT), _previous_error({0, 0}), _integral_error({0, 0})
    {}

Demand PIDController::calculateDemand(int currentX, int currentY) {
    std::vector<int> pOut(2), iOut(2), dOut(2), power(2), error(2);
    int demandRotation, demandPower;

    error[0] = _target[0] - currentX;
    error[1] = _target[1] - currentY;

    std::cerr << "targetX: " << _target[0] << ", currentX: " << currentX << ", errorX: " << error[0] << std::endl;
    std::cerr << "targetY: " << _target[1] << ", currentY: " << currentY << ", errorY: " << error[1] << std::endl;

    pOut[0] = - error[0] * _kP;
    pOut[1] = error[1] * _kP - GRAVITY;

    // Add integral term
    _integral_error[0] += error[0];
    _integral_error[1] += error[1];
    iOut[0] += _integral_error[0] * _kI;
    iOut[1] += _integral_error[1] * _kI;

    // Add derivative term
    dOut[0] = - ((error[0] - _previous_error[0]) / _dT) * _kD;
    dOut[1] = ((error[1] - _previous_error[1]) / _dT) * _kD;
    
    _previous_error[0] = error[0];
    _previous_error[1] = error[1];
    
    power[0] = pOut[0] + iOut[0] + dOut[0];
    power[1] = pOut[1] + iOut[1] + dOut[1];
    
    if (power[1] < 0) {power[1] = 0;}

    std::cerr << "powerX: " << power[0] << " (" << pOut[0] << " + " << iOut[0] << " + " << dOut[0] << ")" << std::endl;
    std::cerr << "powerY: " << power[1] << " (" << pOut[1] << " + " << iOut[1] << " + " << dOut[1] << ")" << std::endl;
    
    demandRotation = atan((double) power[0] / (double) power[1]) * 180 / PI;
    if (power[0] == 0) {demandRotation = 0;}
    demandPower = sqrt(pow(power[0], 2) + pow(power[1], 2));

    if (error[1] > -500) {
        // Land
        demandPower = 4;
        demandRotation = 0;
    }
    if (demandPower > 4) {demandPower = 4;}
    std::cerr << "demandRotation: " << demandRotation << ", demandPower: " << demandPower << std::endl;

    return Demand{demandPower, demandRotation};
}


int main()
{
    int X;
    int Y;
    int HS; // the horizontal speed (in m/s), can be negative.
    int VS; // the vertical speed (in m/s), can be negative.
    int F; // the quantity of remaining fuel in liters.
    int R; // the rotation angle in degrees (-90 to 90).
    int P; // the thrust power (0 to 4).

    int landX; // X coordinate of a surface point. (0 to 6999)
    int landY; // Y coordinate of a surface point. By linking all the points together in a sequential fashion, you form the surface of Mars.
    int landingZoneLeft, landingZoneRight, landingZoneCentre, landingZoneHeight;

    int direction;

    Demand nextDemand;

    int N; // the number of points used to draw the surface of Mars.
    std::cin >> N; std::cin.ignore();
    std::vector<int> landscapeX(N), landscapeY(N);
    for (int i = 0; i < N; i++) {
        std::cin >> landX >> landY; std::cin.ignore();
        landscapeX[i] = landX;
        landscapeY[i] = landY;
        
        std::cerr << "(" << landX << ", " << landY << ")";
        
        if (landY == landscapeY[i - 1]) {
            landingZoneRight = landX;
            landingZoneLeft = landscapeX[i - 1];
            landingZoneCentre = (landingZoneLeft + landingZoneRight) / 2;
            landingZoneHeight = landY;
            std::cerr << " <-- Landing Zone";
        }
        std::cerr << std::endl;
    }

    PIDController controller = PIDController(landingZoneCentre, landingZoneHeight, 1, 0.002, 0.0, 0.1);

    // game loop
    while (1) {
        std::cin >> X >> Y >> HS >> VS >> F >> R >> P; std::cin.ignore();

        nextDemand = controller.calculateDemand(X, Y);

        std::cout << nextDemand.rotation << " " << nextDemand.power << std::endl;
    }
}
