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

#define MAX_VX 20
#define MAX_VY 40
 
typedef struct Demand {
    int power;
    int rotation;
} Demand;


class PIDController {

public:
    PIDController(int startX, int startY, int targetX, int targetY, double kP, double kV);
    Demand calculateDemand(int currentX, int currentY, int currentVX, int currentVY, int currentRotation, int currentPower);

private:
    std::vector<int> _target;
    double _kP;
    double _kV;
};


PIDController::PIDController(int startX, int startY, int targetX, int targetY, double kP, double kV) :
        _target({targetX, targetY}), _kP(kP), _kV(kV) {}

Demand PIDController::calculateDemand(int currentX, int currentY, int currentVX, int currentVY, int currentRotation, int currentPower) {
    std::vector<int> pOut(2), vOut(2), power(2), error(2);
    int demandRotation, demandPower;

    error[0] = _target[0] - currentX;
    error[1] = _target[1] - currentY;

    std::cerr << "targetX: " << _target[0] << ", currentX: " << currentX << ", errorX: " << error[0] << std::endl;
    std::cerr << "targetY: " << _target[1] << ", currentY: " << currentY << ", errorY: " << error[1] << std::endl;

    // Add proportional (displacement) term - Moves lander towards target
    pOut[0] = error[0] * _kP;
    pOut[1] = error[1] * _kP - GRAVITY;

    // Add velocity term - Slows lander to zero velocity
    vOut[0] = - currentVX * _kV;
    vOut[1] = - currentVY * _kV;

    power[0] = - (pOut[0] + vOut[0]);  // X power is inverted compared to axis definition
    power[1] = pOut[1] + vOut[1];

    power[1] = std::max(power[1], 0);  // If we want to drop, set vertical power to zero and leave it to gravity

    std::cerr << "powerX: " << power[0] << " (" << pOut[0] << " + " << vOut[0] << ")" << std::endl;
    std::cerr << "powerY: " << power[1] << " (" << pOut[1] << " + " << vOut[1] << ")" << std::endl;

    // Calculate power and rotation demand
    demandRotation = atan((double) power[0] / (double) power[1]) * 180 / PI;
    demandPower = sqrt(pow(power[0], 2) + pow(power[1], 2));

    // If we are close to the ground and VX is OK, prepare to land
    if (error[1] > -500 && abs(currentVX) <= MAX_VX) {
        demandPower = 4;
        demandRotation = 0;
    }

    // Nerf power based on lag between demand and actual rotation to avoid accelerating in the wrong direction
    if (currentRotation != 0 && abs(demandRotation - currentRotation) > 150) {
        demandPower -= abs(demandRotation - currentRotation) / 30;
    }

    // Apply limits
    demandPower = std::max(demandPower, 0);
    demandPower = std::min(demandPower, 4);

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
        std::cin >> landscapeX[i] >> landscapeY[i]; std::cin.ignore();

        std::cerr << "(" << landscapeX[i] << ", " << landscapeY[i] << ")";

        if (landscapeY[i] == landscapeY[i - 1]) {
            landingZoneRight = landscapeX[i];
            landingZoneLeft = landscapeX[i - 1];
            landingZoneCentre = (landingZoneLeft + landingZoneRight) / 2;
            landingZoneHeight = landscapeY[i];
            std::cerr << " <-- Landing Zone";
        }
        std::cerr << std::endl;
    }

    std::cin >> X >> Y >> HS >> VS >> F >> R >> P; std::cin.ignore();
    PIDController controller = PIDController(X, Y, landingZoneCentre, landingZoneHeight, 0.002, 0.1);

    // game loop
    while (1) {
        std::cerr << X << ", " << Y << ", " << HS << ", " << VS << ", " << F << ", " << R << ", " << P << std::endl;

        nextDemand = controller.calculateDemand(X, Y, HS, VS, R, P);
        std::cout << nextDemand.rotation << " " << nextDemand.power << std::endl;

        std::cin >> X >> Y >> HS >> VS >> F >> R >> P; std::cin.ignore();
    }
}
