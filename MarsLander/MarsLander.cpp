#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

/**
 * Save the Planet.
 * Use less Fossil Fuel.
 **/
 
#define GRAVITY -3.711
#define PI 3.14159265

#define MAX_POWER 4.0
#define MAX_VX 20
#define MAX_VY 40
#define LANDING_X_THRESHOLD 10.0
#define LANDING_Y_THRESHOLD 1000.0

template <typename T> int signOf(T value) {
    return (value > 0) - (value < 0);
}
 
typedef struct Demand {
    double power;
    double rotation;
} Demand;


Demand xyToDemand(double X, double Y) {
    // Calculate power and rotation demand
    double power, rotation;
    power = sqrt(pow(X, 2) + pow(Y, 2));
    rotation = atan((double) X / (double) Y) * 180 / PI;
    // Apply limits
    power = std::max(power, 0.0);
    power = std::min(power, MAX_POWER);
    return Demand{power, rotation};
}


class PIDController {

public:
    PIDController(int startX, int startY, int targetX, int targetY, double kP, double kV);
    Demand calculateDemand(int currentX, int currentY, int currentVX, int currentVY, int currentRotation, int currentPower, bool inLandingZone);

private:
    std::vector<int> _target;
    double _kP;
    double _kV;
};


PIDController::PIDController(int startX, int startY, int targetX, int targetY, double kP, double kV) :
        _target({targetX, targetY}), _kP(kP), _kV(kV) {}

Demand PIDController::calculateDemand(int currentX, int currentY, int currentVX, int currentVY, int currentRotation, int currentPower, bool inLandingZone) {
    std::vector<double> pOut(2), vOut(2), power(2), error(2);

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

    power[1] = std::max(power[1], 0.0);  // If we want to drop, set vertical power to zero and leave it to gravity

    std::cerr << "powerX: " << power[0] << " (" << pOut[0] << " + " << vOut[0] << ")";
    std::cerr << " powerY: " << power[1] << " (" << pOut[1] << " + " << vOut[1] << ")" << std::endl;

    // Calculate power and rotation demand
    Demand demand = xyToDemand(power[0], power[1]);

    // If we are far from the landing zone, maintain height and go slow - set y force to -GRAVITY
    if (fabs(error[0]) > std::max(fabs(error[1] / 1.05), LANDING_Y_THRESHOLD) &&  power[1] < fabs(GRAVITY)) {
        power[1] = fabs(GRAVITY);
        if (sqrt(pow(power[0], 2) + pow(power[1], 2)) > MAX_POWER) {
            power[0] = signOf(power[0]) * sqrt(pow(MAX_POWER, 2) - pow(power[1], 2));
        }
        std::cerr << "[DISTANCE OVERRIDE] powerX: " << power[0];
        std::cerr << " powerY: " << power[1] << std::endl;
        demand = xyToDemand(power[0], power[1]);
    }

    // If we are in the landing zone and VX is OK, prepare to land
    if (inLandingZone && fabs(error[1]) < LANDING_Y_THRESHOLD && fabs(currentVX) <= MAX_VX) {
        std::cerr << "[LANDING OVERRIDE]" << std::endl;
        if (currentVY <= -MAX_VY) {
            demand.power = MAX_POWER;
        } else {
            demand.power = MAX_POWER - 1;
        }
        demand.rotation = 0;
    }

    // Nerf power based on lag between demand and actual rotation to avoid accelerating in the wrong direction
    if (currentRotation != 0 && fabs(demand.rotation - currentRotation) > 150) {
        demand.power -= fabs(demand.rotation - currentRotation) / 30;
    }

    // Apply limits
    demand.power = std::max(demand.power, 0.0);
    demand.power = std::min(demand.power, MAX_POWER);

    std::cerr << "demand.rotation: " << demand.rotation << ", demand.power: " << demand.power << std::endl;
    return demand;
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
    bool inLandingZone = false;

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

        inLandingZone = X > landingZoneLeft + LANDING_X_THRESHOLD && X < landingZoneRight - LANDING_X_THRESHOLD;
        nextDemand = controller.calculateDemand(X, Y, HS, VS, R, P, inLandingZone);
        std::cout << round(nextDemand.rotation) << " " << round(nextDemand.power) << std::endl;

        std::cin >> X >> Y >> HS >> VS >> F >> R >> P; std::cin.ignore();
    }
}
