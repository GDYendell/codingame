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
#define LANDING_Y_THRESHOLD 1500.0
#define OBSTACLE_CLEARANCE 5.0

template <typename T> int signOf(T value) {
    return (value > 0) - (value < 0);
}

bool betweenPoints(int test, int A, int B) {
    return (A < test && test < B) ||
           (B < test && test < A);
}
 
typedef struct Demand {
    double powerX;
    double powerY;
    double power;
    double rotation;
} Demand;


Demand xyToDemand(double X, double Y) {
    // Calculate power and rotation demand
    double power, rotation;
    power = sqrt(pow(X, 2) + pow(Y, 2));
    rotation = atan((double) X / (double) Y) * 180 / PI;

    Demand demand = Demand{X, Y, power, rotation};

    // Apply limits
    if (power > MAX_POWER) {
        demand.power = MAX_POWER;
        demand.powerX = demand.power * sin(rotation * PI / 180);
        demand.powerY = demand.power * cos(rotation * PI / 180);
        std::cerr << "[POWER CAP OVERRIDE] powerX: " << demand.powerX << " powerY: " << demand.powerY << std::endl;
    }

    return demand;
}


class PIDController {

public:
    PIDController(int targetX, int targetY, std::vector<std::vector<int>> obstacles, double kPX, double kVX, double kPY, double kVY);
    Demand calculateDemand(int currentX, int currentY, int currentVX, int currentVY, int currentRotation, int currentPower, bool inLandingZone);

private:
    int _targetX;
    int _targetY;
    std::vector<std::vector<int>> _obstacles;
    double _kPX;
    double _kVX;
    double _kPY;
    double _kVY;
};


PIDController::PIDController(int targetX, int targetY, std::vector<std::vector<int>> obstacles, double kPX, double kVX, double kPY, double kVY) :
        _targetX(targetX), _targetY(targetY), _obstacles(obstacles), _kPX(kPX), _kVX(kVX), _kPY(kPY), _kVY(kVY) {}

Demand PIDController::calculateDemand(int currentX, int currentY, int currentVX, int currentVY, int currentRotation, int currentPower, bool inLandingZone) {
    double errorX, errorY, pOutX, pOutY, vOutX, vOutY, powerX, powerY;

    errorX = _targetX - currentX;
    int targetY = _targetY;
    for (auto &obstacle : _obstacles) {  // Override target Y to avoid obstacles
        if (betweenPoints(obstacle[0], currentX, _targetX)) {
            if (obstacle[1] + OBSTACLE_CLEARANCE > targetY && obstacle[1] > currentY / 1.5) {
                targetY = obstacle[1] + OBSTACLE_CLEARANCE;
                std::cerr << "[OBSTACLE OVERRIDE] targetY: " << targetY << std::endl;
            }
        }
    }
    errorY = targetY - currentY;

    std::cerr << "targetX: " << _targetX << ", currentX: " << currentX << ", errorX: " << errorX << std::endl;
    std::cerr << "targetY: " << targetY << ", currentY: " << currentY << ", errorY: " << errorY << std::endl;

    // Add proportional (displacement) term - Moves lander towards target
    pOutX = errorX * _kPX;
    pOutY = errorY * _kPY - GRAVITY;

    // Add velocity term - Slows lander to zero velocity
    vOutX = - currentVX * _kVX;
    vOutY = - currentVY * _kVY;

    powerX = - (pOutX + vOutX);  // X power is inverted compared to axis definition
    powerY = pOutY + vOutY;

    powerY = std::max(powerY, 0.0);  // If we want to drop, set vertical power to zero and leave it to gravity

    std::cerr << "powerX: " << powerX << " (" << pOutX << " + " << vOutX << ")";
    std::cerr << " powerY: " << powerY << " (" << pOutY << " + " << vOutY << ")" << std::endl;

    // Calculate power and rotation demand
    Demand demand = xyToDemand(powerX, powerY);

    // If we are far from the landing zone, maintain height and go slow - set y force to -GRAVITY
    if (fabs(errorX) > std::max(5 * fabs(errorY), LANDING_Y_THRESHOLD) &&  demand.powerY < fabs(GRAVITY)) {
        powerY = fabs(GRAVITY);
        if (sqrt(pow(demand.powerX, 2) + pow(powerY, 2)) > MAX_POWER) {
            powerX = signOf(demand.powerX) * sqrt(pow(MAX_POWER, 2) - pow(powerY, 2));
        }
        std::cerr << "[DISTANCE OVERRIDE] powerX: " << powerX << " powerY: " << powerY << std::endl;
        demand = xyToDemand(powerX, powerY);
    }

    // If we are in the landing zone and VX is OK, prepare to land
    if (inLandingZone && fabs(errorY) < LANDING_Y_THRESHOLD && fabs(currentVX) <= MAX_VX) {
        std::cerr << "[LANDING OVERRIDE]" << std::endl;
        if (currentVY <= -MAX_VY) {
            demand.power = MAX_POWER;
        } else {
            demand.power = abs(currentVY) / 10;
        }
        demand.rotation = 0;
    }

    // Nerf power based on lag between demand and actual rotation to avoid accelerating in the wrong direction
    if (currentRotation != 0 && fabs(demand.rotation - currentRotation) > 150) {
        std::cerr << "[ROTATION LAG OVERRIDE]" << std::endl;
        demand.power -= fabs(demand.rotation - currentRotation) / 30;
        demand.power = std::max(demand.power, 0.0);
    }

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

    Demand nextDemand;

    int N; // the number of points used to draw the surface of Mars.
    std::cin >> N; std::cin.ignore();
    std::vector<std::vector<int>> landscape(N, std::vector<int>(2));
    for (int i = 0; i < N; i++) {
        std::cin >> landscape[i][0] >> landscape[i][1]; std::cin.ignore();
        std::cerr << "(" << landscape[i][0] << ", " << landscape[i][1] << ")";

        if (landscape[i][1] == landscape[i - 1][1]) {
            landingZoneRight = landscape[i][0];
            landingZoneLeft = landscape[i - 1][0];
            landingZoneCentre = (landingZoneLeft + landingZoneRight) / 2;
            landingZoneHeight = landscape[i][1];
            std::cerr << " <-- Landing Zone";
        }
        std::cerr << std::endl;
    }

    std::cin >> X >> Y >> HS >> VS >> F >> R >> P; std::cin.ignore();

    std::vector<std::vector<int>> obstacles;
    for (int i = 0; i < N; i++) {
        if (betweenPoints(landscape[i][0], X, landingZoneCentre)) {
            // Point is between start and target
            if (landscape[i][1] > landingZoneHeight) {
                obstacles.push_back(landscape[i]);
            }
        }
    }

    PIDController controller = PIDController(landingZoneCentre, landingZoneHeight, obstacles, 0.0027, 0.1, 0.002, 0.1);

    // game loop
    while (1) {
        std::cerr << X << ", " << Y << ", " << HS << ", " << VS << ", " << F << ", " << R << ", " << P << std::endl;

        inLandingZone = X > landingZoneLeft + LANDING_X_THRESHOLD && X < landingZoneRight - LANDING_X_THRESHOLD;
        nextDemand = controller.calculateDemand(X, Y, HS, VS, R, P, inLandingZone);
        std::cout << round(nextDemand.rotation) << " " << round(nextDemand.power) << std::endl;

        std::cin >> X >> Y >> HS >> VS >> F >> R >> P; std::cin.ignore();
    }
}
