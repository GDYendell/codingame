#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

/**
 * Save the Planet.
 * Use less Fossil Fuel.
 **/
 
#define GRAVITY 3.711;
#define PI 3.14159265
 
typedef struct Demand {
    int power;
    int rotation;
} Demand;


Demand calculateDemand(int currentX, int currentY, int targetX, int targetY, float kP) {
    int powerX, powerY, demandRotation, demandPower, errorX, errorY, rotation, power;

    errorX = currentX - targetX;
    errorY = currentY - targetY;

    powerX = errorX * kP;
    powerY = GRAVITY + errorY * kP;
    
    float yOverX = powerY / powerX;

    demandRotation = atan(powerY / powerX) * 180 / PI;
    demandPower = sqrt(pow(powerX, 2) + pow(powerY, 2));

    if (demandPower > 4) {demandPower = 4;}

    std::cerr << "powerX: " << powerX << ", powerY: " << powerY << std::endl;
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
            landingZoneLeft = landX;
            landingZoneRight = landscapeX[i - 1];
            landingZoneCentre = abs(landingZoneLeft - landingZoneRight) / 2;
            landingZoneHeight = landY;
            std::cerr << " <-- Landing Zone";
        }
        std::cerr << std::endl;
    }

    // game loop
    while (1) {
        std::cin >> X >> Y >> HS >> VS >> F >> R >> P; std::cin.ignore();

        std::cerr << "Y: " << Y << std::endl;

        nextDemand = calculateDemand(X, Y, landingZoneCentre, landingZoneHeight, 0.005);

        std::cout << nextDemand.rotation << " " << nextDemand.power << std::endl;
    }
}
