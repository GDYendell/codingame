#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

/**
 * Save the Planet.
 * Use less Fossil Fuel.
 **/
 
 

int main()
{
    int X;
    int Y;
    int HS; // the horizontal speed (in m/s), can be negative.
    int VS; // the vertical speed (in m/s), can be negative.
    int F; // the quantity of remaining fuel in liters.
    int R; // the rotation angle in degrees (-90 to 90).
    int P; // the thrust power (0 to 4).

    int direction;

    int N; // the number of points used to draw the surface of Mars.
    std::cin >> N; std::cin.ignore();
    
    std::vector<int> landscapeX(N), landscapeY(N);
    int landingZoneLeft, landingZoneRight, landingZoneHeight;
    for (int i = 0; i < N; i++) {
        int landX; // X coordinate of a surface point. (0 to 6999)
        int landY; // Y coordinate of a surface point. By linking all the points together in a sequential fashion, you form the surface of Mars.
        std::cin >> landX >> landY; std::cin.ignore();
        landscapeX[i] = landX;
        landscapeY[i] = landY;
        
        std::cerr << "(" << landX << ", " << landY << ")";
        
        if (landY == landscapeY[i - 1]) {
            landingZoneLeft = landX;
            landingZoneRight = landscapeX[i - 1];
            landingZoneHeight = landY;
            std::cerr << " <-- Landing Zone";
        }
        std::cerr << std::endl;
    }

    // game loop
    while (1) {
        std::cin >> X >> Y >> HS >> VS >> F >> R >> P; std::cin.ignore();

        std::cerr << "Y: " << Y << std::endl;
        
        direction = HS/abs(HS);

        if (Y < landingZoneHeight + 300) {
            R = 0;
            P = 4;
        } else if (500 < Y && Y < 1750) {
            R = direction * 20;
            P = 4;
        } else {
            R = - direction * 20;
            P = 3;
        }

        std::cout << R << " " << P << std::endl;
    }
}
