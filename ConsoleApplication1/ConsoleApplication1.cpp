// ConsoleApplication1.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include "vectors.h"
#include "generator.h"
#include "path.h"
#include "pursuit.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <cstdarg>
#include <string>
#include <iomanip>

float round_up(float value, int decimal_places) {
    const float multiplier = std::pow(10.0, decimal_places);
    return std::ceil(value * multiplier) / multiplier;
}

void write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<float>>> dataset) {
    // Make a CSV file with one or more columns of integer values
    // Each column of data is represented by the pair <column name, column data>
    //   as std::pair<std::string, std::vector<int>>
    // The dataset is represented as a vector of these columns
    // Note that all columns should be the same size


    // Create an output filestream object
    std::ofstream myFile(filename, std::ofstream::out | std::ofstream::trunc);
    myFile.clear();

    // Send column names to the stream
    for (int j = 0; j < dataset.size(); ++j)
    {
        myFile << dataset.at(j).first;
        if (j != dataset.size() - 1) myFile << ","; // No comma at end of line
    }
    myFile << "\n";

    // Send data to the stream
    for (int i = 0; i < dataset.at(0).second.size(); ++i)
    {
        for (int j = 0; j < dataset.size(); ++j)
        {
            myFile << round_up(dataset.at(j).second.at(i), 5);
            if (j != dataset.size() - 1) myFile << ","; // No comma at end of line
        }
        myFile << "\n";
    }

    // Close the file
    myFile.close();
}


float convertToVelocity(float velocity) {
    return (60 * velocity) / (2 * M_PI * 2);
}

float convertToVoltage(float velocity) {
    float wheel_velocity = (60 * velocity) / (2 * M_PI * 2);
    return (wheel_velocity / 200) * 127;
}

vector3D odom(vector3D pos, float leftWheelVelocity, float rightWheelVelocity) {
    float deltaLeftDistance = 0.01 * 0.1047 * leftWheelVelocity;
    float deltaRightDistance = 0.01 * 0.1047 * rightWheelVelocity;

    float deltaAverageDistance = (deltaLeftDistance + deltaRightDistance) / 2;
    float deltaHeading = (deltaRightDistance - deltaLeftDistance) / 12;

    float deltaX = deltaAverageDistance * cos(pos.z + (deltaHeading / 2));
    float deltaY = deltaAverageDistance * sin(pos.z + (deltaHeading / 2));

    pos.x = pos.x + deltaX;
    pos.y = pos.y + deltaY;
    pos.z = pos.z + deltaHeading;
    return pos;
}

int main() {
    std::cout << "Hello";


    /*
    generator pathGenerator;

    pathGenerator.addPoint(vector3D(0, 0, 90));
    pathGenerator.addPoint(vector3D(48, 48, 90));

    pathGenerator.setVelocities(39, 3, 2);

    path robotPath = pathGenerator.generatePath();

    pursuitPath = robotPath.getRobotPath();

    std::vector<float> path_x_coordinate;
    std::vector<float> path_y_coordinate;

    for (int i = 0; i < pursuitPath.size(); i++) {
        path_x_coordinate.push_back(pursuitPath.at(i).point.x);
        path_y_coordinate.push_back(pursuitPath.at(i).point.y);
        //std::cout << x_coordinate.at(i) << " " << y_coordinate.at(i) << "\n";
    }

    //std::cout << "\n";

    int lastClosestPoint = 0;
    float lookaheadDistance = 6;
    bool isForward = true;
    bool onLastSegment = false;

    pursuit pursue;
    vector3D pos(0, 0, 90);
    pos.toRadians();

    //add stopping
    std::vector<float> posX, posY, posZ, lookAheadX, lookAheadY, leftWheelVelocity, rightWheelVelocity;

    for (int i = 0; i < 1500; i++) {

        vector2D currPose = pos.to2D();
        float heading = pos.z;

        int closestPointIndex = pursue.getClosestPointIndex(currPose);

       // std::cout << pursuitPath.at(closestPointIndex).point.x << " " << pursuitPath.at(closestPointIndex).point.y << "\n";

        vector2D lookaheadPoint = vector2D(0, 0);

        std::vector<hermite> points = pursuitPath;

       // lookaheadDistance = pursue.changeLkhdDist(pos.x, pos.y, lookaheadDistance);

        for (int i = closestPointIndex + 1; i < points.size(); i++) {

            vector2D startPoint = points.at(i - 1).point;
            vector2D endPoint = points.at(i).point;

            // std::cout << "Start Point: " << startPoint.x << " " << startPoint.y << "\n";
            // std::cout << "End Point: " << endPoint.x << " " << endPoint.y << "\n";

            if (i == points.size() - 1) {
                onLastSegment = true;
                // std::cout << "COOL";
            }



            vector2D lookaheadPtOptional = pursue.calculateLookAheadPoint(startPoint, endPoint, currPose, lookaheadDistance, onLastSegment);

            //  std::cout << lookaheadPtOptional.x << " " << lookaheadPtOptional.y;

            if (!lookaheadPtOptional.getEmpty()) {
                lookaheadPoint = lookaheadPtOptional;
                break;
            }
        }

        //std::cout << lookaheadPoint.x << " " << lookaheadPoint.y << "\n";

        float curvature = pursue.calculateCurvatureLookAheadArc(currPose, heading, lookaheadPoint, lookaheadDistance);

        //std::cout << curvature << "\n";

        float leftTargetVel = pursue.calculateLeftTargetVelocity(pursuitPath.at(pursue.getClosestPointIndex(currPose)).velocity, curvature);
        float rightTargetVel = pursue.calculateRightTargetVelocity(pursuitPath.at(pursue.getClosestPointIndex(currPose)).velocity, curvature);

        float leftWheelVel = convertToVelocity(leftTargetVel);
        float rightWheelVel = convertToVelocity(rightTargetVel);

       // std::cout << convertToVelocity(leftTargetVel) << " " << convertToVelocity(rightTargetVel) << "\n";


        posX.push_back(pos.x);
        posY.push_back(pos.y);
        posZ.push_back(pos.z * (180 / M_PI));
        lookAheadX.push_back(lookaheadPoint.x);
        lookAheadY.push_back(lookaheadPoint.y);
        leftWheelVelocity.push_back(leftWheelVel);
        rightWheelVelocity.push_back(rightWheelVel);

        pos = odom(pos, leftWheelVel, rightWheelVel);
    }

    std::vector<std::pair<std::string, std::vector<float>>> data = { {"PosX",posX }, {"PosY",posY},
       {"lookAheadX", lookAheadX}, {"lookAheadY", lookAheadY}, {"left", leftWheelVelocity}, {"right", rightWheelVelocity}};
    write_csv("cool.csv", data);

    

    */
}
    //create better exceptions. 

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
