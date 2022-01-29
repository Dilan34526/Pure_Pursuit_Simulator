// ConsoleApplication1.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <iostream>
#include <vector>
#include "vectors.h"
#include "pid.h"
#include "generator.h"
#include "path.h"
#include "pursuit.h"
#include "odom.h"
#include "util.h"


#include <opencv2/core/core.hpp>

// Drawing shapes
#include <opencv2/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>
#include "draw.h"

using namespace std;
using namespace cv;

int main() {
  
    generator pathGenerator;

    pathGenerator.addPoint(vector3D(20, -10, 90));
    pathGenerator.addPoint(vector3D(-40, 48, 180));
    pathGenerator.addPoint(vector3D(-4, -30, 270));
    pathGenerator.addPoint(vector3D(30, -60, 0));
    //pathGenerator.addPoint(vector3D(20, -1, 90));

    pathGenerator.setVelocities(39, 3, 2);

    path robotPath = pathGenerator.generatePath();

    pursuitPath = robotPath.getRobotPath();

    std::vector<float> path_x_coordinate;
    std::vector<float> path_y_coordinate;

    for (int i = 0; i < pursuitPath.size(); i++) {
        path_x_coordinate.push_back(pursuitPath.at(i).point.x);
        path_y_coordinate.push_back(pursuitPath.at(i).point.y);
    }

    // Reading the Image
    Mat image = imread("C:/Users/dilan/source/repos/ConsoleApplication1/ConsoleApplication1/VRC_Field_update_720x720.png",
        IMREAD_COLOR);

    // Check if the image is created
    // successfully or not
    if (!image.data) {
        std::cout << "Could not open or "
            << "find the image\n";
        return 0;
    }

    Mat new_path_img = drawPath(image, path_x_coordinate, path_y_coordinate);

    int lastClosestPoint = 0;
    float lookaheadDistance = 10; //change lookahead Distance to show
    float changeLkhd = lookaheadDistance;
    bool isForward = true;
    bool onLastSegment = false;

    pursuit pursue;
    vector3D pos = pathGenerator.getPoint(0);
    float finalHeading = pathGenerator.getLastPoint().z;
    pos.toRadians();

    PID drive;
    PID orientation;
    pidInit(orientation, 0.25, 0, 0, 0, 0);
    pidInit(drive, -1, 0, 0, 0, 0);

    std::vector<float> posX, posY, posZ, lookAheadX, lookAheadY, leftWheelVelocity, rightWheelVelocity, lkhdDist, onLast, index, curv;

    int count = 0;
    bool run = true;

    vector2D lookaheadPoint = vector2D(0, 0);
    float leftWheelVel, rightWheelVel;

    string s;
    cin >> s;



   
    while(run) {


        vector2D currPose = pos.to2D();
        float heading = pos.z;

        int closestPointIndex = pursue.getClosestPointIndex(currPose);

        

        std::vector<hermite> points = pursuitPath;
      

        for (int i = closestPointIndex + 1; i < points.size(); i++) {

            vector2D startPoint = points.at(i - 1).point;
            vector2D endPoint = points.at(i).point;

            // std::cout << "Start Point: " << startPoint.x << " " << startPoint.y << "\n";
            // std::cout << "End Point: " << endPoint.x << " " << endPoint.y << "\n";

           // lookaheadDistance = pursue.changeLkhdDist(pos.x, pos.y, lookaheadDistance);

            if (i == points.size() - 1) {
                onLastSegment = true;
            }


            vector2D lookaheadPtOptional = pursue.calculateLookAheadPoint(startPoint, endPoint, currPose, lookaheadDistance, onLastSegment);

            //  std::cout << lookaheadPtOptional.x << " " << lookaheadPtOptional.y;


            if (!lookaheadPtOptional.getEmpty()) {
                lookaheadPoint = lookaheadPtOptional;
                break;
            }
        }

        //add rate limiter ?
        //need to come up with better solution

        //what we need to do: set lookahead point to a very small value

        /*
        if (closestPointIndex == points.size() - 1) {

            vector2D startPoint = points.at(points.size() - 2).point;
            vector2D endPoint = points.at(points.size() - 1).point;

            lookaheadDistance = 1;
            onLastSegment = false;
            
            vector2D lookaheadPtOptional = pursue.calculateLookAheadPoint(startPoint, endPoint, currPose, lookaheadDistance, onLastSegment);
            if (!lookaheadPtOptional.getEmpty()) {
                lookaheadPoint = lookaheadPtOptional;
            }
        }8?
        */
       // std::cout << lookaheadPoint.x << " " << lookaheadPoint.y << "\n";

        float curvature = pursue.calculateCurvatureLookAheadArc(currPose, heading, lookaheadPoint, lookaheadDistance);

        //std::cout << curvature << "\n";

        float leftTargetVel = pursue.calculateLeftTargetVelocity(pursuitPath.at(pursue.getClosestPointIndex(currPose)).velocity, curvature);
        float rightTargetVel = pursue.calculateRightTargetVelocity(pursuitPath.at(pursue.getClosestPointIndex(currPose)).velocity, curvature);

        leftWheelVel = convertToVelocity(leftTargetVel);
        rightWheelVel = convertToVelocity(rightTargetVel);

        if (closestPointIndex == pursuitPath.size() - 1) {

          
            vector2D lastPoint = pursuitPath.at(pursuitPath.size() - 1).point;

            float distance = currPose.dist(lastPoint);

            float vel = pidCalculate(drive, 0, distance);
            

            lookaheadPoint = pursuitPath.at(pursuitPath.size() - 1).point;
            lookaheadDistance = distance;

            curvature = pursue.calculateCurvatureLookAheadArc(currPose, heading, lookaheadPoint, lookaheadDistance);

            leftTargetVel = pursue.calculateLeftTargetVelocity(vel, curvature);
            rightTargetVel = pursue.calculateRightTargetVelocity(vel, curvature);

            leftWheelVel = convertToVelocity(leftTargetVel);
            rightWheelVel = convertToVelocity(rightTargetVel);
         
            
            if (distance < 0.2) {
                run = false;
                float headingCorrection = pidCalculate(orientation, finalHeading, heading * (180 / M_PI));
                leftWheelVel = -headingCorrection;
                rightWheelVel = headingCorrection;
                if (heading * (180 / M_PI) > finalHeading - 1 && heading * (180 / M_PI) < finalHeading + 1) {
                    leftWheelVel = 0;
                    rightWheelVel = 0;
                }
            }

            //std::cout << heading << endl;

            //std::cout << headingCorrection << endl;

           // std::cout << vel << " " << convertToVelocity(leftTargetVel) << " " << convertToVelocity(rightTargetVel) << endl;

        }

        // std::cout << convertToVelocity(leftTargetVel) << " " << convertToVelocity(rightTargetVel) << "\n";

        drawRobot(new_path_img, pos, lookaheadPoint, leftWheelVel, rightWheelVel);


     //   std::cout << lookaheadDistance << " " << round_up(lookaheadPoint.x, 2) << " " << round_up(lookaheadPoint.y, 2) << "\n";

        
        posX.push_back(pos.x);
        posY.push_back(pos.y);
        posZ.push_back(pos.z * (180 / M_PI));
        lookAheadX.push_back(lookaheadPoint.x);
        lookAheadY.push_back(lookaheadPoint.y);
        leftWheelVelocity.push_back(leftWheelVel);
        rightWheelVelocity.push_back(rightWheelVel);
        lkhdDist.push_back(lookaheadDistance);
        onLast.push_back(onLastSegment);
        index.push_back(closestPointIndex);
        curv.push_back(pursuitPath.at(pursue.getClosestPointIndex(currPose)).velocity);
       
        pos = odom(pos, leftWheelVel, rightWheelVel);
        

       // count++;
        //std::cout << count << endl;
        
    }

    //the vel needs fixing
     
     drawRobot(new_path_img, pos, lookaheadPoint, leftWheelVel, rightWheelVel);

   std::vector<std::pair<std::string, std::vector<float>>> data = {{"PosX",posX}, {"PosY",posY}, {"LKHD DIST", lkhdDist},
    {"lookAheadX", lookAheadX}, {"lookAheadY", lookAheadY}, {"left", leftWheelVelocity}, {"right", rightWheelVelocity}, {"ONLAST", onLast}, {"INDEX", index}, {"VEL", curv} };
    write_csv("newD.csv", data);






    //every time you iterate through pure pursuit, create a tmp image to draw on and set that as the image to create 






    /*
    // Top Left Coordinates
    Point p1(30, 70);

    // Bottom Right Coordinates
    Point p2(115, 155);

    
    int thickness = 2;

    // Drawing the Rectangle
    rectangle(image, p1, p2,
        Scalar(255, 0, 0),
        thickness, LINE_8);

    circle(image, Point(720, 720), 5,
        Scalar(0, 0, 0),
        FILLED,
        LINE_8);*/

    
    // Show our image inside a window
    //imshow("Output", new_path_img);
    //waitKey(0);

    


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
