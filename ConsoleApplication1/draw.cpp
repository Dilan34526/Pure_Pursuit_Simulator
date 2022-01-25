#include "draw.h"
#include <iostream>

float angleChange(float angle) {

    if (0 <= angle && angle <= 90) {
        angle = 90 - angle;
    }
    else if (90 < angle && angle <= 360) {
        angle = -angle + 450;
    }

    return angle;
}

float transformX(float x) {
    return (5 * x) + 360;
}

float transformY(float y) {
    return (-5 * y) + 360; 
}

Mat drawPath(Mat& img, std::vector<float> x, std::vector<float> y) {

    std::vector<std::vector<float>> path;


    for (int i = 0; i < x.size(); i++) {
        x.at(i) = 5 * (x.at(i)) + 360;
        y.at(i) = -5 * (y.at(i)) + 360;
        // std::cout << x.at(i) << " " << y.at(i) << "\n";
    }

    path = { x , y };

    for (int i = 0; i < path.at(0).size(); i++) {
        circle(img, Point(int(path.at(0).at(i)), int(path.at(1).at(i))), 2,
            Scalar(0, 0, 0), -1);
    }

    for (int i = 1; i < path.at(0).size(); i++) {
        line(img, Point(int(path.at(0).at(i - 1)), int(path.at(1).at(i - 1))),
            Point(int(path.at(0).at(i)), int(path.at(1).at(i))),
            Scalar(0, 0, 0), 1);
    }

    return img.clone();
}

void drawRobot(Mat img, vector3D pos, vector2D lookaheadPt, float leftWheelVel, float rightWheelVel) {


    Mat robot = img.clone();

    pos.toDegrees();

    pos.x = transformX(pos.x);
    pos.y = transformY(pos.y);
    pos.z = angleChange(pos.z); //this must be in degrees
    lookaheadPt.x = transformX(lookaheadPt.x);
    lookaheadPt.y = transformY(lookaheadPt.y);

    circle(robot, Point(int(pos.x), int(pos.y)), 2,
        Scalar(255, 255, 255), -1);
    RotatedRect rRect = RotatedRect(Point2f(pos.x, pos.y), Size2f(12*5, 12*5), pos.z);
    Point2f vertices[4];
    rRect.points(vertices);
    for (int i = 0; i < 4; i++) {
        line(robot, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 2);
    }
    
    vector2D pointLB = vector2D(vertices[0].x, vertices[0].y);
    vector2D pointLF = vector2D(vertices[1].x, vertices[1].y);
    vector2D pointRF = vector2D(vertices[2].x, vertices[2].y);
    vector2D pointRB = vector2D(vertices[3].x, vertices[3].y);

    leftWheelVel = (leftWheelVel / 200) * 75;
    rightWheelVel = (rightWheelVel / 200) * 75;

    //circle(robot, Point(int(pointLF.x), int(pointLF.y)), 2, Scalar(0, 0, 255), -1);

    vector2D segL = (pointLF.sub(pointLB)).mult(leftWheelVel / pointLB.dist(pointLF)).add(pointLF);
    vector2D segR = (pointRF.sub(pointRB)).mult(rightWheelVel / pointRB.dist(pointRF)).add(pointRF);

    line(robot, vertices[1], Point(int(segL.x), int(segL.y)), Scalar(0, 0, 255), 1);
    line(robot, vertices[2], Point(int(segR.x), int(segR.y)), Scalar(0, 0, 255), 1);
    
    //std::cout << segL << " " << segL << std::endl;




    circle(robot, Point(int(lookaheadPt.x), int(lookaheadPt.y)), 2, Scalar(0, 0, 255), -1);
    line(robot, Point(int(lookaheadPt.x), int(lookaheadPt.y)), Point(int(pos.x), int(pos.y)), Scalar(0, 0, 0), 1);
    imshow("Output", robot);
    waitKey(40); //sleep for 50 milliseconds
}

// circle(img, Point(150, 225), 5,
    //   Scalar(255, 255, 255), -1);
  // circle(img, Point(350, 225), 5,
    //   Scalar(255, 255, 255), -1);
   //imshow("rectangles", test_image);
   //waitKey(0);