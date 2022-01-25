#pragma once

#include <math.h>
#define M_PI acos(-1)
/*
* Create a 2D vector structure in the form of (x, y)
*/
extern struct vector2D {

private:
    bool empty;

public:
    float x, y;

    
    vector2D() {
        x = 0;
        y = 0;
        empty = true;
    }

    vector2D(float x1, float y1) {
        x = x1;
        y = y1;
        empty = false;
    }

    void set(float x1, float y1) {
        x = x1;
        y = y1;
        empty = false;
    }

    float norm() {
        return sqrt(pow(x, 2) + pow(y, 2));
    }

    vector2D add(vector2D a, vector2D b, vector2D target) {
        if (target.empty) {
            target = vector2D(a.x + b.x, a.y + b.y);
        }
        else {
            target.set(a.x + b.x, a.y + b.y);
        }
        return target;
    }

    vector2D add(vector2D a, vector2D b) {
        return add(a, b, vector2D());
    }

    vector2D sub(vector2D a, vector2D b, vector2D target) {
        if (target.empty) {
            target = vector2D(a.x - b.x, a.y - b.y);
        }
        else {
            target.set(a.x - b.x, a.y - b.y);
        }
        return target;
    }

    vector2D sub(vector2D a, vector2D b) {
        return sub(a, b, vector2D());
    }

    vector2D mult(vector2D a, float n, vector2D target) {
        if (target.empty) {
            target = vector2D(a.x * n, a.y * n);
        }
        else {
            target.set(a.x * n, a.y * n);
        }
        return target;
    }

    vector2D mult(vector2D a, float n) {
        return mult(a, n, vector2D());
    }

    vector2D mult(vector2D a, vector2D b, vector2D target) {
        if (target.empty) {
            target = vector2D(a.x * b.x, a.y * b.y);
        }
        else {
            target.set(a.x * b.x, a.y * b.y);
        }
        return target;
    }

    vector2D mult(vector2D a, vector2D b) {
        return mult(a, b, vector2D());
    }

    vector2D div(vector2D a, float n, vector2D target) {
        if (target.empty) {
            target = vector2D(a.x / n, a.y / n);
        }
        else {
            target.set(a.x / n, a.y / n);
        }
        return target;
    }

    vector2D div(vector2D a, float n) {
        return div(a, n, vector2D());
    }

    vector2D div(vector2D a, vector2D b, vector2D target) {
        if (target.empty) {
            target = vector2D(a.x / b.x, a.y / b.y);
        }
        else {
            target.set(a.x / b.x, a.y / b.y);
        }
        return target;
    }

    vector2D div(vector2D a, vector2D b) {
        return div(a, b, vector2D());
    }

    float dist(vector2D a, vector2D b) {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        return sqrt(pow(dx, 2) + pow(dy, 2));
    }

    float dot(vector2D a, vector2D b) {
        return a.x * b.x + a.y * b.y;
    }

    float angleBetween(vector2D a, vector2D b) {
        float dot1 = dot(a, b);
        float amag = sqrt(pow(a.x, 2) + pow(a.y, 2));
        float bmag = sqrt(pow(b.x, 2) + pow(b.y, 2));
        return acos(dot1 / (amag * bmag));
    }

    void add(float x1, float y1) {
        x += x1;
        y += y1;
    }

    vector2D add(vector2D v) {
        x += v.x;
        y += v.y;
        return vector2D(x, y);
    }

    void sub(float x1, float y1) {
        x -= x1;
        y -= y1;
    }

    vector2D sub(vector2D a) {
        x = x - a.x;
        y = y - a.y;
        return vector2D(x, y);
    }

    vector2D mult(float n) {
        x *= n;
        y *= n;
        return vector2D(x, y);
        return vector2D(x, y);
    }

    vector2D mult(vector2D a) {
        x *= a.x;
        y *= a.y;
        return vector2D(x, y);
    }

    void div(float n) {
        x /= n;
        y /= n;
    }

    vector2D div(vector2D a) {
        x /= a.x;
        y /= a.y;
        return vector2D(x, y);
    }

    float dist(vector2D v) {
        float dx = x - v.x;
        float dy = y - v.y;
        return sqrt(pow(dx, 2) + pow(dy, 2));
    }

    float dot(vector2D v) {
        return x * v.x + y * v.y;
    }

    float dot(float x1, float y1) {
        return x * x1 + y * y1;
    }

    void normalize() {
        float m = norm();
        if (m != 0 && m != 1) {
            div(m);
        }
    }

    vector2D normalize(vector2D target) {
        if (target.empty) {
            target = vector2D();
        }
        float m = norm();
        if (m > 0) {
            target.set(x / m, y / m);
        }
        else {
            target.set(x, y);
        }
        return target;
    }

    void limit(float max) {
        if (norm() > max) {
            normalize();
            mult(max);
        }
    }

    bool getEmpty() {
        return empty;
    }

};

/*
* Creates a 3D vector structure in the form of (x, y, z/theta)
*/

struct vector3D {

    float x;
    float y;
    float z;

    vector3D() {
        x = 0;
        y = 0;
        z = 0;
    }

    vector3D(float x1, float y1, float z1) {

        x = x1;
        y = y1;
        z = z1;

    }

    vector2D to2D() {
        return vector2D(x, y);
    }

    float dist(vector3D a) {
        float dx = a.x - x;
        float dy = a.y - y;
        return sqrt(pow(dx, 2) + pow(dy, 2));
    }

    void toRadians() {

        z = z * M_PI / 180;
    }

    void toDegrees() {
        z = z * 180 / M_PI;
    }

};


extern struct hermite {

    vector2D point;
    float velocity;
    float curvature;
    float distance;

    hermite() {
        point = vector2D();
        velocity = 0;
        curvature = 0;
        distance = 0;
    }

    hermite(vector2D vec) {
        point = vec;
        velocity = 0;
        curvature = 0;
        distance = 0;
    }

    void setCurvature(float c1) {
        curvature = c1;
    }

    void setVelocity(float v1) {
        velocity = v1;
    }

    void setDistance(float d1) {
        distance = d1;
    }

    float getVelocity() {
        return velocity;
    }
};