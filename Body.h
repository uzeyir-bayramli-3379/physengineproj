#pragma once
#include "Vector2.h"

class Body {
public:
    Vector2 position;
    Vector2 velocity;
    float mass;
    bool isDragging = false;
    float angle = 0.0f;
    float angularVelocity = 0.0f;
    float inertia = 1.0f; // Resistance to rotation
    float invInertia = 1.0f; // 1.0f / inertia
    float invMass; // 0 for static, 1/mass for dynamic
    
    Body(float x, float y, float m);
    void updateRK4(float dt, Vector2 force);

private:
    // Define helper structs and function INSIDE the class
    struct State { Vector2 pos; Vector2 vel; };
    struct Derivative { Vector2 dp; Vector2 dv; };
    
    Derivative evaluate(const State& initial, float dt, const Derivative& d, Vector2 force);
};