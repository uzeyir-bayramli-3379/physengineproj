#pragma once
#include "Vector2.h"

class Body {
public:
    // Physical State
    Vector2 position;
    Vector2 velocity;
    
    // Physical Properties
    float mass;
    float invMass; // 0.0f for static (immovable), 1.0f/mass for dynamic
    float restitution; // 0 = no bounce, 1 = perfect bounce
    float friction;
    int id;

    Body(float x, float y, float m, float res, float fric, int bodyId);

    // RK4 Integration step (Translation only)
    //void updateRK4(float dt, Vector2 force);
    void updateEuler(float dt, Vector2 force);

private:
    // Helper structures for RK4
    struct State { Vector2 pos, vel; };
    struct Derivative { Vector2 dp, dv; };

    Derivative evaluate(const State& initial, float dt, const Derivative& d, Vector2 force);
};