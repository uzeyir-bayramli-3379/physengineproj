#include "Body.h"

// THIS IS THE LINE THE LINKER CAN'T FIND
Body::Body(float x, float y, float m) : position{x, y}, velocity{0, 0}, mass(m) {
        invMass = (m > 0.0f) ? (1.0f / m) : 0.0f;
}

Body::Derivative Body::evaluate(const State& initial, float dt, const Derivative& d, Vector2 force) {
    State state;
    state.pos = initial.pos + d.dp * dt;
    state.vel = initial.vel + d.dv * dt;

    Derivative output;
    output.dp = state.vel;
    if (mass > 0.0f) {
        output.dv = { force.x / mass, force.y / mass };
    } else {
        output.dv = { 0.0f, 0.0f }; // Static objects have 0 acceleration
    }
    return output;
}

void Body::updateRK4(float dt, Vector2 force) {
    if (isDragging) return;

    State initial = {position, velocity};

    Derivative a = evaluate(initial, 0.0f, {0,0}, force);
    Derivative b = evaluate(initial, dt*0.5f, a, force);
    Derivative c = evaluate(initial, dt*0.5f, b, force);
    Derivative d = evaluate(initial, dt, c, force);

    Vector2 dpos = (a.dp + (b.dp + c.dp) * 2.0f + d.dp) * (1.0f/6.0f);
    Vector2 dvel = (a.dv + (b.dv + c.dv) * 2.0f + d.dv) * (1.0f/6.0f);

    position += dpos * dt;
    velocity += dvel * dt;
}