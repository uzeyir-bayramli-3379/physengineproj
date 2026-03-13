#include <vector>
#include <cmath>
#include <GL/glew.h> // Or your OpenGL loader
#include <GLFW/glfw3.h>
#include "Vector2.h"
#include "Body.h"
#include "Shape.h"
#include <cfloat>
#include <iostream>
#include <algorithm>
#include <limits>
// For rendering (if needed)
struct Manifold {
    bool colliding;
    float depth;      // The amount of penetration
    Vector2 normal;   // The collision axis (must be unit length)
};
struct PhysicsObject {
    Body body;
    const Shape* shape;

    PhysicsObject(Body b, const Shape* s) : body(b), shape(s) {}
};
struct Projection { float min, max; };
std::vector<Vector2> getWorldNormals(const PhysicsObject& obj) {
    // Since there's no rotation, we just return the local normals!
    return obj.shape->localNormals; 
}
Projection projectShape(const PhysicsObject& obj, Vector2 axis) {
    float min = FLT_MAX, max = -FLT_MAX;
    for (const auto& v : obj.shape->localVertices) {
        // Position is just offset + vertex
        float worldX = obj.body.position.x + v.x;
        float worldY = obj.body.position.y + v.y;
        
        float dot = (worldX * axis.x) + (worldY * axis.y);
        if (dot < min) min = dot;
        if (dot > max) max = dot;
    }
    return { min, max };
}

Manifold CheckCollision(PhysicsObject& A, PhysicsObject& B) {
    Manifold m;
    m.colliding = false;
    m.depth = std::numeric_limits<float>::max();

    // Since we are not rotating, our axes are always the World X and Y axes
    // Axis 1: (1, 0), Axis 2: (0, 1)
    Vector2 axes[] = { {1, 0}, {0, 1} };

    for (const auto& axis : axes) {
        Projection pA = projectShape(A, axis);
        Projection pB = projectShape(B, axis);

        // SAT: If there is a gap, return immediately (no collision)
        if (pA.max <= pB.min || pB.max <= pA.min) {
            m.colliding = false;
            return m;
        }

        // Calculate overlap depth on this axis
        float overlap = std::min(pA.max, pB.max) - std::max(pA.min, pB.min);
        
        // Keep the smallest overlap (The MTV)
        if (overlap < m.depth) {
            m.depth = overlap;
            m.normal = axis;
        }
    }
    if (m.depth > 0) {
    std::cout << "Collision Depth: " << m.depth << " Normal: " << m.normal.y << std::endl;
}
    m.colliding = true;
    return m;
}

void ResolveConstraint(PhysicsObject& A, PhysicsObject& B, Manifold m) {
    Vector2 relativePos = A.body.position - B.body.position;
    if ((m.normal.x * relativePos.x + m.normal.y * relativePos.y) < 0) {
        m.normal.x *= -1.0f;
        m.normal.y *= -1.0f;
    }
    float totalInvMass = A.body.invMass + B.body.invMass;
    if (totalInvMass == 0.0f) return;
    
    // Push objects apart based on the overlap depth
    Vector2 correction = { m.normal.x * m.depth, m.normal.y * m.depth };
    
    A.body.position.x += correction.x * (A.body.invMass / totalInvMass);
    A.body.position.y += correction.y * (A.body.invMass / totalInvMass);
    
    B.body.position.x -= correction.x * (B.body.invMass / totalInvMass);
    B.body.position.y -= correction.y * (B.body.invMass / totalInvMass);
}
void render(const std::vector<PhysicsObject>& world) {
    for (const auto& obj : world) {
        // Set color based on whether it's a dynamic object (red) or static wall (blue)
        if (obj.body.invMass > 0.0f) {
            glColor3f(1.0f, 0.0f, 0.0f); // Red for boxes
        } else {
            glColor3f(0.0f, 0.0f, 1.0f); // Blue for static walls
        }

        glBegin(GL_POLYGON);
        for (const auto& v : obj.shape->localVertices) {
            // Because there is no rotation, this is just simple vector addition
            float wx = obj.body.position.x + v.x;
            float wy = obj.body.position.y + v.y;
            glVertex2f(wx, wy);
        }
        glEnd();
    }
}
int main() {
    // 1. Setup Window
    if (!glfwInit()) return -1;
    GLFWwindow* window = glfwCreateWindow(640, 480, "Physics Engine", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable V-Sync to cap framerate and fix speed

    // 2. Setup World
    std::vector<PhysicsObject> world;
    Vector2 gravity = {0.0f, -9.81f};
    
    Shape floorShape({ {-100, -1}, {100, -1}, {100, 1}, {-100, 1} });
    Shape box({{0,0}, {1,0}, {1,1}, {0,1}});
    
    world.emplace_back(Body(0.0f, -2.0f, 0.0f), &floorShape); // Static floor
    world.emplace_back(Body(0.0f, 10.0f, 1.0f), &box);       // Dynamic box

    // 3. Main Loop
    while (!glfwWindowShouldClose(window)) {
        float timeScale = 1.0f;

        // Inside the while loop, check for a key press
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
            timeScale = 0.1f; // Slow down when space is held
        } else {
            timeScale = 1.0f; // Return to normal
        }
        float frameTime = 0.016f*timeScale; // Target 60 FPS
        int subSteps = 4;
        float dt = frameTime / subSteps;

        // --- Physics Logic ---
        for (int s = 0; s < subSteps; s++) {
            for (auto& obj : world) {
                if (obj.body.invMass > 0.0f) {
                    obj.body.updateRK4(dt, gravity * obj.body.mass);
                }
            }

            for (size_t i = 0; i < world.size(); i++) {
                for (size_t j = i + 1; j < world.size(); j++) {
                    Manifold m = CheckCollision(world[i], world[j]);
                    if (m.colliding) {
                        ResolveConstraint(world[i], world[j], m);
                        // Zero out vertical velocity on contact
                        if (world[i].body.invMass > 0.0f) world[i].body.velocity.y = 0.0f;
                        if (world[j].body.invMass > 0.0f) world[j].body.velocity.y = 0.0f;
                    }
                }
            }
        }

        // --- Rendering ---
        glClear(GL_COLOR_BUFFER_BIT);
        
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-10, 10, -10, 10, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        render(world);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
