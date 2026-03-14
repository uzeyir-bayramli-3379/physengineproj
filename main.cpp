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
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
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

    // Get all axes to test: normals from both shapes
    std::vector<Vector2> axes;
    auto normalsA = getWorldNormals(A);
    auto normalsB = getWorldNormals(B);
    axes.insert(axes.end(), normalsA.begin(), normalsA.end());
    axes.insert(axes.end(), normalsB.begin(), normalsB.end());

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
class ImpulseResolver {
public:
    // This is where all the math lives
    static void Resolve(PhysicsObject& A, PhysicsObject& B, Manifold& m) {
        // Ensure the normal always points from A toward B.
        // The SAT can return the normal in either direction; this corrects it
        // for any collision axis (floors, left walls, right walls, box-vs-box).
        Vector2 aToB = { B.body.position.x - A.body.position.x,
                         B.body.position.y - A.body.position.y };
        if ((m.normal.x * aToB.x + m.normal.y * aToB.y) < 0) {
            m.normal = { -m.normal.x, -m.normal.y };
        }
        // 1. Position Correction (to stop sinking)
        ApplyPositionCorrection(A, B, m);

        // 2. Impulse Resolution (Bounce and Friction)
        ApplyImpulse(A, B, m);
        if (m.depth<0.01f) {
            if (std::abs(B.body.velocity.y) < 0.001f) {
                B.body.velocity.y = 0.0f; // Stop vertical movement if very small
            }
            if (std::abs(B.body.velocity.x) < 0.001f) {
                B.body.velocity.x = 0.0f; // Stop horizontal movement if very small
            }
        }
    }

private:
    static void ApplyPositionCorrection(PhysicsObject& A, PhysicsObject& B, const Manifold& m) {
        float totalInvMass = A.body.invMass + B.body.invMass;
        if (totalInvMass == 0.0f) return;

        const float slop = 0.001f; 
        const float percent = 0.8f; // INCREASED to 0.8f for faster correction

        float correctionMag = (std::max(m.depth - slop, 0.0f) / totalInvMass) * percent;
        Vector2 correction = { m.normal.x * correctionMag, m.normal.y * correctionMag };

        A.body.position -= correction * A.body.invMass;
        B.body.position += correction * B.body.invMass;
        
    }

    static void ApplyImpulse(PhysicsObject& A, PhysicsObject& B, const Manifold& m) {
        Vector2 rv = B.body.velocity - A.body.velocity;
        float velAlongNormal = (rv.x * m.normal.x) + (rv.y * m.normal.y);
        
        if (velAlongNormal > 0) return;
        
        
        // 2. Calculate Restitution (Bounce)
        float e = std::min(A.body.restitution, B.body.restitution);
        //if (std::abs(velAlongNormal) < 0.5f) {
        //    e = 0.0f; // Treat as perfectly inelastic
        //}
        float j = -(1 + e) * velAlongNormal;
        j /= (A.body.invMass + B.body.invMass);

        // 3. Apply Normal Impulse
        Vector2 impulse = { j * m.normal.x, j * m.normal.y };
        A.body.velocity -= impulse * A.body.invMass;
        B.body.velocity += impulse * B.body.invMass;
        // --- Friction Logic ---
        // Recalculate relative velocity AFTER normal impulse
        rv = B.body.velocity - A.body.velocity;

        // Recompute rvAlongNormal from the NEW rv (not the stale pre-impulse one)
        float rvAlongNormal = (rv.x * m.normal.x) + (rv.y * m.normal.y);
        // Tangent = post-impulse rv minus its component along the normal
        Vector2 tangent = { rv.x - m.normal.x * rvAlongNormal,
                            rv.y - m.normal.y * rvAlongNormal };
        float tangentLen = std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y);

        if (tangentLen > 0.0001f) {
            tangent = { tangent.x / tangentLen, tangent.y / tangentLen }; // Normalize

            float jt = -(rv.x * tangent.x + rv.y * tangent.y);
            jt /= (A.body.invMass + B.body.invMass);

            // Coulomb's Law: limit friction based on normal impulse
            float mu = 0.0f;
            Vector2 frictionImpulse;
            if (std::abs(jt) < j * mu) {
                frictionImpulse = { tangent.x * jt, tangent.y * jt };   // static: use jt directly (sign opposes motion)
            } else {
                float kineticJ = std::copysign(j * mu, jt);             // kinetic: clamp magnitude, preserve direction
                frictionImpulse = { tangent.x * kineticJ, tangent.y * kineticJ };
            }

            // Same sign convention as normal impulse
            A.body.velocity -= frictionImpulse * A.body.invMass;
            B.body.velocity += frictionImpulse * B.body.invMass;
      }
    }
};
int main() {
    int idCounter = 0;
    // 1. Setup Window
    if (!glfwInit()) return -1;
    GLFWwindow* window = glfwCreateWindow(640, 480, "Physics Engine", NULL, NULL);
    glfwMakeContextCurrent(window);
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");
    // 2. Setup World
    std::vector<PhysicsObject> world;
    Vector2 gravity = {0.0f, -9.81f};
    // CCW rectangle: BL→BR→TR→TL  → inner left face normal = (−1, 0) ✓
    Shape wallShapeR({ {-1, -1}, {1, -1}, {1, 100}, {-1, 100} });
    // x-mirror of wallShapeR (negate x, same slot order → winding flips)
    // → inner right face normal = (+1, 0) ✓
    Shape wallShapeL({ {1, -1}, {-1, -1}, {-1, 100}, {1, 100} });
    Shape floorShape({ {-100, -1}, {100, -1}, {100, 1}, {-100, 1} });
    Shape boxShape({{0,0}, {1,0}, {1,1}, {0,1}});
    Body box(0.0f, 0.0f, 1.0f, 0.6f, 0.1f, idCounter++);
    box.velocity={10.0f, 0.0f};
    world.emplace_back(Body(0.0f, -2.0f, 0.0f,0.5f, 0.3f, -1), &floorShape); // Static floor
    world.emplace_back(Body(-10.0f, 0.0f, 0.0f,0.5f, 0.3f, -2), &wallShapeL); // Static left wall
    world.emplace_back(Body(5.0f, 0.0f, 0.0f,0.5f, 0.3f, -3), &wallShapeR); // Static right wall
    world.emplace_back(box, &boxShape);   // Dynamic box
    // ... inside main ...
double lastTime = glfwGetTime();
std::vector<PhysicsObject> spawnQueue; // Safety first!

while (!glfwWindowShouldClose(window)) {
    double currentTime = glfwGetTime();
    double deltaTime = currentTime - lastTime;
    lastTime = currentTime;

    // Accumulator for fixed time-stepping
    static double accumulator = 0.0;
    accumulator += deltaTime;

    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("Spawner");
    if (ImGui::Button("Add Box")) {
        // Spawn within the -10 to 10 view range
        Body newBox(0.0f, 0.0f, 1.0f, 0.6f, 0.3f,idCounter++);
        newBox.velocity = {10.0f, 0.0f}; // Initial
        spawnQueue.emplace_back(newBox, &boxShape);
    }
    ImGui::End();

    // 1. Process Spawning safely
    for(auto& newObj : spawnQueue) world.push_back(newObj);
    spawnQueue.clear();

    // 2. Fixed Physics Step (The "Slow-down" logic)
    const double fixedDt = 0.01667; 
    while (accumulator >= fixedDt) {
        for (auto& obj : world) {
            if (obj.body.invMass > 0.0f) 
                obj.body.updateEuler(fixedDt, gravity * obj.body.mass);
        }
        
        for (size_t i = 0; i < world.size(); i++) {
            for (size_t j = i + 1; j < world.size(); j++) {
                Manifold m = CheckCollision(world[i], world[j]);
                if (m.colliding) ImpulseResolver::Resolve(world[i], world[j], m);
            }
        }
        accumulator -= fixedDt;
    }

    // 3. Render
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-10, 10, -10, 10, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    render(world);

    ImDrawList* draw_list = ImGui::GetForegroundDrawList();
ImGuiIO& io = ImGui::GetIO();
float windowWidth = ImGui::GetIO().DisplaySize.x;
float windowHeight = ImGui::GetIO().DisplaySize.y;

// World bounds
float worldMin = -10.0f;
float worldMax = 10.0f;
float worldRange = worldMax - worldMin; // 20.0f
for (const auto& obj : world) {
    // 1. Convert world position to a 0.0 to 1.0 "percentage" of the screen
    float normX = (obj.body.position.x - worldMin) / worldRange;
    float normY = (worldMax - obj.body.position.y) / worldRange;

    // 2. Convert to pixel coordinates based on current window size
    float screenX = normX * 640.0f; // io.DisplaySize.x;
    float screenY = normY * 480.0f; // io.DisplaySize.y;

    // 3. Draw text
    std::string text = std::to_string(obj.body.id);
    draw_list->AddText(ImVec2(screenX, screenY), IM_COL32(0, 255, 0, 255), text.c_str());
}
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
}
}
