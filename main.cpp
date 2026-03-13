#include <GLFW/glfw3.h>
#include "Body.h"
#include <cmath>
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include <vector>
#include <cstdio>
#include <algorithm>


struct Shape {
    std::vector<Vector2> localVertices; // Points relative to (0,0)
    std::vector<Vector2> localNormals;  // Pre-calculated normals
};

struct PhysicsObject {
    Body body;
    const Shape* shape; // Pointer to the blueprint
    // The "Compass" vectors
    Vector2 localRight = {1.0f, 0.0f};
    Vector2 localUp    = {0.0f, 1.0f};

    void updateRotation(float dt) {
        // 1. Integrate rotation
        body.angle += body.angularVelocity * dt;

        // 2. Update basis vectors
        float s = sin(body.angle);
        float c = cos(body.angle);
        
        localRight = {c, s};
        localUp    = {-s, c};
    }
};
std::vector<Vector2> getRotatedNormals(const PhysicsObject& obj) {
    std::vector<Vector2> worldNormals;
    for (const auto& localN : obj.shape->localNormals) {
        // Dot product projection to transform the vector
        float x = (localN.x * obj.localRight.x) + (localN.y * obj.localUp.x);
        float y = (localN.x * obj.localRight.y) + (localN.y * obj.localUp.y);
        worldNormals.push_back({ x, y });
    }
    return worldNormals;
}
struct Projection { float min; float max; };

Projection projectShape(const PhysicsObject& obj, Vector2 axis) {
    float min = FLT_MAX;
    float max = -FLT_MAX;

    for (const auto& localV : obj.shape->localVertices) {
        // 1. Transform vertex to world space
        float worldX = obj.body.position.x + (localV.x * obj.localRight.x) + (localV.y * obj.localUp.x);
        float worldY = obj.body.position.y + (localV.x * obj.localRight.y) + (localV.y * obj.localUp.y);
        
        // 2. Project onto axis using Dot Product
        float projection = (worldX * axis.x) + (worldY * axis.y);
        
        // 3. Track min/max
        if (projection < min) min = projection;
        if (projection > max) max = projection;
    }
    return { min, max };
}
Shape createShape(std::vector<Vector2> vertices) {
    Shape s;
    s.localVertices = vertices;
    int count = vertices.size();
    for (int i = 0; i < count; i++) {
        Vector2 p1 = vertices[i];
        Vector2 p2 = vertices[(i + 1) % count];
        Vector2 edge = { p2.x - p1.x, p2.y - p1.y };
        Vector2 normal = { -edge.y, edge.x }; // Perpendicular
        float len = sqrt(normal.x * normal.x + normal.y * normal.y);
        s.localNormals.push_back({ normal.x / len, normal.y / len });
    }
    return s;
}
void drawPolygon(PhysicsObject& obj) {
    glDisable(GL_CULL_FACE);
    glBegin(GL_POLYGON);
    for (auto& localV : obj.shape->localVertices) {
        // Convert to World space inside the draw call
        float wx = obj.body.position.x + (localV.x * obj.localRight.x) + (localV.y * obj.localUp.x);
        float wy = obj.body.position.y + (localV.x * obj.localRight.y) + (localV.y * obj.localUp.y);
        glVertex2f(wx, wy);
    }
    glEnd();
}

struct Manifold {
    bool colliding;
    float depth;      // The smallest overlap found
    Vector2 normal;    // The axis along which the overlap occurred
    Vector2 contactPoint; // The contact point for torque
};

Manifold CheckCollisionAndFindManifold(PhysicsObject& A, PhysicsObject& B) {
    Manifold m;
    m.colliding = false;
    m.depth = FLT_MAX;
    
    // Transform vertices to world space once
    std::vector<Vector2> worldVertsA;
    for (const auto& localV : A.shape->localVertices) {
        float wx = A.body.position.x + (localV.x * A.localRight.x) + (localV.y * A.localUp.x);
        float wy = A.body.position.y + (localV.x * A.localRight.y) + (localV.y * A.localUp.y);
        worldVertsA.push_back({wx, wy});
    }
    std::vector<Vector2> worldVertsB;
    for (const auto& localV : B.shape->localVertices) {
        float wx = B.body.position.x + (localV.x * B.localRight.x) + (localV.y * B.localUp.x);
        float wy = B.body.position.y + (localV.x * B.localRight.y) + (localV.y * B.localUp.y);
        worldVertsB.push_back({wx, wy});
    }

    // Transform normals to world space
    std::vector<Vector2> worldNormalsA;
    for (const auto& localN : A.shape->localNormals) {
        float nx = localN.x * A.localRight.x + localN.y * A.localUp.x;
        float ny = localN.x * A.localRight.y + localN.y * A.localUp.y;
        worldNormalsA.push_back({nx, ny});
    }
    std::vector<Vector2> worldNormalsB;
    for (const auto& localN : B.shape->localNormals) {
        float nx = localN.x * B.localRight.x + localN.y * B.localUp.x;
        float ny = localN.x * B.localRight.y + localN.y * B.localUp.y;
        worldNormalsB.push_back({nx, ny});
    }

    std::vector<Vector2> axes = worldNormalsA;
    axes.insert(axes.end(), worldNormalsB.begin(), worldNormalsB.end());

    // 2. The SAT Loop
    for (auto& axis : axes) {
        // Project world vertices
        float minA = FLT_MAX, maxA = -FLT_MAX;
        for (auto& v : worldVertsA) {
            float p = v.x * axis.x + v.y * axis.y;
            if (p < minA) minA = p;
            if (p > maxA) maxA = p;
        }
        float minB = FLT_MAX, maxB = -FLT_MAX;
        for (auto& v : worldVertsB) {
            float p = v.x * axis.x + v.y * axis.y;
            if (p < minB) minB = p;
            if (p > maxB) maxB = p;
        }

        // Check for a gap
        if (maxA < minB || maxB < minA) {
            return { false, 0.0f, {0,0}, {0,0} }; // Found a gap, no collision!
        }

        // 3. Find the overlap depth on this axis
        float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        
        // Is this the smallest overlap so far? (The smallest overlap is the Collision Normal)
        if (overlap < m.depth) {
            m.depth = overlap;
            m.normal = axis;
        }
    }

    // Fix normal direction: should point from A to B
    Vector2 fromAtoB = {B.body.position.x - A.body.position.x, B.body.position.y - A.body.position.y};
    float dot = m.normal.x * fromAtoB.x + m.normal.y * fromAtoB.y;
    if (dot > 0) {
        m.normal.x = -m.normal.x;
        m.normal.y = -m.normal.y;
    }

    // Calculate contact point: average of vertices with min projection for A and max for B
    float minProjA = FLT_MAX;
    std::vector<Vector2> contactVertsA;
    for (auto& v : worldVertsA) {
        float p = v.x * m.normal.x + v.y * m.normal.y;
        if (p < minProjA) {
            minProjA = p;
            contactVertsA.clear();
            contactVertsA.push_back(v);
        } else if (std::abs(p - minProjA) < 1e-6f) {
            contactVertsA.push_back(v);
        }
    }
    float maxProjB = -FLT_MAX;
    std::vector<Vector2> contactVertsB;
    for (auto& v : worldVertsB) {
        float p = v.x * m.normal.x + v.y * m.normal.y;
        if (p > maxProjB) {
            maxProjB = p;
            contactVertsB.clear();
            contactVertsB.push_back(v);
        } else if (std::abs(p - maxProjB) < 1e-6f) {
            contactVertsB.push_back(v);
        }
    }
    // Average all contact verts
    Vector2 sum = {0,0};
    int count = 0;
    for (auto& v : contactVertsA) { sum.x += v.x; sum.y += v.y; count++; }
    for (auto& v : contactVertsB) { sum.x += v.x; sum.y += v.y; count++; }
    if (count > 0) {
        m.contactPoint = {sum.x / count, sum.y / count};
    } else {
        m.contactPoint = {(A.body.position.x + B.body.position.x)/2, (A.body.position.y + B.body.position.y)/2};
    }

    m.colliding = true;
    return m;
}
void ResolveCollision(PhysicsObject& A, PhysicsObject& B, Manifold m) {
    const float percent = 0.8f; // How much of the overlap to fix (20% to 80%)
    const float slop = 0.01f;    // Allowed overlap
    
    float totalInvMass = A.body.invMass + B.body.invMass;
    if (totalInvMass == 0.0f) return;

    // Calculate r vectors
    Vector2 rA = { m.contactPoint.x - A.body.position.x, m.contactPoint.y - A.body.position.y };
    Vector2 rB = { m.contactPoint.x - B.body.position.x, m.contactPoint.y - B.body.position.y };

    // Velocity resolution (impulse) first
    Vector2 relativeVelocity = { A.body.velocity.x - B.body.velocity.x, A.body.velocity.y - B.body.velocity.y };
    float velocityAlongNormal = relativeVelocity.x * m.normal.x + relativeVelocity.y * m.normal.y;
    
    // if (velocityAlongNormal > 0) return; // Already separating - remove this for inelastic
    
    float restitution = 0.1f; // Slight bounce

    // Calculate denominator for impulse
    float crossA = rA.x * m.normal.y - rA.y * m.normal.x;
    float crossB = rB.x * m.normal.y - rB.y * m.normal.x;
    float denom = totalInvMass + A.body.invInertia * crossA * crossA + B.body.invInertia * crossB * crossB;
    
    float impulse = -(1 + restitution) * velocityAlongNormal / denom;
    
    Vector2 impulseVector = { m.normal.x * impulse, m.normal.y * impulse };
    
    A.body.velocity.x += impulseVector.x * A.body.invMass;
    A.body.velocity.y += impulseVector.y * A.body.invMass;
    
    B.body.velocity.x -= impulseVector.x * B.body.invMass;
    B.body.velocity.y -= impulseVector.y * B.body.invMass;

    // Angular impulse
    A.body.angularVelocity += crossA * impulse * A.body.invInertia;
    B.body.angularVelocity -= crossB * impulse * B.body.invInertia;

    // Friction
    // Recalculate relative velocity after normal impulse
    relativeVelocity = { A.body.velocity.x - B.body.velocity.x, A.body.velocity.y - B.body.velocity.y };
    Vector2 tangent = { -m.normal.y, m.normal.x }; // perpendicular to normal
    float velocityAlongTangent = relativeVelocity.x * tangent.x + relativeVelocity.y * tangent.y;
    
    float friction = 0.1f; // friction coefficient
    float crossTangentA = rA.x * tangent.y - rA.y * tangent.x;
    float crossTangentB = rB.x * tangent.y - rB.y * tangent.x;
    float denomTangent = totalInvMass + A.body.invInertia * crossTangentA * crossTangentA + B.body.invInertia * crossTangentB * crossTangentB;
    
    float tangentImpulse = -velocityAlongTangent / denomTangent;
    tangentImpulse = std::max(-friction * std::abs(impulse), std::min(tangentImpulse, friction * std::abs(impulse))); // clamp
    
    Vector2 tangentImpulseVector = { tangent.x * tangentImpulse, tangent.y * tangentImpulse };
    
    A.body.velocity.x += tangentImpulseVector.x * A.body.invMass;
    A.body.velocity.y += tangentImpulseVector.y * A.body.invMass;
    
    B.body.velocity.x -= tangentImpulseVector.x * B.body.invMass;
    B.body.velocity.y -= tangentImpulseVector.y * B.body.invMass;
    
    A.body.angularVelocity += crossTangentA * tangentImpulse * A.body.invInertia;
    B.body.angularVelocity -= crossTangentB * tangentImpulse * B.body.invInertia;

    // Position correction after impulse
    float magnitude = (std::max(m.depth - 0.0f, 0.0f) / totalInvMass) * 1.0f;
    
    A.body.position.x += m.normal.x * magnitude * A.body.invMass;
    A.body.position.y += m.normal.y * magnitude * A.body.invMass;
    
    B.body.position.x -= m.normal.x * magnitude * B.body.invMass;
    B.body.position.y -= m.normal.y * magnitude * B.body.invMass;

    // Stop x velocity when colliding with vertical walls
    if (B.body.mass == 0.0f && std::abs(m.normal.x) > std::abs(m.normal.y)) {
        A.body.velocity.x = 0.0f;
        A.body.angularVelocity = 0.0f; // Stop spinning on wall collision
    }
    // Stop spinning on floor collision
    if (B.body.mass == 0.0f && std::abs(m.normal.y) > std::abs(m.normal.x)) {
        A.body.angularVelocity = 0.0f;
    }
}
int sign(float val) {
    if (val > 0.0f) return 1;
    if (val < 0.0f) return -1;
    return 0;
}
int main() {
    // Initialize GLFW
    if (!glfwInit()) return -1;
    GLFWwindow* window = glfwCreateWindow(640, 480, "Physics Engine", NULL, NULL);
    glfwMakeContextCurrent(window);
    
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    // This ensures the default font is built
    io.Fonts->AddFontDefault();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");
    //Body ball(0.0f, 0.5f, 1.0f);
    //
    //float radius = 0.05f;
    //float pullForceY = 0.0f;
    //float gravity = -0.981f;
    //float friction = 0.0f;
    float netFX = 0.0f;
    float netFY = -0.0f;
    float floorY = 0.9f;
    float wallX = 0.9f; 
    //// coefficients of friction
    //float mu_k = 0.45f;      // kinetic friction coefficient
    //float mu_s = 0.5f;       // static (still) friction coefficient

    std::vector<PhysicsObject> world; 
    float lastFrame = 0.0f;
    // 1. Define shapes centered at (0,0) with half-extents (e.g., width 1.0, height 1.0)
    Shape boxShape = createShape({ {-0.5f, -0.5f}, {0.5f, -0.5f}, {0.5f, 0.5f}, {-0.5f, 0.5f} });
    Shape wallShape = createShape({ {-1.5f, -2.0f}, {-1.0f, -2.0f}, {-1.0f, 2.0f}, {-1.5f, 2.0f} });
    Shape floorShape = createShape({ {-2.0f, -0.2f}, {2.0f, -0.2f}, {2.0f, 0.2f}, {-2.0f, 0.2f} });

    // 2. Add to world at specific positions
    world.push_back({ Body(0.0f, 1.5f, 0.1f), &boxShape });   // Square falling
    world[0].body.velocity.x = 1.5f; // Initial x velocity
    world.push_back({ Body(0.0f, -1.8f, 0.0f), &floorShape }); // Floor at bottom
    world.push_back({ Body(-1.0f, 0.0f, 0.0f), &wallShape });  // Left wall
    world.push_back({ Body(3.0f, 0.0f, 0.0f), &wallShape });   // Right wall
    while (!glfwWindowShouldClose(window)) {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f); // Dark grey background
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 2. Setup your camera/projection
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        float aspect = (float)width / (float)height;
        glOrtho(-2.0 * aspect, 2.0 * aspect, -2.0, 2.0, -1.0, 1.0);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        float currentFrame = glfwGetTime();
        float dt = currentFrame - lastFrame;
        if (dt > 0.05f) dt = 0.05f; // Never let the physics step be larger than 50ms
        lastFrame = currentFrame;
        Vector2 gravity = {0.0f, -9.81f};
        for (auto& obj : world) {
            Vector2 force = { gravity.x * obj.body.mass, gravity.y * obj.body.mass };
            obj.body.updateRK4(dt, force);
            obj.updateRotation(dt);
        }
        std::vector<std::pair<int, int>> collisions;
        std::vector<Manifold> manifolds;
        // --- B. COLLISION DETECTION & RESOLUTION ---
        for (size_t i = 0; i < world.size(); i++) {
            for (size_t j = i + 1; j < world.size(); j++) {
                // Skip collision detection between static objects
                if (world[i].body.mass == 0.0f && world[j].body.mass == 0.0f) continue;

                // This is the "Universal" SAT check we built
                Manifold m = CheckCollisionAndFindManifold(world[i], world[j]);

                if (m.colliding) {
                    printf("Colliding! Depth: %f, Normal: (%f, %f), Contact: (%f, %f)\n", m.depth, m.normal.x, m.normal.y, m.contactPoint.x, m.contactPoint.y);
                    if (m.colliding && m.depth > 0.001f) { // Ignore tiny depth "jitters"
                        ResolveCollision(world[i], world[j], m);
                    }
                }
            }
        }

        // --- C. RENDER ---
        for (auto& obj : world) {
            if (obj.body.mass > 0.0f) glColor3f(1.0f, 0.0f, 0.0f); 
            else glColor3f(0.0f, 0.0f, 1.0f);
            drawPolygon(obj); // Pass object to a function that loops through localVertices
        }
        // 8. RENDER IMGUI & SWAP
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
        glfwPollEvents();

    }
    glfwTerminate();
    return 0;
    
}