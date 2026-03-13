#include <GLFW/glfw3.h>
#include "Body.h"
#include <cmath>
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include <vector>
#include <cstdio>


struct Shape {
    std::vector<ImVec2> localVertices; // Points relative to (0,0)
    std::vector<ImVec2> localNormals;  // Pre-calculated normals
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
std::vector<ImVec2> getRotatedNormals(const PhysicsObject& obj) {
    std::vector<ImVec2> worldNormals;
    for (const auto& localN : obj.shape->localNormals) {
        // Dot product projection to transform the vector
        float x = (localN.x * obj.localRight.x) + (localN.y * obj.localUp.x);
        float y = (localN.x * obj.localRight.y) + (localN.y * obj.localUp.y);
        worldNormals.push_back({ x, y });
    }
    return worldNormals;
}
struct Projection { float min; float max; };

Projection projectShape(const PhysicsObject& obj, ImVec2 axis) {
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
Shape createShape(std::vector<ImVec2> vertices) {
    Shape s;
    s.localVertices = vertices;
    int count = vertices.size();
    for (int i = 0; i < count; i++) {
        ImVec2 p1 = vertices[i];
        ImVec2 p2 = vertices[(i + 1) % count];
        ImVec2 edge = { p2.x - p1.x, p2.y - p1.y };
        ImVec2 normal = { -edge.y, edge.x }; // Perpendicular
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
    ImVec2 normal;    // The axis along which the overlap occurred
};

Manifold CheckCollisionAndFindManifold(PhysicsObject& A, PhysicsObject& B) {
    Manifold m;
    m.colliding = false;
    m.depth = FLT_MAX;
    
    // 1. Get rotated normals for both shapes
    std::vector<ImVec2> axes = getRotatedNormals(A);
    std::vector<ImVec2> bNormals = getRotatedNormals(B);
    axes.insert(axes.end(), bNormals.begin(), bNormals.end());

    // 2. The SAT Loop
    for (auto& axis : axes) {
        Projection projA = projectShape(A, axis);
        Projection projB = projectShape(B, axis);

        // Check for a gap
        if (projA.max < projB.min || projB.max < projA.min) {
            return { false, 0.0f, {0,0} }; // Found a gap, no collision!
        }

        // 3. Find the overlap depth on this axis
        float overlap = std::min(projA.max, projB.max) - std::max(projA.min, projB.min);
        
        // Is this the smallest overlap so far? (The smallest overlap is the Collision Normal)
        if (overlap < m.depth) {
            m.depth = overlap;
            m.normal = axis;
        }
    }

    m.colliding = true;
    return m;
}
void ResolveCollision(PhysicsObject& A, PhysicsObject& B, Manifold m) {
    const float percent = 0.8f; // How much of the overlap to fix (20% to 80%)
    const float slop = 0.01f;    // Allowed overlap
    
    float totalInvMass = A.body.invMass + B.body.invMass;
    if (totalInvMass == 0.0f) return;

    // Position correction
    float magnitude = (std::max(m.depth - slop, 0.0f) / totalInvMass) * percent;
    
    A.body.position.x += m.normal.x * magnitude * A.body.invMass;
    A.body.position.y += m.normal.y * magnitude * A.body.invMass;
    
    B.body.position.x -= m.normal.x * magnitude * B.body.invMass;
    B.body.position.y -= m.normal.y * magnitude * B.body.invMass;

    // Velocity resolution (impulse)
    Vector2 relativeVelocity = { A.body.velocity.x - B.body.velocity.x, A.body.velocity.y - B.body.velocity.y };
    float velocityAlongNormal = relativeVelocity.x * m.normal.x + relativeVelocity.y * m.normal.y;
    
    if (velocityAlongNormal > 0) return; // Already separating
    
    float restitution = 0.0f; // No bounce
    float impulse = -(1 + restitution) * velocityAlongNormal / totalInvMass;
    
    Vector2 impulseVector = { m.normal.x * impulse, m.normal.y * impulse };
    
    A.body.velocity.x += impulseVector.x * A.body.invMass;
    A.body.velocity.y += impulseVector.y * A.body.invMass;
    
    B.body.velocity.x -= impulseVector.x * B.body.invMass;
    B.body.velocity.y -= impulseVector.y * B.body.invMass;
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
    world.push_back({ Body(0.0f, -1.8f, 0.0f), &floorShape }); // Floor at bottom
    world.push_back({ Body(-1.0f, 1.0f, 0.0f), &wallShape });  // Left wall
    world.push_back({ Body(3.0f, 1.0f, 0.0f), &wallShape });   // Right wall
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

                // This is the "Universal" SAT check we built
                Manifold m = CheckCollisionAndFindManifold(world[i], world[j]);

                if (m.colliding) {
                    printf("Colliding! Depth: %f, Normal: (%f, %f)\n", m.depth, m.normal.x, m.normal.y);
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