#pragma once
#include <vector>
#include "Vector2.h"
#include <cmath>
struct Shape {
    // These are constant "Blueprints"
    std::vector<Vector2> localVertices; // Points defined in CCW order around (0,0)
    std::vector<Vector2> localNormals;  // Pre-calculated normals for each edge

    // Constructor to automatically calculate normals upon creation
    Shape(std::vector<Vector2> vertices) : localVertices(vertices) {
        int count = vertices.size();
        for (int i = 0; i < count; i++) {
            Vector2 p1 = vertices[i];
            Vector2 p2 = vertices[(i + 1) % count];
            
            // Calculate edge vector
            Vector2 edge = { p2.x - p1.x, p2.y - p1.y };
            
            // Perpendicular vector: (-edge.y, edge.x)
            // Because our vertices are CCW, this normal points OUTWARD.
            Vector2 normal = { -edge.y, edge.x };
            
            // Normalize the vector (length of 1)
            float len = sqrt(normal.x * normal.x + normal.y * normal.y);
            localNormals.push_back({ normal.x / len, normal.y / len });
        }
    }
};