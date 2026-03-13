#pragma once
struct Vector2 {
    float x, y;

    Vector2 operator+(const Vector2& other) const { return {x + other.x, y + other.y}; }
    Vector2 operator*(float scalar) const { return {x * scalar, y * scalar}; }
    void operator+=(const Vector2& other) { x += other.x; y += other.y; }
    Vector2 operator-(const Vector2& v) const { return { x - v.x, y - v.y }; }
};