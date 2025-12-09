#ifndef VEC2_HPP
#define VEC2_HPP

#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>

// 2D Vector class for flight dynamics
struct Vec2 {
    double x, y;
    
    Vec2(double x = 0.0, double y = 0.0) : x(x), y(y) {}
    
    // Vector addition
    Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }
    
    // Vector subtraction
    Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }
    
    // Scalar multiplication
    Vec2 operator*(double scalar) const {
        return Vec2(x * scalar, y * scalar);
    }
    
    // Scalar division
    Vec2 operator/(double scalar) const {
        return Vec2(x / scalar, y / scalar);
    }
    
    // Dot product
    double dot(const Vec2& other) const {
        return x * other.x + y * other.y;
    }
    
    // Magnitude (length)
    double magnitude() const {
        return std::sqrt(x * x + y * y);
    }
    
    // Magnitude squared (useful for comparisons without sqrt)
    double magnitudeSquared() const {
        return x * x + y * y;
    }
    
    // Normalize (return unit vector)
    Vec2 normalized() const {
        double mag = magnitude();
        if (mag < 1e-9) return Vec2(0, 0);
        return Vec2(x / mag, y / mag);
    }
    
    // Rotate by angle (radians)
    Vec2 rotated(double angle) const {
        double cos_a = std::cos(angle);
        double sin_a = std::sin(angle);
        return Vec2(x * cos_a - y * sin_a, x * sin_a + y * cos_a);
    }
    
    // Get angle (radians)
    double angle() const {
        return std::atan2(y, x);
    }
    
    // Print helper
    void print(const std::string& name = "") const {
        std::cout << name << "(" << std::fixed << std::setprecision(2) 
                  << x << ", " << y << ")";
    }
};

// Scalar * Vector (for commutative multiplication)
inline Vec2 operator*(double scalar, const Vec2& vec) {
    return vec * scalar;
}

#endif // VEC2_HPP
