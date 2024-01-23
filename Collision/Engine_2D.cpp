#include <iostream>
#include <cmath>

class Vector2D
{
public:
    float x, y;

    Vector2D(float x = 0.0, float y = 0.0) : x(x), y(y) {}

    Vector2D operator+(const Vector2D &other) const
    {
        return Vector2D(x + other.x, y + other.y);
    }

    Vector2D operator-(const Vector2D &other) const
    {
        return Vector2D(x - other.x, y - other.y);
    }

    Vector2D operator*(float scalar) const
    {
        return Vector2D(x * scalar, y * scalar);
    }

    Vector2D operator/(float scalar) const
    {
        if (scalar != 0.0)
        {
            return Vector2D(x / scalar, y / scalar);
        }
        else
        {
            throw std::invalid_argument("Cannot divide by zero");
        }
    }

    void print() const
    {
        std::cout << "Vector2D (" << x << ", " << y << ")" << std::endl;
    }

    float magnitude() const
    {
        return std::sqrt(x * x + y * y);
    }

    Vector2D normalize() const
    {
        float mag = magnitude();
        if (mag != 0.0)
        {
            return *this / mag;
        }
        else
        {
            throw std::invalid_argument("Cannot normalize a zero vector");
        }
    }
};

class PhysicsObject
{
public:
    Vector2D position, velocity, acceleration;
    float mass, radius;

    PhysicsObject(Vector2D position = Vector2D(), Vector2D velocity = Vector2D(),
                  Vector2D acceleration = Vector2D(), float mass = 1.0, float radius = 0.0)
        : position(position), velocity(velocity), acceleration(acceleration), mass(mass), radius(radius) {}

    void update(float dt)
    {
        velocity = velocity + acceleration * dt;
        position = position + velocity * dt;
    }

    void print() const
    {
        std::cout << "PhysicsObject(Position: " << position.x << ", " << position.y
                  << ", Velocity: " << velocity.x << ", " << velocity.y
                  << ", Acceleration: " << acceleration.x << ", " << acceleration.y
                  << ", Mass: " << mass << ")" << std::endl;
    }
};
class AABB
{
public:
    Vector2D min_point, max_point;

    AABB(Vector2D min_point, Vector2D max_point) : min_point(min_point), max_point(max_point) {}

    bool intersects(const AABB &other) const
    {
        return !(max_point.x < other.min_point.x || min_point.x > other.max_point.x ||
                 max_point.y < other.min_point.y || min_point.y > other.max_point.y);
    }
};

bool check_collision(const PhysicsObject &obj1, const PhysicsObject &obj2)
{
    const AABB &aabb1 = obj1.aabb;
    const AABB &aabb2 = obj2.aabb;

    return aabb1.max_point.x >= aabb2.min_point.x &&
           aabb1.min_point.x <= aabb2.max_point.x &&
           aabb1.max_point.y >= aabb2.min_point.y &&
           aabb1.min_point.y <= aabb2.max_point.y;
}

bool check_collision_circle(const PhysicsObject &circle1, const PhysicsObject &circle2)
{
    float dx = circle1.position.x - circle2.position.x;
    float dy = circle1.position.y - circle2.position.y;
    float distance = std::sqrt(dx * dx + dy * dy);

    return distance <= (circle1.radius + circle2.radius);
}