#include <iostream>
#include <vector>
#include "Vector2D.h"      // Include the Vector2D class
#include "PhysicsObject.h" // Include the PhysicsObject class

// Assuming the existence of functions like check_collision_circle() and collision_response()

void update_plot(PhysicsObject &obj1, PhysicsObject &obj2)
{
    // Update objects
    obj1.update(0.1);
    obj2.update(0.1);

    // Check for collision and respond
    if (check_collision_circle(obj1, obj2))
    {
        collision_response(obj1, obj2);
    }

    draw_circle(obj1.position, obj1.radius, "red");
    draw_circle(obj2.position, obj2.radius, "blue");
}

int main()
{
    PhysicsObject obj1(Vector2D(-2, 0), Vector2D(1, 0), 1, 0.05);
    PhysicsObject obj2(Vector2D(3, 0), Vector2D(-1, 0), 1, 0.05);

    for (int frame = 0; frame < 1000; ++frame)
    {
        update_plot(obj1, obj2);
    }

    return 0;
}
