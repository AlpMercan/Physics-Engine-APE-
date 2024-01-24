import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from Engine_2D import (
    PhysicsObject,
    Vector2D,
    check_collision,
    collision_response,
    AABB,
    check_collision_circle,
    collision_response_circle_box,
    QuadtreeNode,
    resolve_overlap,
)

# Define the boundary of the entire simulation
simulation_area = AABB(Vector2D(-5, -5), Vector2D(5, 5))
quadtree_capacity = 20  # Adjust as needed
quadtree = QuadtreeNode(simulation_area, quadtree_capacity)

total_simulation_time = 10.0
time_step = 0.1
accumulated_time = 0.0
N = 20
objects = []


# for _ in range(N):
#   position = Vector2D(np.random.uniform(-3, 3), np.random.uniform(-3, 3))
#   velocity = Vector2D(np.random.uniform(-1, 1), np.random.uniform(-1, 1))
##  obj = PhysicsObject(position=position, velocity=velocity, mass=1, radius=0.05)
# objects.append(obj)
# Define a constant speed
constant_speed = 1.0

for _ in range(N):
    position = Vector2D(np.random.uniform(-3, 3), np.random.uniform(-3, 3))

    # Create a random direction vector with a constant speed
    angle = np.random.uniform(0, 2 * np.pi)
    velocity = Vector2D(constant_speed * np.cos(angle), constant_speed * np.sin(angle))

    obj = PhysicsObject(position=position, velocity=velocity, mass=1, radius=0.05)
    objects.append(obj)

box = AABB(Vector2D(-3, -3), Vector2D(3, 3))


def update_plot(num, objects, ax, box, quadtree):
    global accumulated_time, total_simulation_time

    if accumulated_time >= total_simulation_time:
        return

    quadtree.clear()
    for obj in objects:
        quadtree.insert(obj)

    ax.clear()
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)

    box_x = [
        box.min_point.x,
        box.min_point.x,
        box.max_point.x,
        box.max_point.x,
        box.min_point.x,
    ]
    box_y = [
        box.min_point.y,
        box.max_point.y,
        box.max_point.y,
        box.min_point.y,
        box.min_point.y,
    ]
    ax.plot(box_x, box_y, "k-")

    for i, obj in enumerate(objects):
        obj.update(time_step)

        if box.intersects_with_circle(obj):
            collision_response_circle_box(obj, box)

        search_range = AABB(
            Vector2D(obj.position.x - obj.radius, obj.position.y - obj.radius),
            Vector2D(obj.position.x + obj.radius, obj.position.y + obj.radius),
        )

        potential_colliders = quadtree.query(search_range, [])
        print(potential_colliders)

        for collider in potential_colliders:
            if collider is not obj and check_collision_circle(obj, collider):
                print("çarptim")
                resolve_overlap(obj, collider)
                collision_response(obj, collider)

        ax.plot([obj.position.x], [obj.position.y], "o")

    accumulated_time += time_step


fig, ax = plt.subplots()
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)

ani = animation.FuncAnimation(
    fig,
    update_plot,
    fargs=(objects, ax, box, quadtree),
    frames=int(total_simulation_time / time_step),
    interval=time_step * 1000,
    repeat=False,
)

plt.show()
