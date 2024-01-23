import matplotlib.pyplot as plt
import matplotlib.animation as animation
from Engine_2D import (
    PhysicsObject,
    Vector2D,
    check_collision,
    collision_response,
    AABB,
    check_collision_circle,
)


obj1 = PhysicsObject(
    position=Vector2D(-2, 0), velocity=Vector2D(1, 0), mass=1, radius=0.05
)
obj2 = PhysicsObject(
    position=Vector2D(3, 0), velocity=Vector2D(-1, 0), mass=1, radius=0.05
)


def update_plot(num):
    global obj1, obj2

    obj1.update(0.1)
    obj2.update(0.1)

    if check_collision_circle(obj1, obj2):
        collision_response(obj1, obj2)

    ax.clear()
    ax.set_xlim(-3, 4)
    ax.set_ylim(-1, 1)

    ax.plot([obj1.position.x], [obj1.position.y], "ro", label="Obj1")
    ax.plot([obj2.position.x], [obj2.position.y], "bo", label="Obj2")

    ax.legend()


fig, ax = plt.subplots()

ax.set_xlim(-3, 4)
ax.set_ylim(-1, 1)
ax.plot([obj1.position.x], [obj1.position.y], "ro", label="Obj1")
ax.plot([obj2.position.x], [obj2.position.y], "bo", label="Obj2")

ax.legend()


ani = animation.FuncAnimation(fig, update_plot, frames=100, interval=50, repeat=False)


ani.save(
    "C:/Users/alpme/OneDrive/Desktop/Projeler/Physics Engine/Collision/simulation.gif",
    writer="imagemagick",
    fps=30,
)


plt.show()
