import matplotlib.pyplot as plt
import matplotlib.animation as animation
from Engine_2D import PhysicsObject, Vector2D, check_collision, collision_response, AABB

obj1 = PhysicsObject(position=Vector2D(-2, 0), velocity=Vector2D(1, 0), mass=2)
obj1.aabb = AABB(Vector2D(-1.5, -0.5), Vector2D(-1, 0.5))  # Rectangle coordinates

obj2 = PhysicsObject(position=Vector2D(3, 0), velocity=Vector2D(-1, 0), mass=1)
obj2.aabb = AABB(Vector2D(2, -0.5), Vector2D(2.5, 0.5))  # Rectangle coordinates


# Create a function to update the plot
def update_plot(num):
    global obj1, obj2
    obj1.update(0.1)
    obj2.update(0.1)

    ax.clear()
    ax.set_xlim(-3, 4)
    ax.set_ylim(-1, 1)

    # Plot objects as points
    ax.plot([obj1.position.x], [obj1.position.y], "ro", label="Obj1")
    ax.plot([obj2.position.x], [obj2.position.y], "bo", label="Obj2")

    ax.legend()


# Create a Matplotlib figure and axis
fig, ax = plt.subplots()

# Set the initial plot properties
ax.set_xlim(-3, 4)
ax.set_ylim(-1, 1)
ax.plot([obj1.position.x], [obj1.position.y], "ro", label="Obj1")
ax.plot([obj2.position.x], [obj2.position.y], "bo", label="Obj2")

ax.legend()

# Create an animation with shorter interval and more frames
ani = animation.FuncAnimation(fig, update_plot, frames=100, interval=50, repeat=True)

# Display the animation
plt.show()
