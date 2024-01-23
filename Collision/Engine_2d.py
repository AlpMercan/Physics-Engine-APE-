import numpy as np


##### Vector implimentation
class Vector2D:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2D(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        if isinstance(scalar, (int, float)):  # Check if scalar is a number
            return Vector2D(self.x * scalar, self.y * scalar)
        else:
            raise TypeError(f"Cannot multiply Vector2D and {type(scalar)}")

    def __truediv__(self, scalar):
        if isinstance(scalar, (int, float)) and scalar != 0:
            return Vector2D(self.x / scalar, self.y / scalar)
        else:
            raise TypeError(f"Cannot divide Vector2D by {type(scalar)} or zero")

    def __repr__(self):
        return f"Vector2D ({self.x},{self.y})"

    def magnitude(self):
        return np.sqrt(self.x**2 + self.y**2)

    def normalize(self):
        mag = self.magnitude()
        if mag == 0:
            return Vector2D()
        return Vector2D(self.x / mag, self.y / mag)


##### Physical Properties
class PhysicsObject:
    def __init__(
        self, position=None, velocity=None, acceleration=None, mass=1, radius=None
    ):
        self.position = position if position is not None else Vector2D()
        self.velocity = velocity if velocity is not None else Vector2D()
        self.acceleration = acceleration if acceleration is not None else Vector2D()
        self.mass = mass
        self.radius = radius

    def update(self, dt):
        self.velocity += self.acceleration * dt
        self.position += self.velocity * dt

    def __repr__(self):
        return f"PhysicsObject(Position: {self.position}, Velocity: {self.velocity}, Acceleration: {self.acceleration}, Mass: {self.mass})"


##### Collision Detection
class AABB:
    def __init__(self, min_point, max_point):
        self.min_point = min_point  # down left
        self.max_point = max_point  # top right

    def intersects(self, other):
        if (
            self.max_point.x < other.min_point.x
            or self.min_point.x > other.max_point.x
            or self.max_point.y < other.min_point.y
            or self.min_point.y > other.max_point.y
        ):
            return False
        return True


def check_collision(obj1, obj2):
    return (
        obj1.aabb.max_point.x >= obj2.aabb.min_point.x
        and obj1.aabb.min_point.x <= obj2.aabb.max_point.x
        and obj1.aabb.max_point.y >= obj2.aabb.min_point.y
        and obj1.aabb.min_point.y <= obj2.aabb.max_point.y
    )


def check_collision_circle(circle1, circle2):
    dx = circle1.position.x - circle2.position.x
    dy = circle1.position.y - circle2.position.y
    distance = (dx**2 + dy**2) ** 0.5

    if distance <= (circle1.radius + circle2.radius):
        return True  # Collision detected
    return False


def collision_response(obj1, obj2):
    m1, m2 = obj1.mass, obj2.mass
    v1_old, v2_old = obj1.velocity, obj2.velocity

    new_v1_x = (v1_old.x * (m1 - m2) + 2 * m2 * v2_old.x) / (m1 + m2)
    new_v1_y = (v1_old.y * (m1 - m2) + 2 * m2 * v2_old.y) / (m1 + m2)

    new_v2_x = (v2_old.x * (m2 - m1) + 2 * m1 * v1_old.x) / (m1 + m2)
    new_v2_y = (v2_old.y * (m2 - m1) + 2 * m1 * v1_old.y) / (m1 + m2)

    obj1.velocity = Vector2D(new_v1_x, new_v1_y)
    obj2.velocity = Vector2D(new_v2_x, new_v2_y)
