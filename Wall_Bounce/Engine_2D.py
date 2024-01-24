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
        self,
        position=None,
        velocity=None,
        acceleration=None,
        mass=1,
        radius=None,
        angle=0,
    ):
        self.position = position if position is not None else Vector2D()
        self.velocity = velocity if velocity is not None else Vector2D()
        self.acceleration = acceleration if acceleration is not None else Vector2D()
        self.mass = mass
        self.radius = radius
        self.angle = angle
        self.forces = Vector2D()
        self.angular_velocity = 0
        self.angular_acceleration = 0
        self.torque = 0

    def apply_force(self, force, apply_point=None):
        self.forces += force
        if apply_point is not None:
            r = apply_point - self.position
            self.torque += r.x * force.y - r.y * force.x

    def apply_torque(self, torque):
        self.torque += torque

    def update(self, dt):
        if self.mass != 0:
            self.acceleration = self.forces / self.mass
        else:
            self.acceleration = Vector2D()

        self.velocity += self.acceleration * dt
        self.position += self.velocity * dt

        if self.mass != 0:
            moment_of_inertia = (
                (2 / 5) * self.mass * (self.radius**2) if self.radius else self.mass
            )
            self.angular_acceleration = self.torque / moment_of_inertia
        else:
            self.angular_acceleration = 0

        self.angular_velocity += self.angular_acceleration * dt
        self.angle += self.angular_velocity * dt

        self.angle %= 360

        self.forces = Vector2D()
        self.torque = 0

    def __repr__(self):
        return f"PhysicsObject(Position: {self.position}, Velocity: {self.velocity}, Acceleration: {self.acceleration}, Mass: {self.mass})"


##### Collision Detection
class AABB:
    def __init__(self, min_point, max_point):
        self.min_point = min_point  # down left
        self.max_point = max_point  # top right

    def intersects1(self, circle):
        # Find the closest point to the circle within the box
        closest_x = max(self.min_point.x, min(circle.position.x, self.max_point.x))
        closest_y = max(self.min_point.y, min(circle.position.y, self.max_point.y))

        # Calculate the distance between the circle's center and this closest point
        distance_x = circle.position.x - closest_x
        distance_y = circle.position.y - closest_y

        # The circle and the box intersect if the distance is less than the circle's radius
        distance_squared = distance_x**2 + distance_y**2
        return distance_squared < (circle.radius**2)

    def contains(self, obj):
        # Check if a circle object is entirely within the AABB
        # This checks that all edges of the circle are within the box
        left_edge = obj.position.x - obj.radius >= self.min_point.x
        right_edge = obj.position.x + obj.radius <= self.max_point.x
        bottom_edge = obj.position.y - obj.radius >= self.min_point.y
        top_edge = obj.position.y + obj.radius <= self.max_point.y

        return left_edge and right_edge and bottom_edge and top_edge

    def intersects(self, other):
        # Check if two AABBs intersect
        # This is a collision check between two rectangles (AABBs)
        return (
            self.min_point.x <= other.max_point.x
            and self.max_point.x >= other.min_point.x
            and self.min_point.y <= other.max_point.y
            and self.max_point.y >= other.min_point.y
        )

    def intersects_with_circle(self, circle):
        # Check if the AABB intersects with a circle (PhysicsObject)
        closest_x = max(self.min_point.x, min(circle.position.x, self.max_point.x))
        closest_y = max(self.min_point.y, min(circle.position.y, self.max_point.y))

        # Calculate the distance between the circle's center and this closest point
        distance_x = circle.position.x - closest_x
        distance_y = circle.position.y - closest_y

        # The circle and the AABB intersect if the distance is less than the circle's radius
        distance_squared = distance_x**2 + distance_y**2
        return distance_squared < (circle.radius**2)


def collision_response_circle_box(circle, box):
    # Adjust position if the circle is outside the box
    if circle.position.x - circle.radius < box.min_point.x:
        circle.position.x = box.min_point.x + circle.radius
        circle.velocity.x *= -1
    elif circle.position.x + circle.radius > box.max_point.x:
        circle.position.x = box.max_point.x - circle.radius
        circle.velocity.x *= -1

    if circle.position.y - circle.radius < box.min_point.y:
        circle.position.y = box.min_point.y + circle.radius
        circle.velocity.y *= -1
    elif circle.position.y + circle.radius > box.max_point.y:
        circle.position.y = box.max_point.y - circle.radius
        circle.velocity.y *= -1


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


def collision_response(circle1, circle2):
    # Calculate the normal and tangent vectors at the point of collision
    normal = (circle2.position - circle1.position).normalize()
    tangent = Vector2D(-normal.y, normal.x)

    # Project the velocities onto the normal and tangent vectors
    v1n = normal.dot(circle1.velocity)
    v1t = tangent.dot(circle1.velocity)
    v2n = normal.dot(circle2.velocity)
    v2t = tangent.dot(circle2.velocity)

    # Update the normal components of the velocities
    v1n, v2n = v2n, v1n

    # Convert the scalar normal and tangential velocities into vectors
    v1n_vec = v1n * normal
    v1t_vec = v1t * tangent
    v2n_vec = v2n * normal
    v2t_vec = v2t * tangent

    # Update the velocities
    circle1.velocity = v1n_vec + v1t_vec
    circle2.velocity = v2n_vec + v2t_vec


def resolve_overlap(circle1, circle2):
    # Calculate the overlap distance
    overlap = (circle1.radius + circle2.radius) - np.linalg.norm(
        circle1.position - circle2.position
    )

    # Calculate the direction vector from circle2 to circle1
    direction = (circle1.position - circle2.position).normalize()

    # Move each circle away from the other by half the overlap distance
    circle1.position += direction * (overlap / 2)
    circle2.position -= direction * (overlap / 2)


class QuadtreeNode:
    def __init__(self, boundary, capacity):
        # Boundary is an AABB that defines the space boundaries of this node
        self.boundary = boundary
        self.capacity = (
            capacity  # Capacity of objects this node can hold before subdividing
        )
        self.objects = []  # Objects (circles) in this node
        self.divided = False
        self.northwest = None  # Children of this quadtree node
        self.northeast = None
        self.southwest = None
        self.southeast = None

    def subdivide(self):
        x, y = self.boundary.min_point.x, self.boundary.min_point.y
        w, h = self.boundary.max_point.x - x, self.boundary.max_point.y - y
        half_w, half_h = w / 2, h / 2

        nw = AABB(Vector2D(x, y + half_h), Vector2D(x + half_w, y + h))
        self.northwest = QuadtreeNode(nw, self.capacity)

        ne = AABB(Vector2D(x + half_w, y + half_h), Vector2D(x + w, y + h))
        self.northeast = QuadtreeNode(ne, self.capacity)

        sw = AABB(Vector2D(x, y), Vector2D(x + half_w, y + half_h))
        self.southwest = QuadtreeNode(sw, self.capacity)

        se = AABB(Vector2D(x + half_w, y), Vector2D(x + w, y + half_h))
        self.southeast = QuadtreeNode(se, self.capacity)

        self.divided = True

    def insert(self, obj):
        if not self.boundary.contains(obj):
            return False

        if len(self.objects) < self.capacity:
            self.objects.append(obj)
            return True

        if not self.divided:
            self.subdivide()

        return (
            self.northwest.insert(obj)
            or self.northeast.insert(obj)
            or self.southwest.insert(obj)
            or self.southeast.insert(obj)
        )

    def query(self, range, found):
        if not self.boundary.intersects(range):
            return found

        for obj in self.objects:
            if range.contains(obj):
                found.append(obj)

        if self.divided:
            self.northwest.query(range, found)
            self.northeast.query(range, found)
            self.southwest.query(range, found)
            self.southeast.query(range, found)

        return found

    def clear(self):
        self.objects.clear()
        if self.divided:
            self.northwest.clear()
            self.northeast.clear()
            self.southwest.clear()
            self.southeast.clear()
            self.divided = False
