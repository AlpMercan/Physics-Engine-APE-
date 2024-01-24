
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
