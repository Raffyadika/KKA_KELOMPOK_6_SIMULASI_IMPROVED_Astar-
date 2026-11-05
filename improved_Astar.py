import math

DIR_8 = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]
DIR_16 = DIR_8 + [(-2, 1), (-1, 2), (1, 2), (2, 1), (2, -1), (1, -2), (-1, -2), (-2, -1)]


def free(grid, n):  # di luar grid dianggap obstacle
    return 0 <= n[0] < len(grid) and 0 <= n[1] < len(grid[0]) and grid[n[0]][n[1]] == 0


def dist(a, b):  # DIST sekaligus h(n): jarak Euclidean
    return math.dist(a, b)


def angle(a, b):
    return math.atan2(b[0] - a[0], b[1] - a[1])


def w(h, h0):  
    return 1 + h / h0


def line_of_sight(a, b, grid):  # cek semua sel yang dilewati garis a -> b
    (r, c), (r1, c1) = a, b
    nr, nc = abs(r1 - r), abs(c1 - c)
    sr, sc = (1 if r1 > r else -1), (1 if c1 > c else -1)
    ir = ic = 0
    while ir < nr or ic < nc:
        t = (1 + 2 * ic) * nr - (1 + 2 * ir) * nc
        if t <= 0:
            c, ic = c + sc, ic + 1
        if t >= 0:
            r, ir = r + sr, ir + 1
        if not free(grid, (r, c)):
            return False
    return True


def get_neighbors(node, target, grid):
    r, c = node
    if any(not free(grid, (r + dr, c + dc)) for dr, dc in DIR_8):
        dirs, k = DIR_8, 5
    else:
        dirs, k = DIR_16, 7
    alpha = angle(node, target)
    gap = lambda d: abs((math.atan2(*d) - alpha + math.pi) % (2 * math.pi) - math.pi)
    dirs = sorted(dirs, key=gap)[:k]  # k arah paling dekat dengan arah ke target
    return [(r + dr, c + dc) for dr, dc in dirs if free(grid, (r + dr, c + dc))]


class Side:  # satu arah pencarian: open, closed, g(n), dan parent (untuk CONNECT-PATHS)
    def __init__(self, origin):
        self.open, self.closed = {origin}, {origin}
        self.g, self.parent = {origin: 0}, {origin: None}


def expand_best(pnode, target, side, grid, h0):
    for n in get_neighbors(pnode, target, grid):
        if n not in side.closed:
            g = side.g[pnode] + dist(pnode, n)
            if n not in side.open or g < side.g[n]:
                side.g[n], side.parent[n] = g, pnode
                side.open.add(n)
    if not side.open:
        return None
    s = min(side.open, key=lambda n: side.g[n] + w(dist(n, target), h0) * dist(n, target))
    side.open.remove(s)
    side.closed.add(s)
    return s


def connect_paths(m, a, b):  # path1: start..m, path2: (setelah m)..goal
    def trace(parent, n):
        out = []
        while n is not None:
            out.append(n)
            n = parent[n]
        return out
    return trace(a.parent, m)[::-1], trace(b.parent, m)[1:]


def bidirectional_search(start, goal, grid):
    h0 = dist(start, goal) or 1
    a, b = Side(start), Side(goal)
    pnode1, pnode2 = start, goal
    while True:
        s1 = expand_best(pnode1, goal, a, grid, h0)
        s2 = expand_best(pnode2, start, b, grid, h0)
        if s1 is None or s2 is None:  
            return None
        if s1 in b.open:
            return connect_paths(s1, a, b)
        if s2 in a.closed:
            return connect_paths(s2, a, b)
        pnode1, pnode2 = s1, s2


def remove_redundant(path, grid):
    i = len(path) - 1
    result = [path[i]]
    while i > 0:
        j = i - 1
        while j > 0 and line_of_sight(path[i], path[j - 1], grid):
            j -= 1
        result.append(path[j])
        i = j
    return result[::-1]


def improved_astar_search(start, goal, grid):
    res = bidirectional_search(start, goal, grid)
    if res is None:
        return None, None
    raw = res[0] + res[1]
    return remove_redundant(raw, grid), raw

MAP = """
........................................
............#...........................
............#.....#########.............
............#.....#.......#.............
............#.....#.......#.............
........######..S.#..###..#.............
..................#..#.#..#.....#######.
..................#..#.#..#.......#.....
.......#########..#..#.#..#...#...#.....
...............#.....#.#......#...#.....
...............#.....#.#......#.........
...............#######.#......##########
.......................#................
...........................#............
.....#######...............#.........G..
.....#.....................#............
"""

if __name__ == "__main__":
    rows = MAP.strip().splitlines()
    grid = [[1 if ch == "#" else 0 for ch in row] for row in rows]
    find = lambda ch: next((r, row.index(ch)) for r, row in enumerate(rows) if ch in row)
    start, goal = find("S"), find("G")

    path, raw = improved_astar_search(start, goal, grid)
    if path is None:
        raise SystemExit("Gagal menemukan jalur")
    length = lambda p: sum(dist(x, y) for x, y in zip(p, p[1:]))
    print(f"Sebelum REMOVE-REDUNDANT: {len(raw)} node, panjang {length(raw):.2f}")
    print(f"Sesudah                 : {len(path)} node, panjang {length(path):.2f}")

    canvas = [["#" if v else "." for v in row] for row in grid]
    for r, c in raw:
        canvas[r][c] = "o"
    for r, c in path:
        canvas[r][c] = "*"
    canvas[start[0]][start[1]], canvas[goal[0]][goal[1]] = "S", "G"
    print("\n".join("".join(row) for row in canvas))

    try:  # visualisasi (opsional)
        import matplotlib.pyplot as plt
        plt.imshow(grid, cmap="Greys")
        plt.plot([c for _, c in raw], [r for r, _ in raw], "b--", lw=1, label="sebelum")
        plt.plot([c for _, c in path], [r for r, _ in path], "r-o", lw=2, label="sesudah")
        plt.legend()
        plt.show()
    except ImportError:
        pass
