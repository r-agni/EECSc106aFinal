from collections import deque

def bfs_path(grid, start, goal):
    """
    Returns shortest path (as list of (r,c)) from start to goal using BFS,
    or None if unreachable.
    0 = free, 1 = obstacle
    """
    rows = len(grid)
    cols = len(grid[0]) if rows else 0

    sr, sc = start
    gr, gc = goal

    if not (0 <= sr < rows and 0 <= sc < cols): 
        return None
    if not (0 <= gr < rows and 0 <= gc < cols): 
        return None
    if grid[sr][sc] == 1 or grid[gr][gc] == 1:
        return None

    # 4-neighbor moves: up, down, left, right
    moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    q = deque()
    q.append((sr, sc))

    visited = set([(sr, sc)])
    parent = { (sr, sc): None }

    while q:
        r, c = q.popleft()

        if (r, c) == (gr, gc):
            # reconstruct path
            path = []
            cur = (r, c)
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            path.reverse()
            return path

        for dr, dc in moves:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols \
               and grid[nr][nc] == 0 \
               and (nr, nc) not in visited:
                visited.add((nr, nc))
                parent[(nr, nc)] = (r, c)
                q.append((nr, nc))

    return None


# ---- Example ----
if __name__ == "__main__":
    grid = [
        [0,0,0,0,0],
        [0,1,1,1,0],
        [0,0,0,1,0],
        [0,1,0,0,0],
        [0,0,0,1,0],
    ]
    start = (0, 0)
    goal  = (4, 4)

    path = bfs_path(grid, start, goal)
    print("Path:", path)

    # Optional: visualize path
    if path:
        g2 = [row[:] for row in grid]
        for r, c in path:
            g2[r][c] = 2
        # Print with symbols
        for r in range(len(g2)):
            line = ""
            for c in range(len(g2[0])):
                if (r, c) == start: line += "S "
                elif (r, c) == goal: line += "G "
                elif g2[r][c] == 1: line += "# "
                elif g2[r][c] == 2: line += ". "
                else: line += "_ "
            print(line)
