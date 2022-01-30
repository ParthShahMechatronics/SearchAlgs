import collections
import numpy as np
import heapq

def bfs(start,exit, maze):
    sCoord = ((start[0],start[1]))
    # define exit status
    maze[exit[0]][exit[1]] = 2
    open_queue = collections.deque([[sCoord]])
    visited = set([sCoord])
    while open_queue:
        path = open_queue.popleft()
        y, x = path[-1]
        # terminate when goal is found
        if maze[y][x] == 2:
            # reset exit status
            maze[exit[0]][exit[1]] = 0
            return path, len(path), len(visited)
        # expand node and add neighboring nodes to fringe queue
        for x2, y2 in ((x-1, y), (x, y+1), (x+1, y), (x, y-1)):
            if 0 <= x2 < len(maze[0]) and 0 <= y2 < len(maze) and maze[y2][x2] != 1 and (y2, x2) not in visited:
                open_queue.append(path + [(y2, x2)])
                visited.add((y2 , x2))
    return [], 0, len(visited)

def recursive_dfs(maze, open_queue, visited, route, visited_len):
    path = open_queue.popleft()
    y, x = path[-1]
    # terminate when goal is found
    if maze[y][x] == 2:
        route = path
        visited_len = len(visited)
        return route, visited_len
    for x2, y2 in ((x-1, y), (x, y+1), (x+1, y), (x, y-1)):
        if 0 <= x2 < len(maze[0]) and 0 <= y2 < len(maze) and maze[y2][x2] != 1 and (y2, x2) not in visited:
            open_queue.append(path + [(y2, x2)])
            visited.add((y2 , x2))
            route, visited_len = recursive_dfs(maze, open_queue, visited, route, visited_len)
    return route, visited_len

def dfs(start,exit,maze):
    sCoord = ((start[0],start[1]))
    # define exit status
    maze[exit[0]][exit[1]] = 2
    open_queue = collections.deque([[sCoord]])
    visited = set([sCoord])
    route = []
    visited_len = 0
    route, visited_len = recursive_dfs(maze, open_queue, visited, route, visited_len)
    # reset exit status
    maze[exit[0]][exit[1]] = 0
    return route, len(route), visited_len 

def heuristic(y2,x2,exit):
    manh_dist = abs(exit[0]-y2) + abs(exit[1]-x2)
    return manh_dist

def astar(start,exit,maze):
    sCoord = (start[0],start[1])
    open_queue = []
    path = [sCoord]
    # define exit status
    maze[exit[0]][exit[1]] = 2
    gval = {}
    gval[sCoord] = 0, [path]
    hval = {sCoord:heuristic(start[0],start[1], exit)}
    fval = {sCoord:gval[sCoord][0] +hval[sCoord]}
    heapq.heappush(open_queue, (fval[sCoord], hval[sCoord], gval[sCoord], sCoord))
    visited = set([(sCoord)])

    while open_queue:
        curr_node = heapq.heappop(open_queue)[3]
        y, x = curr_node[0], curr_node[1]
        path = gval[curr_node][1]

        # terminate when goal is found
        if maze[y][x] == 2:
            # reset exit status
            maze[exit[0]][exit[1]] = 0
            return path, len(path), len(visited) 

        for x2, y2 in ((x-1, y), (x, y+1), (x+1, y), (x, y-1)):
            if 0 <= x2 < len(maze[0]) and 0 <= y2 < len(maze) and maze[y2][x2] != 1 and (y2,x2) not in visited:
                # find values of node
                node_gval = gval[curr_node][0] + 1
                node_hval = heuristic(y2,x2, exit)
                node_fval = node_hval + node_gval

                if ((y2,x2) in gval and gval[curr_node][0] + 1 < gval[(y2,x2)][0]) or ((y2,x2) not in gval):
                    # add values to table
                    gval[(y2,x2)] = node_gval, path + [(y2,x2)]
                    hval[(y2,x2)] = node_hval
                    fval[(y2,x2)] = node_fval
                    # push to queue based on priority
                    heapq.heappush(open_queue, (node_fval, node_hval, node_gval, (y2, x2)))
            visited.add((y2 , x2))  
    return [], 0, len(visited)


if __name__ == "__main__":
    maze = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0]]

    # Start position [y, x]
    start = [11,2]

    # Exit position [y, x]
    E1 = [19,23]
    E2 = [21,2]

    # bfs
    bfs_patha, bfs_costa, bfs_checkeda = bfs(start,E1,maze)
    bfs_pathb, bfs_costb, bfs_checkedb = bfs(start,E2,maze)
    bfs_pathc, bfs_costc, bfs_checkedc = bfs([0,0],[24,24],maze)

    #dfs
    dfs_patha, dfs_costa, dfs_checkeda = dfs(start,E1,maze)
    dfs_pathb, dfs_costb, dfs_checkedb = dfs(start,E2,maze)
    dfs_pathc, dfs_costc, dfs_checkedc = dfs([0,0],[24,24],maze)

    # A star
    astar_patha, astar_costa, astar_checkeda = astar(start,E1,maze)
    astar_pathb, astar_costb, astar_checkedb = astar(start,E2,maze)
    astar_pathc, astar_costc, astar_checkedc = astar([0,0],[24,24],maze)

    # Output of complete path, path cost and number of nodes explored
    print("ECE457A Assignment 2 Question 2 output \n")

    # bfs output
    print("BFS solution path from S to E1:\n", bfs_patha)
    print("cost of solution =", bfs_costa)
    print("squares checked =", bfs_checkeda)
    print("BFS solution path from S to E2:\n", bfs_pathb)
    print("cost of solution =", bfs_costb)
    print("squares checked =", bfs_checkedb)
    print("BFS solution path from (0,0) to (24,24):\n", bfs_pathc)
    print("cost of solution =", bfs_costc)
    print("squares checked =", bfs_checkedc)

    # dfs output
    print("DFS solution path from S to E1:\n", dfs_patha)
    print("cost of solution =", dfs_costa)
    print("squares checked =", dfs_checkeda)
    print("DFS solution path from S to E2:\n", dfs_pathb)
    print("cost of solution =", dfs_costb)
    print("squares checked =", dfs_checkedb)
    print("DFS solution path from (0,0) to (24,24):\n", dfs_pathc)
    print("cost of solution =", dfs_costc)
    print("squares checked =", dfs_checkedc)

    # A star output
    print("A Star solution path from S to E1:\n", astar_patha)
    print("cost of solution =", astar_costa)
    print("squares checked =", astar_checkeda)
    print("A Star solution path from S to E2:\n", astar_pathb)
    print("cost of solution =", astar_costb)
    print("squares checked =", astar_checkedb)
    print("A Star solution path from (0,0) to (24,24):\n", astar_pathc)
    print("cost of solution =", astar_costc)
    print("squares checked =", astar_checkedc)

