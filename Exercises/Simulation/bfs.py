from edmunds import edmunds105


def is_valid_coordinate(coord, graph):
    return coord[0] >= 0 and coord[0] < len(graph[0]) and coord[1] >= 0 and coord[1] < len(graph)

def bfs(graph, start, goal):
    bfs_paths = [[0 for _ in range(len(graph[0]))] for _ in range(len(graph))]
    q = []
    
    visited = []
    visited.append(start)

    q.append(start)

    possible_directions = [[0, 1], [0, -1], [-1, 0], [1, 0]]

    while len(q) != 0:
        v = q.pop(0)
        
        if (v == goal):
            return bfs_paths
                
        for offset in possible_directions:
            adj = [v[0] + offset[0], v[1] + offset[1]]
            if is_valid_coordinate(adj, graph) and graph[adj[1]][adj[0]] == 0 and adj not in visited:
                visited.append(adj)
                bfs_paths[adj[1]][adj[0]] = v
                q.append(adj)

    return 0  
 
edmunds105.reverse()   
# print(bfs(edmunds105, [0,0], [2,2]))

def get_path(paths, start, goal):
    if paths == 0: 
        return -1

    path = []
    curr = goal
    
    while curr != start: 
        path.append(curr)
        curr = paths[curr[1]][curr[0]]    
        
    return path[::-1]
    
print(get_path(bfs(edmunds105, [26,0], [19, 12]), [26,0], [19,12]))