def grafo_hamiltoniano(graph):
    def is_cycle(path):
        return len(path) == len(graph) + 1 and path[-1] == path[0]

    def hamiltonian_util(v, path):
        if v not in graph:
            return False
        if is_cycle(path):
            return True
        for neighbor in graph[v]:
            if neighbor not in path:
                path.append(neighbor)
                if hamiltonian_util(neighbor, path):
                    return True
                path.pop()
        return False

    for vertex in graph:
        path = [vertex]
        if hamiltonian_util(vertex, path):
            return True
    return False

