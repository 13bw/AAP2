def arvore_geradora_minima(grafo, lista_vertices):

    print("=========================================")
    print("     -=- Árvores Geradoras Mínimas -=-   ")
    print("          -=- Algoritmo de Prim -=-      ")
    print("=========================================\n")
    
    descricao = (
        "O Algoritmo de Prim é um método utilizado para encontrar Árvores Geradoras Mínimas em um grafo. "
        "Este algoritmo é essencial em diversos campos da ciência da computação e engenharia, "
        "pois permite a otimização de redes de conexão, como rodovias, redes elétricas e comunicação de dados. "
        "Para aplicar este algoritmo, é necessário atribuir pesos às arestas do grafo, que representam os custos "
        "ou distâncias entre os vértices. Neste exemplo, serão adicionados pesos aleatórios a cada aresta do grafo.\n"
    )
    
    print(descricao)
    input("Pressione ENTER para continuar")

    vertices = set(grafo)
    edges = [(weight, u, v) for u in grafo for v, weight in grafo[u]]

    parent = {v: v for v in vertices}
    rank = {v: 0 for v in vertices}

    def find(v):
        if parent[v] != v:
            parent[v] = find(parent[v])
        return parent[v]

    def union(u, v):
        root_u = find(u)
        root_v = find(v)
        if root_u != root_v:
            if rank[root_u] > rank[root_v]:
                parent[root_v] = root_u
            else:
                parent[root_u] = root_v
                if rank[root_u] == rank[root_v]:
                    rank[root_v] += 1

    edges.sort()
    mst = set()

    for weight, u, v in edges:
        if find(u) != find(v):
            union(u, v)
            mst.add((u, v, weight))

    return mst
