def busca_por_largura(grafo, lista_vertices):

    print("=========================================")
    print("-=- Algoritmo de Busca por Largura -=-")
    print("=========================================\n")
    
    descricao = (
        "O Algoritmo de Busca por Largura é uma técnica utilizada para encontrar o caminho mais curto entre "
        "dois vértices de um grafo. Esta abordagem é essencial em diversas aplicações, como em sistemas de "
        "navegação e em redes de computadores. O algoritmo explora todos os vértices vizinhos de um vértice inicial "
        "antes de avançar para os vértices mais distantes. Isso permite encontrar o caminho mais curto "
        "considerando o número mínimo de arestas. Após a execução do algoritmo, será possível determinar o caminho "
        "ótimo entre os vértices de origem e destino informados.\n"
    )
    
    print(descricao)
    input("Pressione ENTER para continuar")

    origem = input("Informe o vértice de origem: ").upper()
    destino = input("Informe o vértice de destino: ").upper()

    if origem not in grafo or destino not in grafo:
        print("Vértice inválido!")
        return None

    queue = [(origem, [origem])]
    visitados = set()

    while queue:
        (vertice, caminho) = queue.pop(0)
        visitados.add(vertice)

        for vizinho in grafo[vertice]:
            if isinstance(vizinho, tuple):
                vizinho = vizinho[0]
            if vizinho not in visitados:
                if vizinho == destino:
                    return caminho + [vizinho]
                else:
                    queue.append((vizinho, caminho + [vizinho]))

    return None
