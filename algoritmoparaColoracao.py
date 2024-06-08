def colorir_grafo(grafo):
    cores_disponiveis = ['Vermelho', 'Verde', 'Azul', 'Amarelo', 'Laranja', 'Roxo', 'Marrom', 'Rosa']

    coloracao = {}

    for vertice in reversed(list(grafo.keys())):
        cores_proibidas = {coloracao.get(adj) for adj in grafo[vertice] if adj in coloracao}
        for cor in cores_disponiveis:
            if cor not in cores_proibidas:
                coloracao[vertice] = cor
                # print(coloracao)
                break
    for vertice, cor in coloracao.items():
        print(f"Vértice {vertice}: {cor}")
    return coloracao