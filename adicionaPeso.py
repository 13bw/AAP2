from random import randint

def adicionar_pesos_aleatorios(grafo):
    for vertice, vizinhos in grafo.items():
        for i, vizinho in enumerate(vizinhos):
            if isinstance(vizinho, tuple):
                grafo[vertice][i] = (vizinho[0], randint(1, 10))
            else:
                peso = randint(1, 10)
                grafo[vertice][i] = (vizinho, peso)
                grafo[vizinho].append((vertice, peso))
    return grafo
