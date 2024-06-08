from exibeGrafo import exibir_grafo_com_pesos
from limpaTela import limpaTela
from time import sleep

def encontra_componente(componentes, vertice):
    for componente, vertices in componentes.items():
        if vertice in vertices:
            return componente

def busca_gulosa_kruskal(grafo):
    print("===================================")
    print("       -=- Algoritmos Gulosos -=-  ")
    print("     -=- Algoritmo de Kruskal -=-  ")
    print("===================================\n")
    
    descricao = (
        "O Algoritmo de Kruskal é um exemplo clássico de algoritmos gulosos aplicado em grafos. "
        "Este algoritmo é utilizado para encontrar uma Árvore Geradora Mínima, selecionando arestas com os menores pesos "
        "de forma a evitar ciclos e cobrir todos os vértices do grafo. "
        "Para a execução deste algoritmo, é fundamental atribuir pesos às arestas, representando custos ou distâncias. "
        "Neste contexto, serão atribuídos pesos aleatórios às arestas do grafo.\n"
    )
    
    print(descricao)
    input("Pressione ENTER para continuar")

    limpaTela()

    # exibir_grafo_com_pesos(grafo)
    print(" ")
    sleep(1)

    arvore = set()
    componentes = {vertice: {vertice} for vertice in grafo}

    arestas = sorted([(u, v, w) for u in grafo for v, w in grafo[u]], key=lambda x: x[2])

    for u, v, w in arestas:
        componente_u = encontra_componente(componentes, u)
        componente_v = encontra_componente(componentes, v)

        if componente_u != componente_v:
            arvore.add((u, v, w))
            componentes[componente_u].update(componentes[componente_v])
            del componentes[componente_v]

            if len(componentes) == 1:
                return arvore

    return arvore
