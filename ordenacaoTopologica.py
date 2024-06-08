from limpaTela import limpaTela
from exibeGrafo import exibir_grafo
import os
from time import sleep

def ordenacao_topologica(grafo, pre_determinado):

    novo_grafo = {
        'A': ['B', 'C'],
        'B': ['D', 'E'],
        'C': ['F'],
        'D': [],
        'E': ['F'],
        'F': []
    }

    print("-=-=-=-=-=-=-=-=-=-=-=-=-=-=")
    print("-=- Ordenação Topológica -=-")
    print("-=-=-=-=-=-=-=-=-=-=-=-=-=-=")
    print(" ")
    print("Esse algoritimo utiliza dos conceitos de Ordenação Topológica")
    print("pra armazenar a ordem topológica dos vértices de um grafo.")
    print("")
    input("Prescione ENTER para continuar")
    limpaTela()

    if (pre_determinado == True):

        while True:
            print("-=-=-=-=-=-=-=-=-=")
            print(" -=- ATENÇÃO! -=- ")
            print("-=-=-=-=-=-=-=-=-=")
            print(" ")
            print("Seu Grafo pré determinado possui um ciclo!")
            print("A Ordenação Topológica não funciona em grafos com ciclos.")
            print(" ")
            break

    limpaTela()                  
    # exibir_grafo(grafo)
    print(" ")

    visitados = set()   # Conjunto que armazenará os vértices visitados
    pilha = []  # Pilha para armazenar a ordenação topológica

    for vertice in grafo:   # Percorre todos os vértices do grafo
        if vertice not in visitados:    # Realiza a busca em profundidade para os vértices não visitados
            stack = [(vertice, iter(grafo[vertice]))]

            while stack:
                v, vizinhos_iter = stack[-1]
                try:
                    vizinho = next(vizinhos_iter)
                except StopIteration:
                    stack.pop()
                    pilha.append(v)
                    continue

                if vizinho not in visitados:
                    visitados.add(vizinho)
                    stack.append((vizinho, iter(grafo[vizinho])))

    if len(pilha) != len(grafo):    # Verifica se a ordenação topológica foi bem-sucedida (grafo sem ciclos)
        print("Não é possível encontrar uma ordenação topológica. O grafo contém um ciclo.")
        return None

    # Retorna a ordenação topológica encontrada, invertendo a ordem da pilha
    return pilha[::-1]
