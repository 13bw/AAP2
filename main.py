from grafoPreDeterminado import grafo_pre_determinado
from grafoHamiltoniano import gerar_grafo_hamiltoniano
from exibeGrafo import exibir_grafo
from hamiltoniano import grafo_hamiltoniano
from limpaTela import limpaTela
from adicionaPeso import adicionar_pesos_aleatorios
from buscaLargura import busca_por_largura
from buscaProfundidade import busca_por_profundidade
from arvoreGeradoraMinima import arvore_geradora_minima
from algoritmosGulosos import busca_gulosa_kruskal
from ordenacaoTopologica import ordenacao_topologica
from algoritmoMenorcaminhoDijkstra import algoritmo_dijkstra
from algoritmoparaColoracao import colorir_grafo

import os
from time import sleep

lista_vertices = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z']

def main():
    while True:
        limpaTela()

        while True:
            limpaTela()
            print("Selecione seu algoritmo:")
            print("="*100)
            print(" ")
            print("[1] Busca por profundidade")
            print("[2] Hamiltoniano")
            print("[3] Busca por Largura")
            print("[4] Árvore geradora mínima")
            print("[5] Algoritmos gulosos")
            print("[6] Ordenação topológica")
            print("[7] Algoritmo para coloração")
            print("[8] Algoritmo menor caminho (Dijkstra)")
            print(" ")
            try:
                tipo_algoritmo = int(input("Escolha uma opção: "))
                if tipo_algoritmo < 1 or tipo_algoritmo > 8:
                    raise ValueError
            except ValueError:
                print("Número inválido!")
                sleep(1)
                continue
            break

        if tipo_algoritmo == 2:
            grafo = gerar_grafo_hamiltoniano()
            caminho_hamiltoniano = grafo_hamiltoniano(grafo)

            if caminho_hamiltoniano:
                print("Existe um caminho hamiltoniano no grafo.")
            else:
                print("Não existe um caminho hamiltoniano no grafo.")
        else:
            grafo = grafo_pre_determinado()
            if tipo_algoritmo == 1:
                busca_por_profundidade(grafo, lista_vertices)
            elif tipo_algoritmo == 3:
                caminho_menor = busca_por_largura(grafo, lista_vertices)
                if caminho_menor:
                    # exibir_grafo(grafo)
                    print(" ")
                    print("Caminho mais curto entre os vértices selecionados:", ' -> '.join(caminho_menor))
                    print(" ")
                else:
                    print("Não há caminho entre os vértices.")
            elif tipo_algoritmo == 4:
                adicionar_pesos_aleatorios(grafo)
                prim = arvore_geradora_minima(grafo, lista_vertices)
                print("Árvore Geradora Mínima (Prim):", prim)
            elif tipo_algoritmo == 5:
                adicionar_pesos_aleatorios(grafo)
                kruskal = busca_gulosa_kruskal(grafo)
                print("Busca Gulosa - Kruskal")
                print("Árvore Mínima de Abrangência:")
                for aresta in kruskal:
                    print(aresta)
            elif tipo_algoritmo == 6:
                pre_determinado = True if grafo == grafo_pre_determinado() else False
                topologica = ordenacao_topologica(grafo, pre_determinado)
                if topologica is not None:
                    print("Ordenação Topológica:")
                    print(topologica)
            elif tipo_algoritmo == 7:
                colorir_grafo(grafo)
            elif tipo_algoritmo == 8:
                src = str(input("Informe o vértice de origem (índice): "))
                algoritmo_dijkstra(grafo, src)

        input("Pressione ENTER para voltar ao menu principal")

if __name__ == "__main__":
    main()
