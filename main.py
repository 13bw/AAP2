import heapq
import os
from random import randint
from time import sleep

lista_vertices = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z']


def limpaTela():
    os.system('cls' if os.name == 'nt' else 'clear')


def ordenacao_topologica(grafo, pre_determinado):

    print("-=-=-=-=-=-=-=-=-=-=-=-=-=-=")
    print("-=- Ordenação Topológica -=-")
    print("-=-=-=-=-=-=-=-=-=-=-=-=-=-=")
    print(" ")
    print("Esse algoritimo utiliza dos conceitos de Ordenação Topológica")
    print("pra armazenar a ordem topológica dos vértices de um grafo.")
    print("")
    input("Prescione ENTER para continuar")
    limpaTela()

    if (pre_determinado is True):

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




def grafo_pre_determinado(): 
    grafo = {
        '1': ['2', '3', '4', '5', '6'], 
        '2': ['3', '4', '7'], 
        '3': ['4', '5', '6', '8'], 
        '4': ['5', '7'], 
        '5': ['6', '8'], 
        '6': ['7', '8'], 
        '7': ['8'], 
        '8': [] 
    }

    return grafo

def gerar_grafo_hamiltoniano():  
    grafo = {
        '1': ['2', '3', '4', '5', '6', '7', '8'], 
        '2': ['1', '3', '4', '5', '6', '7', '8'], 
        '3': ['1', '2', '4', '5', '6', '7', '8'], 
        '4': ['1', '2', '3', '5', '6', '7', '8'], 
        '3': ['1', '2', '3', '4', '6', '7', '8'], 
        '6': ['1', '2', '3', '4', '5', '7', '8'], 
        '7': ['1', '2', '3', '4', '5', '6', '8'], 
        '8': ['1', '2', '3', '4', '5', '6', '7'], 
    }

    return grafo

def busca_por_profundidade(grafo, lista_vertices):

    print("=========================================")
    print("-=- Algoritmo de Busca por Profundidade -=-")
    print("=========================================\n")
    
    descricao = (
        "O Algoritmo de Busca por Profundidade é uma técnica fundamental para explorar grafos. "
        "Este algoritmo realiza a busca a partir de um vértice selecionado pelo usuário, "
        "explorando o máximo possível de vértices em uma ramificação antes de retroceder. "
        "Essa abordagem é especialmente útil para detectar ciclos em grafos e para encontrar "
        "soluções em problemas de percurso. Após a execução do algoritmo, será possível visualizar "
        "o resultado da busca por profundidade no grafo, partindo do vértice informado.\n"
    )
    
    print(descricao)
    input("Pressione ENTER para continuar")

    origem = input("Informe o vértice de origem: ").upper()
    destino = input("Informe o vértice de destino: ").upper()

    if origem not in grafo or destino not in grafo:
        print("Vértice inválido!")
        return

    caminho = []
    visitados = set()

    def dfs(v, destino):
        caminho.append(v)
        visitados.add(v)

        if v == destino:
            return True

        for vizinho in grafo[v]:
            if isinstance(vizinho, tuple):
                vizinho = vizinho[0]
            if vizinho not in visitados:
                if dfs(vizinho, destino):
                    return True

        caminho.pop()
        return False

    if dfs(origem, destino):
        print("Caminho encontrado:", ' -> '.join(caminho))
    else:
        print("Não há caminho entre os vértices.")

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





def algoritmo_dijkstra(grafo, inicio):
    grafo = adicionar_pesos_aleatorios(grafo)
    distancias = {vertice: float('infinity') for vertice in grafo}
    distancias[inicio] = 0
    fila_prioridade = [(0, inicio)]
    
    while fila_prioridade:
        distancia_atual, vertice_atual = heapq.heappop(fila_prioridade)
        
        if distancia_atual > distancias[vertice_atual]:
            continue
        
        for vizinho, peso in grafo[vertice_atual]:
            distancia = distancia_atual + peso
            
            if distancia < distancias[vizinho]:
                distancias[vizinho] = distancia
                heapq.heappush(fila_prioridade, (distancia, vizinho))
    
    for vertice, distancia in distancias.items():
       print(f"Distância até {vertice}: {distancia}")
    
    return distancias



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
