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
