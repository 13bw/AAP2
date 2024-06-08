import heapq
from adicionaPeso import adicionar_pesos_aleatorios

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