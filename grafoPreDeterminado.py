from exibeGrafo import exibir_grafo
import os

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

    # exibir_grafo(grafo)

    return grafo