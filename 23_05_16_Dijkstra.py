import sys
import tkinter as tk
import networkx as nx
import matplotlib.pyplot as plt
from tkinter import messagebox

def dijkstra(graph, start):
    # Inicializar las estructuras de datos
    distances = {node: sys.maxsize for node in graph}
    distances[start] = 0
    visited = set()

    while len(visited) < len(graph):
        # Encontrar el nodo con la distancia mínima no visitado
        min_distance = sys.maxsize
        min_node = None
        for node in graph:
            if node not in visited and distances[node] < min_distance:
                min_distance = distances[node]
                min_node = node
        
        # Marcar el nodo actual como visitado
        visited.add(min_node)

        # Actualizar las distancias de los nodos adyacentes
        for neighbor, weight in graph[min_node].items():
            distance = distances[min_node] + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance

    return distances

def procesar_grafo():
    # Obtener los datos ingresados por el usuario
    datos = entrada.get()

    # Dividir los datos en conexiones individuales
    conexiones = datos.split(',')

    # Crear un grafo vacío
    graph = {}

    # Procesar cada conexión
    for conexion in conexiones:
        # Dividir la conexión en nodo1-nodo2 y peso
        nodos, peso = conexion.split(':')

        # Extraer los nodos individuales
        nodo1, nodo2 = nodos.split('-')

        # Agregar la conexión al grafo
        if nodo1 not in graph:
            graph[nodo1] = {}
        if nodo2 not in graph:
            graph[nodo2] = {}

        graph[nodo1][nodo2] = int(peso)
        graph[nodo2][nodo1] = int(peso)

    # Ejecutar el algoritmo de Dijkstra
    start_node = start_entry.get()
    end_node = end_entry.get()

    shortest_distances = dijkstra(graph, start_node)

    # Verificar si el nodo de destino es alcanzable desde el nodo de inicio
    if shortest_distances[end_node] == sys.maxsize:
        messagebox.showinfo("Ruta no encontrada", "No hay ruta posible desde el nodo de inicio al nodo de destino.")
        return

    # Crear un objeto de grafo dirigido utilizando NetworkX
    G = nx.DiGraph()

    # Agregar los nodos al grafo
    for node in graph:
        G.add_node(node)

    # Agregar las aristas al grafo con las distancias como etiquetas
    for node, neighbors in graph.items():
        for neighbor, weight in neighbors.items():
            G.add_edge(node, neighbor, weight=weight)

    # Crear una lista con las rutas más cortas encontradas por Dijkstra
    shortest_paths = []
    for node, distance in shortest_distances.items():
        if node != start_node:
            path = nx.shortest_path(G, start_node, node, weight='weight')
            shortest_paths.append((path, distance))

    # Crear un diseño para el grafo
    pos = nx.spring_layout(G)

    # Dibujar los nodos del grafo
    nx.draw_networkx_nodes(G, pos)

    # Dibujar las aristas del grafo con las etiquetas de distancia
    nx.draw_networkx_edges(G, pos)
    
    # Dibujar las aristas del grafo con las etiquetas de distancia
    nx.draw_networkx_edges(G, pos)
    edge_labels = {(u, v): d['weight'] for u, v, d in G.edges(data=True)}
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

    # Resaltar la ruta más corta desde el nodo de inicio al nodo de destino
    shortest_path = nx.shortest_path(G, start_node, end_node, weight='weight')
    edges = [(shortest_path[i], shortest_path[i+1]) for i in range(len(shortest_path)-1)]
    nx.draw_networkx_edges(G, pos, edgelist=edges, edge_color='r', width=2)

    # Mostrar el grafo
    plt.axis('off')
    plt.show()

# Crear la ventana
ventana = tk.Tk()

# Configurar la ventana
ventana.title("Algoritmo de Dijkstra")
ventana.geometry("350x275")

# Texto de explicación para el usuario
graph_label = tk.Label(ventana, text="Para ingresar el grafo, utiliza la siguiente sintaxis:\nA-B:#,A-C:#,B-C:#,B-D:#,C-D:#\nPor ejemplo, si hay una conexión entre A y B con peso 5,\nse escribiría 'A-B:5'.")
graph_label.pack(pady=5)

# Etiqueta para el campo de entrada del grafo
graph_label = tk.Label(ventana, text="Ingrese el grafo:")
graph_label.pack(pady=1)

# Campo de entrada del grafo
entrada = tk.Entry(ventana, width=30)
entrada.pack(pady=2)

# Etiqueta para el campo de entrada del nodo de inicio
start_label = tk.Label(ventana, text="Nodo de inicio:")
start_label.pack(pady=1)

# Campo de entrada del nodo de inicio
start_entry = tk.Entry(ventana, width=10)
start_entry.pack(pady=2)

# Etiqueta para el campo de entrada del nodo de destino
end_label = tk.Label(ventana, text="Nodo de destino:")
end_label.pack(pady=1)

# Campo de entrada del nodo de destino
end_entry = tk.Entry(ventana, width=10)
end_entry.pack(pady=2)

# Botón para procesar el grafo
procesar_boton = tk.Button(ventana, text="Procesar", command=procesar_grafo)
procesar_boton.pack(pady=5)

# Ejecutar el bucle de eventos de la ventana
ventana.mainloop()