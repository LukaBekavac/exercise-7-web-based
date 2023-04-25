import math

import numpy as np
import tsplib95
import networkx as nx
import matplotlib.pyplot as plt

# Class representing the environment of the ant colony
"""
    rho: pheromone evaporation rate
"""


class Environment:
    def __init__(self, rho, ant_population):

        self.rho = rho
        self.ant_population = ant_population

        # Initialize the environment topology

        problem = tsplib95.load('att48-specs/att48.tsp')
        self.problem = problem
        self.node = list(problem.get_nodes())
        self.edge = list(problem.get_edges())
        self.node_coords = problem.node_coords
        self.number_nodes = len(problem.node_coords)
        self.number_edges = len(list(problem.get_edges()))
        self.possible_locations = list(range(self.number_nodes))
        # print(self.possible_locations)

        self.pheromone_map = self.initialize_pheromone_map()
        self.draw = self.draw_graph()
        # Print the number of nodes in the TSP instance
        print("number of nodes: ")
        print(self.problem.dimension)

    # Intialize the pheromone trails in the environment
    def initialize_pheromone_map(self):

        distance_map = np.zeros((self.number_nodes, self.number_nodes))
        for i in range(self.number_nodes):
            for j in range(i, self.number_nodes):
                dist = self.distances(self.node_coords[i + 1], self.node_coords[j + 1])
                distance_map[i][j] = dist
                distance_map[j][i] = dist

        pheromone_map = np.zeros((self.number_nodes, self.number_nodes))
        for i in range(self.number_nodes):
            for j in range(self.number_nodes):
                if i != j:
                    pheromone_map[i][j] = self.ant_population / distance_map[i][j]

        np.fill_diagonal(pheromone_map, 0)

        self.distance_map = distance_map
        self.pheromone_map = pheromone_map

        return pheromone_map

    def distances(self, coord1, coord2):
        x1, y1 = coord1
        x2, y2 = coord2

        # Calculate the Euclidean distance between the points
        xd = x1 - x2
        yd = y1 - y2
        rij = np.sqrt((xd ** 2 + yd ** 2) / 10.0)
        tij = np.round(rij)

        # Round up if necessary to get the "pseudo-Euclidean" distance
        if tij < rij:
            dij = tij + 1
        else:
            dij = tij

        return dij

    # Added a Visualization of the graph for better understanding
    def draw_graph(self):
        G = nx.Graph()
        node_positions = {}
        for node in self.node:
            node_positions[node] = self.node_coords[node]
            G.add_node(node)
        for edge in self.edge:
            G.add_edge(*edge)

        plt.figure(figsize=(8, 8))
        nx.draw_networkx(G, pos=node_positions, node_size=20, font_size=8, with_labels=True)
        plt.show()

    # Update the pheromone trails in the environment
    def update_pheromone_map(self, better_solution):
        # the loss of pheromone based on rho
        self.pheromone_map = self.pheromone_map * self.rho

        # adding pheromone if a better path was found
        self.pheromone_map = self.pheromone_map + better_solution

    # Get the pheromone trails in the environment
    def get_pheromone_map(self):
        return self.pheromone_map

    # Get the environment topology
    def get_possible_locations(self):
        return self.possible_locations

    def get_locations_count(self):
        return self.number_nodes

    #   # Get the current location
    def get_location(self, location):
        # print("location:", location)
        return self.node_coords[location+1] # because numpy arrays start with 0
