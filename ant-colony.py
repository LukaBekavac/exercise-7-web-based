import random

import numpy as np

from environment import Environment
from ant import Ant

# Class representing the ant colony
"""
    ant_population: the number of ants in the ant colony
    iterations: the number of iterations 
    alpha: a parameter controlling the influence of the amount of pheromone during ants' path selection process
    beta: a parameter controlling the influence of the distance to the next node during ants' path selection process
    rho: pheromone evaporation rate
"""


class AntColony:
    def __init__(self, ant_population: int, iterations: int, alpha: float, beta: float, rho: float):
        self.ant_population = ant_population
        self.iterations = iterations
        self.alpha = alpha
        self.beta = beta
        self.rho = rho

        # Initialize the environment of the ant colony
        self.environment = Environment(self.rho, self.ant_population)

        # Initalize the list of ants of the ant colony
        self.ants = []

        # Get Pheromone map
        self.pheromone_map = self.environment.get_pheromone_map()

        # Initialize the sum of pheromone of zero values
        self.better_solution = np.zeros((self.environment.get_locations_count(), self.environment.get_locations_count()))

    # Solve the ant colony optimization problem
    def solve(self):
        iter = 0
        shortest_distance = np.inf
        assigned_locations = set()

        while iter < self.iterations:

            print("=> iteration: ", iter + 1)

            for ant_number in range(self.ant_population):

                print("=> ant: ", ant_number + 1)

                # Randomly select an initial location for the ant that has not already been assigned to another ant
                unassigned_locations = set(range(self.environment.get_locations_count())) - assigned_locations
                if len(unassigned_locations) > 0:
                    initial_location = random.choice(list(unassigned_locations))
                    assigned_locations.add(initial_location)
                else:
                    initial_location = random.randint(0, self.environment.get_locations_count() - 1)

                print("=> initial_location: ", initial_location)

                # Initialize an ant
                ant = Ant(self.alpha, self.beta, initial_location)
                ant.join(self.environment)

                # Make the ant move and traverse the environment
                ant.run()

                # Get the ant's traveled distance, the sum of traveled distance and visited locations
                ant_travel_distance, sum_distance = ant.get_travelled_distance()
                ant_visited_locations = ant.get_visited_locations()

                print("traveled distance: ", ant_travel_distance)
                print("total distance sum: ", sum_distance)
                print("visited locations: ", ant_visited_locations)
                print("final location: ", ant_visited_locations[-1])

                # Update the shortest distance and the solution if the ant's traveled distance is less than the shortest distance
                if sum_distance < shortest_distance:
                    shortest_distance = sum_distance
                    solution = ant_visited_locations

                for loc_idx in range(len(ant_visited_locations) - 1):
                    # Update pheromone level for each visited location
                    location = ant_visited_locations[loc_idx]
                    next_location = ant_visited_locations[loc_idx + 1]
                    self.better_solution[location][next_location] += 1 / ant_travel_distance[loc_idx + 1]
                    self.better_solution[next_location][location] += 1 / ant_travel_distance[loc_idx + 1]

                # Add the ant to the ant colony
                self.ants.append(ant)

            self.environment.update_pheromone_map(self.better_solution)

            iter += 1

        return solution, shortest_distance

def main():
    # Intialize the ant colony
    ant_colony = AntColony(ant_population=48, iterations=100, alpha=0, beta=0, rho=0.5)

    # Solve the ant colony optimization problem
    solution, distance = ant_colony.solve()
    print("Solution: ", solution)
    print("Distance: ", distance)


if __name__ == '__main__':
    main()
