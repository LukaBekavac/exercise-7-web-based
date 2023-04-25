# Class representing an artificial ant of the ant colony
"""
    alpha: a parameter controlling the influence of the amount of pheromone during ants' path selection process
    beta: a parameter controlling the influence of the distance to the next node during ants' path selection process
"""
import math

import numpy as np


class Ant():
    def __init__(self, alpha: float, beta: float, initial_location):
        self.alpha = alpha
        self.beta = beta
        self.current_location = initial_location
        self.travelled_distance = [0]
        self.visited_locations = [initial_location]
        self.keep_running = True

    # The ant runs to visit all the possible locations of the environment
    def run(self):
        if self.keep_running:
            self.possible_locations = self.environment.get_possible_locations().copy()
        self.keep_running = False

        if self.current_location in self.possible_locations:
            del self.possible_locations[self.possible_locations.index(self.current_location)]

        if self.possible_locations:
            self.select_path()
            self.run()
        else:
            # Check that all locations have been visited
            assert len(self.visited_locations) == self.environment.get_locations_count()
            assert len(self.travelled_distance) == self.environment.get_locations_count()

            print("=> finished iteration")

    # Select the next path based on the random proportional rule of the ACO algorithm
    def select_path(self):
        self.pheromone_map = self.environment.get_pheromone_map()

        prob_list = []
        total_prob = 0

        for possible_location in self.possible_locations:
            current_to_j = (self.pheromone_map[self.current_location][possible_location] ** self.alpha) * (
                    self.environment.distance_map[self.current_location][possible_location] ** self.beta)
            sum_current_to_l = sum([(self.pheromone_map[self.current_location][l] ** self.alpha) * (
                    self.environment.distance_map[self.current_location][l] ** self.beta) for l in
                                    self.possible_locations])
            prob = current_to_j / sum_current_to_l
            prob_list.append(prob)
            total_prob += prob

        prob_list = [p / total_prob for p in prob_list]
        future_location = np.random.choice(self.possible_locations, p=prob_list)

        self.travelled_distance.append(self.get_distance(self.current_location, future_location))
        self.current_location = future_location
        self.visited_locations.append(self.current_location)
        self.possible_locations.remove(self.current_location)

    # Position an ant in an environment
    def join(self, environment):
        self.environment = environment

    def get_distance(self, location1, location2):
        x1, y1 = self.environment.get_location(location1)
        x2, y2 = self.environment.get_location(location2)

        xd = x1 - x2
        yd = y1 - y2
        rij = math.sqrt((xd * xd + yd * yd) / 10.0)
        tij = round(rij)

        if tij < rij:
            dij = tij + 1
        else:
            dij = tij

        return dij

    def get_visited_locations(self):
        return self.visited_locations

    def get_travelled_distance(self):
        return self.travelled_distance, sum(self.travelled_distance)
