from pqueue import PQueue
import math
from copy import deepcopy


def distance_on_unit_sphere(lat1, long1, lat2, long2):
    # Convert latitude and longitude to
    # spherical coordinates in radians.
    degrees_to_radians = math.pi / 180.0

    # phi = 90 - latitude
    phi1 = (90.0 - lat1) * degrees_to_radians
    phi2 = (90.0 - lat2) * degrees_to_radians

    # theta = longitude
    theta1 = long1 * degrees_to_radians
    theta2 = long2 * degrees_to_radians

    # Compute spherical distance from spherical coordinates.

    # For two locations in spherical coordinates
    # (1, theta, phi) and (1, theta', phi')
    # cosine( arc length ) =
    # sin phi sin phi' cos(theta-theta') + cos phi cos phi'
    # distance = rho * arc length

    cos = (math.sin(phi1) * math.sin(phi2) * math.cos(theta1 - theta2) +
           math.cos(phi1) * math.cos(phi2))
    arc = math.acos(cos)

    # Remember to multiply arc by the radius of the earth
    # in your favorite set of units to get length.
    return arc


class Location(object):
    """
    A Location object represents an intersection (vertex) in the road network.
    """

    def __init__(self, locId, latitude, longitude):
        self.locId = locId
        self.longitude = longitude
        self.latitude = latitude

    def __str__(self):
        return f'Location: {self.locId}, long={self.longitude}, lat={self.latitude}'

    def __repr__(self):
        return f'Location({self.locId}, {self.longitude}, {self.latitude})'


class Road(object):
    """
    A Road object represents a connection between Locations (an edge) in the road network.
    """

    def __init__(self, startId, endId, speed, name):
        self.startId = startId
        self.endId = endId
        self.speed = speed
        self.name = name

    def __str__(self):
        return f'Road: start={self.startId}, end={self.endId}, {self.speed}, {self.name}'


class Node:
    def __init__(self, state: dict, parent: Location, action=None, g: float = 0.0, h: float = 0.0, speeding=None):
        self.state = state  # Dictionary with 'location' and 'speed
        self.parent = parent  # Parent Location
        self.action = action  # Connected roads with state as staring location
        self.g = g  # The path cost
        self.h = h  # The heuristic cost
        self.speeding = speeding  # Binary but initialized as Null

    def f(self):
        return self.g + self.h

    def __eq__(self, other_node):
        return self.state['loc'].locId == other_node.state['loc'].locId

    def __str__(self):
        # action={self.action}
        parent_loc_id = self.parent.state['loc'].locId if self.parent else None
        loc_id = [self.state['loc'].locId, self.state['speed_sessions']]
        return f"Node: state={loc_id}, speeding={self.speeding}, parent={parent_loc_id}, , g={self.g}, h={self.h}, f={self.f()}"




class RoadNetwork(object):
    """
    A RoadNetwork holds a graph of Locations and Roads.
    """

    def __init__(self):
        self.frontier = None
        self.a_star_path = None
        self.reached = None
        self.locations = dict()
        self.roads = dict()
        self.node_visited = 0

    def add_location(self, location: Location):
        """
        Add a new Location to the network.
        :param location:
        :return:
        """
        self.locations[location.locId] = location
        self.roads[location.locId] = []

    def add_road(self, road: Road):
        """
        Add a new Road to the network.  We assume its starting and ending locations
        already exist in the network.
        """
        self.roads[road.startId].append(road)

    def get_location_by_id(self, locId):
        """
        Get a Location by its identifier.
        """
        return self.locations[locId]

    def get_roads_connected_to(self, locId):
        """
        Get a list of all the roads connected to a specific location id.
        """
        return self.roads[locId]

    def get_all_road_speed(self, base_roads):
        speed_road_collection = list()
        for base_road in base_roads:
            speed_road = deepcopy(base_road)
            speed_road.speed *= 2
            speed_road_collection.append(speed_road)

        return (speed_road_collection + base_roads)

    def get_nodes_connected_to(self, base_node: Node, goal_node: Node):
        connected_nodes = []
        connected_roads = self.get_roads_connected_to(base_node.state['loc'].locId)
        roads = connected_roads

        if base_node.state['speed_sessions'] > 0:
            speedable_roads = self.get_all_road_speed(connected_roads)
            roads = speedable_roads
            half_len = len(roads)//2
            first_half_roads = roads[:half_len]


        for road in roads:
            # Only defining dest_node just to have a Node instance
            dest_node_loc = self.get_location_by_id(road.endId)
            speeding = False;

            if base_node.state['speed_sessions'] > 0 and (road in first_half_roads):
                dest_node_state = {"loc": dest_node_loc, "speed_sessions": base_node.state["speed_sessions"] - 1}
                speeding = True
                heuristic_dest_node = Node(dest_node_state, base_node.state['loc'], 0, 0, 0, speeding)
            else:
                dest_node_state = {"loc": dest_node_loc, "speed_sessions": base_node.state["speed_sessions"]}
                heuristic_dest_node = Node(dest_node_state, base_node.state['loc'], 0, 0, 0, speeding)


            dest_node_speedable = dest_node_state["speed_sessions"] > 0
            h = self.get_heuristic(heuristic_dest_node, goal_node, dest_node_speedable)

            if base_node.parent is not None:
                g = self.get_travel_time(road, base_node, heuristic_dest_node) + base_node.g
            else:
                g = self.get_travel_time(road, base_node, heuristic_dest_node)

            connected_nodes.append(Node(dest_node_state, base_node, road, g, h, speeding=speeding))

        return connected_nodes

    def get_travel_time(self, road, start_node: Node, end_node: Node):
        lat1 = float(start_node.state['loc'].latitude)
        long1 = float(start_node.state['loc'].longitude)
        lat2 = float(end_node.state['loc'].latitude)
        long2 = float(end_node.state['loc'].longitude)
        dist = distance_on_unit_sphere(lat1, long1, lat2, long2) * 3960
        speed = int(road.speed)
        time_sec = dist / speed * 60 * 60
        return time_sec

    def get_heuristic(self, start_node: Node, end_node: Node, speeding: bool):
        lat1 = start_node.state['loc'].latitude
        long1 = start_node.state['loc'].longitude
        lat2 = end_node.state['loc'].latitude
        long2 = end_node.state['loc'].longitude
        dist = distance_on_unit_sphere(lat1, long1, lat2, long2) * 3960
        speed = 130 if speeding else 65
        time_sec = dist / speed * 60 * 60
        return time_sec

    def get_link_from_final_node(self, node, connected_road=None):
        linked_nodes = []

        current_node = node
        while current_node is not None:
            linked_nodes.insert(0, current_node.state['loc'].locId)
            roads = self.get_roads_connected_to(current_node.state['loc'].locId)
            # connected_road =""
            # for road in roads:
            #     if current_node.parent is not None:
            #         if road.endId == current_node.parent.state.locId:
            #             connected_road == road.name
            #             print("Connected_road is", connected_road.name)
            #
            current_node = current_node.parent

        return linked_nodes

    def get_a_star_path(self, start_node: Node, goal_node: Node, frontier: PQueue):

        print(start_node.state)
        node = Node(start_node.state, None, None, 0.0, self.get_heuristic(start_node, goal_node, True), speeding=None)
        self.frontier = frontier
        frontier.enqueue(node, node.f())
        self.reached = {hash(tuple(node.state.items())): node}

        while not frontier.empty():
            node = frontier.dequeue()
            self.node_visited += 1

            if node.__eq__(goal_node):
                print(str(node))
                # print("\nVisiting [state=", node.state['loc'].locId, ", parent=null", ", g=", node.g, ", h=", node.h, ", f=",
                #       node.f())
                print("Total time travel in seconds is", node.f())

                chain_to_goal = self.get_link_from_final_node(node)
                # print(chain_to_goal)
                print("\nChain Link is:")
                for chainlink in chain_to_goal:
                    print(chainlink)
                print("The number of nodes visited is: ", self.node_visited, " Nodes")
                return chain_to_goal

            if node.parent is not None:
                print("\n Visiting {}".format(str(node)))

            else:
                print("\n Visiting {}".format(str(node)))

            connected_nodes = self.get_nodes_connected_to(node, goal_node)
            for child_node in connected_nodes:
                child_state = child_node.state
                if hash(tuple(child_state.items())) not in self.reached or child_node.f() < self.reached[
                   hash(tuple(child_state.items()))].f():

                    self.reached[hash(tuple(child_state.items()))] = child_node
                    frontier.enqueue(child_node, child_node.f())
                    print("    Adding [{}] ".format(str(child_node)))
                else:
                    print("    Skipping [{}] ".format(str(child_node)))

        return None


def main():
    graph = RoadNetwork()
    frontier = PQueue()

    # filename = input('Enter a filename: ')
    filename = "memphis-medium.txt"

    # Create graph network

    with open(filename, 'r') as file:
        for line in file:
            line = line.rstrip()
            pieces = line.split('|')

            if pieces[0] == 'location':
                _, locId, latitude, longitude = pieces
                locId = int(locId)
                longitude = float(longitude)
                latitude = float(latitude)
                loc = Location(locId, latitude, longitude)
                graph.add_location(loc)

            elif pieces[0] == 'road':
                _, startId, endId, speed, name = pieces
                startId = int(startId)
                endId = int(endId)
                speed = int(speed)
                road = Road(startId, endId, speed, name)
                graph.add_road(road)
                road_backwards = Road(endId, startId, speed, name)
                graph.add_road(road_backwards)

    # speed = int(intput('Enter number of times allowed to speed: )
    speed_sessions = 2
    start_state = {'loc': graph.get_location_by_id(int(203874746)), 'speed_sessions': speed_sessions}
    end_state = {'loc': graph.get_location_by_id(int(203744893)), 'speed_sessions': speed_sessions}

    # start_loc = int(input('Enter start location: '))
    start_node = Node(start_state, None, 0, 0, speeding=None)

    # end_loc = int(input('Enter end location: '))
    goal_node = Node(end_state, None, 0, 0, speeding=None)

    a_star_path = graph.get_a_star_path(start_node, goal_node, frontier)


main()
