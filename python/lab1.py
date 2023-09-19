from pqueue import PQueue
import math


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
    def __init__(self, state: Location, parent: Location, action=None, g=0, h=0):
        self.state = state  # Location
        self.parent = parent  # Parent Location
        self.action = action  # Connected roads with state as staring location
        self.g = g  # The path cost
        self.h = h  # The heuristic cost

    def f(self):
        return self.g + self.h

    def __eq__(self, other_node):
        return self.state.locId == other_node.state.locId

    def __hash__(self):
        return hash(self.state)


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

    def get_nodes_connected_to(self, base_node: Node):
        connected_nodes = []
        connected_roads = self.get_roads_connected_to(base_node.state.locId)
        for road in connected_roads:
            # Only defining dest_node just to have a Node instance
            dest_node = Node(self.get_location_by_id(road.endId), None, 0, 0, 0)
            g_cost = self.get_travel_time(road, base_node,dest_node)
            connected_nodes.append(Node(self.get_location_by_id(road.endId), base_node, road, g_cost, 0))

        return connected_nodes


    def get_travel_time(self,road, start_node: Node, end_node: Node):
        lat1 = start_node.state.latitude
        long1 = start_node.state.longitude
        lat2 = end_node.state.latitude
        long2 = end_node.state.longitude
        dist = distance_on_unit_sphere(lat1, long1, lat2, long2) * 3960
        speed = road.speed
        time_sec = dist/speed * 60 * 60
        return time_sec

    def get_link_from_final_node(self, final_node):
        linked_nodes =[]

        current_node = final_node
        while current_node is not None:
            linked_nodes.insert(0, current_node.state.locId)
            current_node = current_node.parent
        return linked_nodes




    def get_a_star_path(self, start_node: Node, end_node: Node, frontier: PQueue):
        node = Node(start_node.state, None, None, 0, 0)
        self.frontier = frontier
        frontier.enqueue(node, node.f())
        self.reached = {node.state.locId: node}

        while not frontier.empty():
            node = frontier.dequeue()
            if node.__eq__(end_node):
                # print("Reached Equal")
                chain_to_goal = self.get_link_from_final_node(node)
                print(chain_to_goal)
                return chain_to_goal

            connected_nodes = self.get_nodes_connected_to(node)
            for child_node in connected_nodes:
                child_state = child_node.state
                if child_state.locId not in self.reached or child_node.f() < self.reached[child_state.locId].f():
                    self.reached[child_state.locId] = child_node
                    frontier.enqueue(child_node, child_node.f())
                    # print("Adding Node:", child_node,"to frontier")

        return None

    # def expand(self, node: Node):
    #     child_node_collection = list();
    #     state = node.state
    #     for each action in actions(state):
    #         child_state =


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

    # start_loc = int(input('Enter start location: '))
    start_node = Node(graph.get_location_by_id(int(2471207719)), None, 0, 0)

    # end_loc = int(input('Enter end location: '))
    end_node = Node(graph.get_location_by_id(int(203785186)), None, 0, 0)

    a_star_path = graph.get_a_star_path(start_node, end_node, frontier)


main()
