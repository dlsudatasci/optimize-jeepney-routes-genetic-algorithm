
from pathlib import Path
import pickle

import sumolib


class stopCandidate:
    pass


class networkObj:
    pass


class Route:
    pass


ROOT = Path(__file__).resolve().parents[2]

STOP_GRAPH_PATH = (
    ROOT / "Saved Networks/Manila/stop_graph_Manila.pkl"
)

GA_PATH = (
    ROOT / "Saved Networks/Manila/"
    "Manila_GA_Route_networks_Area_W300_cov17_k17_t17_r17_b17_h17_S42.pkl"
)

SUMO_NET_PATH = ROOT / "SUMO/networks/Manila/Manila_routable.net.xml"


def edge_id(u, v, key):
    return f"e_{u}_{v}_{key}"


def best_edge_id(graph, u, v):
    """Choose the minimum-length parallel edge, matching the GA."""
    edges = graph.get_edge_data(u, v)

    if not edges:
        raise ValueError(f"Missing NetworkX edge: {u} -> {v}")

    key = min(
        edges,
        key=lambda k: float(edges[k]["length"]),
    )

    return edge_id(u, v, key)


def main():
    print("Loading saved stop graph...")

    with STOP_GRAPH_PATH.open("rb") as file:
        graph = pickle.load(file)

    print("Loading selected GA solution...")

    with GA_PATH.open("rb") as file:
        ga_data = pickle.load(file)

    selected_network = ga_data["population"][0]

    print("Loading converted SUMO network...")

    net = sumolib.net.readNet(
        str(SUMO_NET_PATH),
        withConnections=True,
    )

    sumo_nodes = {node.getID() for node in net.getNodes()}

    sumo_edges = {
        edge.getID(): edge
        for edge in net.getEdges()
    }

    print("\n=== SUMO NETWORK ===")
    print("Original graph nodes:", graph.number_of_nodes())
    print("SUMO nodes:", len(sumo_nodes))
    print("Original graph edges:", graph.number_of_edges())
    print("SUMO edges:", len(sumo_edges))

    # Check whether any other road edges were removed.
    expected_edges = {
        edge_id(u, v, k)
        for u, v, k in graph.edges(keys=True)
    }

    removed_edges = expected_edges - set(sumo_edges)

    self_loops = {
        edge_id(u, v, k)
        for u, v, k in graph.edges(keys=True)
        if u == v
    }

    print("\n=== REMOVED ROAD EDGES ===")
    print("Total removed:", len(removed_edges))
    print("Removed self-loops:", len(removed_edges & self_loops))
    print("Other removed edges:", len(removed_edges - self_loops))

    for item in sorted(removed_edges - self_loops)[:10]:
        print("  Unexpected removal:", item)

    # Cache the outgoing connections for efficiency.
    outgoing = {
        edge_id_: {
            next_edge.getID()
            for next_edge in edge.getOutgoing()
        }
        for edge_id_, edge in sumo_edges.items()
    }

    routes = (
        list(selected_network.routes)
        + list(selected_network.reversed_routes)
    )

    missing_route_edges = set()
    missing_connections = set()
    missing_stop_nodes = set()
    disconnected_routes = []
    short_stop_approaches = set()
    valid_routes = 0
    total_transitions = 0
    selected_stops = set()

    for route in routes:
        if not route.route:
            disconnected_routes.append(route.route_id)
            continue

        # Reconstruct the complete road-node sequence.
        nodes = list(route.route[0])

        selected_stops.add(route.route[0][0])

        for segment in route.route:
            selected_stops.add(segment[-1])

        route_is_connected = True

        for segment in route.route[1:]:
            if nodes[-1] != segment[0]:
                route_is_connected = False
                break

            nodes.extend(segment[1:])

        if not route_is_connected:
            disconnected_routes.append(route.route_id)
            continue

        for stop in selected_stops:
            if f"n_{stop}" not in sumo_nodes:
                missing_stop_nodes.add(stop)

        route_edges = []

        for u, v in zip(nodes[:-1], nodes[1:]):
            identifier = best_edge_id(graph, u, v)

            if identifier not in sumo_edges:
                missing_route_edges.add(identifier)

            route_edges.append(identifier)

        route_has_problem = any(
            identifier not in sumo_edges
            for identifier in route_edges
        )

        for first, second in zip(
            route_edges[:-1],
            route_edges[1:],
        ):
            total_transitions += 1

            if (
                first in sumo_edges
                and second in sumo_edges
                and second not in outgoing[first]
            ):
                missing_connections.add((first, second))
                route_has_problem = True

        # Check the immediate inbound road segment at each stop.
        for segment in route.route:
            if len(segment) < 2:
                continue

            u, v = segment[-2], segment[-1]
            identifier = best_edge_id(graph, u, v)

            if identifier in sumo_edges:
                if sumo_edges[identifier].getLength() < 10:
                    short_stop_approaches.add(
                        (str(route.route_id), v, identifier)
                    )

        if not route_has_problem:
            valid_routes += 1

    print("\n=== OPTIMIZED ROUTE VALIDATION ===")
    print("Route directions checked:", len(routes))
    print("Routes without missing edges/turns:", valid_routes)
    print("Unique selected stops:", len(selected_stops))
    print("Missing stop nodes:", len(missing_stop_nodes))
    print("Missing route edges:", len(missing_route_edges))
    print("Missing turning connections:", len(missing_connections))
    print("Transitions checked:", total_transitions)
    print("Disconnected route definitions:", len(disconnected_routes))

    print("\n=== STOP PLACEMENT PRECHECK ===")
    print(
        "Route-stop approaches shorter than 10m:",
        len(short_stop_approaches),
    )

    if missing_route_edges:
        print("\nMissing route edges (first 10):")
        for item in sorted(missing_route_edges)[:10]:
            print(" ", item)

    if missing_connections:
        print("\nMissing turning connections (first 10):")
        for first, second in sorted(missing_connections)[:10]:
            print(f"  {first} -> {second}")

    if missing_stop_nodes:
        print("\nMissing stop nodes (first 10):")
        print(sorted(missing_stop_nodes)[:10])

    if disconnected_routes:
        print("\nDisconnected routes (first 10):")
        print(disconnected_routes[:10])

    if short_stop_approaches:
        print("\nShort stop approaches (first 10):")
        for item in sorted(short_stop_approaches)[:10]:
            print(" ", item)


if __name__ == "__main__":
    main()
