
from pathlib import Path
import pickle
import osmnx as ox


# These classes allow us to load objects saved by
# your original Jupyter notebook without running
# the notebook or the genetic algorithm.
class stopCandidate:
    pass


class networkObj:
    pass


class Route:
    pass


PROJECT_ROOT = Path(__file__).resolve().parents[2]

ORIGINAL_GRAPH_PATH = PROJECT_ROOT / "map/Manila.graphml"

SAVED_NETWORKS = PROJECT_ROOT / "Saved Networks/Manila"

STOP_GRAPH_PATH = SAVED_NETWORKS / "stop_graph_Manila.pkl"

GA_PATH = SAVED_NETWORKS / (
    "Manila_GA_Route_networks_"
    "Area_W300_cov17_k17_t17_r17_b17_h17_S42.pkl"
)


def load_pickle(path):
    with open(path, "rb") as file:
        return pickle.load(file)


def main():
    print("Loading original Manila road graph...")
    original_graph = ox.load_graphml(ORIGINAL_GRAPH_PATH)

    print("Loading saved stop graph...")
    stop_graph = load_pickle(STOP_GRAPH_PATH)

    print("Loading selected GA results...")
    ga_data = load_pickle(GA_PATH)

    population = ga_data["population"]

    # Your existing notebook uses population[0].
    selected_network = population[0]

    print("\n=== ROAD NETWORK ===")
    print("Original nodes:", original_graph.number_of_nodes())
    print("Original edges:", original_graph.number_of_edges())

    print("Stop-graph nodes:", stop_graph.number_of_nodes())
    print("Stop-graph edges:", stop_graph.number_of_edges())

    original_nodes = set(original_graph.nodes)
    stop_nodes = set(stop_graph.nodes)

    print(
        "Additional nodes in stop graph:",
        len(stop_nodes - original_nodes)
    )

    print(
        "Original nodes missing from stop graph:",
        len(original_nodes - stop_nodes)
    )

    print("Stop graph CRS:", stop_graph.graph.get("crs"))

    print("\n=== SELECTED GA NETWORK ===")
    print("Population size:", len(population))
    print("Forward routes:", len(selected_network.routes))
    print("Reverse routes:", len(selected_network.reversed_routes))

    print(
        "Forward stops:",
        len(selected_network.stops)
    )

    print(
        "Reverse stops:",
        len(selected_network.reversed_stops)
    )

    print(
        "Selected fitness score:",
        getattr(selected_network, "fitness_score", None)
    )

    scores = [
        (i, float(network.fitness_score))
        for i, network in enumerate(population)
        if getattr(network, "fitness_score", None) is not None
    ]

    if scores:
        highest_index, highest_score = max(
            scores,
            key=lambda item: item[1]
        )

        print("Highest-scoring population index:", highest_index)
        print("Highest fitness score:", highest_score)

    print("\n=== ROUTE VALIDATION ===")

    missing_edges = []
    disconnected_connections = []
    ambiguous_edges = set()
    checked_edges = 0

    all_routes = (
        list(selected_network.routes)
        + list(selected_network.reversed_routes)
    )

    for route in all_routes:
        if route is None:
            continue

        connections = route.route

        for i, connection in enumerate(connections):
            if not connection:
                continue

            if i > 0 and connections[i - 1]:
                if connections[i - 1][-1] != connection[0]:
                    disconnected_connections.append(
                        (route.route_id, i)
                    )

            for u, v in zip(connection[:-1], connection[1:]):
                checked_edges += 1

                if not stop_graph.has_edge(u, v):
                    missing_edges.append(
                        (route.route_id, u, v)
                    )

                elif stop_graph.number_of_edges(u, v) > 1:
                    ambiguous_edges.add((u, v))

    print("Road-edge traversals checked:", checked_edges)
    print("Missing directed edges:", len(missing_edges))
    print(
        "Disconnected route connections:",
        len(disconnected_connections)
    )
    print(
        "Unique parallel-edge ambiguities:",
        len(ambiguous_edges)
    )

    selected_stops = (
        set(selected_network.stops)
        | set(selected_network.reversed_stops)
    )

    missing_stops = selected_stops - stop_nodes

    print("Unique selected stops:", len(selected_stops))
    print("Selected stops missing from graph:", len(missing_stops))

    if missing_edges:
        print("Example missing edges:", missing_edges[:5])

    if disconnected_connections:
        print(
            "Example disconnected connections:",
            disconnected_connections[:5]
        )

    if missing_stops:
        print("Example missing stops:", list(missing_stops)[:5])

    print("\n=== SAMPLE ROUTE ===")

    if selected_network.routes:
        route = selected_network.routes[0]

        print("Route ID:", route.route_id)
        print("Number of stop-to-stop paths:", len(route.route))

        if route.route:
            print(
                "First path:",
                list(route.route[0])[:10]
            )
            
            
    # --------------------------------------------------
    # Inspect parallel edges used by the selected routes
    # --------------------------------------------------

    print("\n=== PARALLEL EDGE INSPECTION ===")

    ambiguous_pairs = set()

    all_routes = (
        list(selected_network.routes)
        + list(selected_network.reversed_routes)
    )

    for route in all_routes:
        for connection in route.route:
            for u, v in zip(connection[:-1], connection[1:]):
                if stop_graph.number_of_edges(u, v) > 1:
                    ambiguous_pairs.add((u, v))

    tied_pairs = []

    for u, v in sorted(ambiguous_pairs):

        edge_options = []

        for key, data in stop_graph[u][v].items():
            edge_options.append({
                "key": key,
                "length": float(data["length"]),
                "source_edge_id": data.get("edge_id"),
            })

        edge_options.sort(key=lambda item: item["length"])

        print(f"\nNode pair: {u} -> {v}")

        for option in edge_options:
            print(
                f"  Key: {option['key']} | "
                f"Length: {option['length']:.6f} m | "
                f"Source edge ID: {option['source_edge_id']}"
            )

        minimum_length = edge_options[0]["length"]

        shortest_options = [
            option
            for option in edge_options
            if abs(option["length"] - minimum_length) < 1e-6
        ]

        if len(shortest_options) > 1:
            tied_pairs.append((u, v))

    print("\nTotal ambiguous pairs:", len(ambiguous_pairs))
    print("Pairs with tied minimum lengths:", len(tied_pairs))
    print("Tied pairs:", tied_pairs)


if __name__ == "__main__":
    main()
