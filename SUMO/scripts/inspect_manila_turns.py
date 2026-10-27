
from pathlib import Path
import math
import pickle

import sumolib


class stopCandidate:
    pass


class networkObj:
    pass


class Route:
    pass


ROOT = Path(__file__).resolve().parents[2]

GRAPH_PATH = (
    ROOT / "Saved Networks/Manila/stop_graph_Manila.pkl"
)

GA_PATH = (
    ROOT / "Saved Networks/Manila/"
    "Manila_GA_Route_networks_Area_W300_cov17_k17_t17_r17_b17_h17_S42.pkl"
)

SUMO_PATH = (
    ROOT / "SUMO/networks/Manila/Manila.net.xml"
)

MISSING_TURNS = [
    (
        "e_1315_6525656067_0",
        "e_6525656067_1314_0",
    ),
    (
        "e_1658_9791018387_0",
        "e_9791018387_1657_0",
    ),
    (
        "e_1922_12015446657_0",
        "e_12015446657_1923_0",
    ),
    (
        "e_2586_1399440591_0",
        "e_1399440591_2585_0",
    ),
]


def parse_edge_id(identifier):
    u, v, key = identifier[2:].rsplit("_", 2)
    return int(u), int(v), int(key)


def get_route_edge_id(graph, u, v):
    edge_data = graph.get_edge_data(u, v)

    key = min(
        edge_data,
        key=lambda k: float(edge_data[k]["length"]),
    )

    return f"e_{u}_{v}_{key}"


def turn_angle(first_edge, second_edge):
    shape_a = first_edge.getShape()
    shape_b = second_edge.getShape()

    if len(shape_a) < 2 or len(shape_b) < 2:
        return None

    a1, a2 = shape_a[-2], shape_a[-1]
    b1, b2 = shape_b[0], shape_b[1]

    heading_a = math.degrees(
        math.atan2(a2[1] - a1[1], a2[0] - a1[0])
    )

    heading_b = math.degrees(
        math.atan2(b2[1] - b1[1], b2[0] - b1[0])
    )

    return (heading_b - heading_a + 180) % 360 - 180


def main():
    with GRAPH_PATH.open("rb") as file:
        graph = pickle.load(file)

    with GA_PATH.open("rb") as file:
        network = pickle.load(file)["population"][0]

    net = sumolib.net.readNet(str(SUMO_PATH))

    routes = (
        list(network.routes)
        + list(network.reversed_routes)
    )

    affected_routes = {
        pair: set()
        for pair in MISSING_TURNS
    }

    for route in routes:
        if not route.route:
            continue

        nodes = list(route.route[0])

        for segment in route.route[1:]:
            nodes.extend(segment[1:])

        edge_ids = [
            get_route_edge_id(graph, u, v)
            for u, v in zip(nodes[:-1], nodes[1:])
        ]

        for pair in zip(edge_ids[:-1], edge_ids[1:]):
            if pair in affected_routes:
                affected_routes[pair].add(str(route.route_id))

    for index, (first_id, second_id) in enumerate(
        MISSING_TURNS, start=1
    ):
        first = net.getEdge(first_id)
        second = net.getEdge(second_id)

        angle = turn_angle(first, second)

        print(f"\n=== MISSING TURN {index} ===")
        print("From:", first_id)
        print("To:  ", second_id)

        print(
            "Shared junction:",
            first.getToNode().getID(),
        )

        print(
            "Approximate turn angle:",
            f"{angle:.2f} degrees"
            if angle is not None else "Unavailable",
        )

        print(
            "Affected routes:",
            sorted(affected_routes[(first_id, second_id)]),
        )

        print("Original road attributes:")

        for identifier in (first_id, second_id):
            u, v, key = parse_edge_id(identifier)
            data = graph[u][v][key]

            print(f"\n  {identifier}")

            for attribute in (
                "name",
                "highway",
                "oneway",
                "osmid",
                "length",
                "edge_id",
            ):
                print(
                    f"    {attribute}: "
                    f"{data.get(attribute)}"
                )

        print("\nSUMO's currently allowed outgoing edges:")

        outgoing_ids = sorted(
            edge.getID()
            for edge in first.getOutgoing()
        )

        for identifier in outgoing_ids:
            print(" ", identifier)

        if not outgoing_ids:
            print("  None")


if __name__ == "__main__":
    main()
