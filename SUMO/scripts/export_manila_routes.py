
from pathlib import Path
import json
import pickle
import xml.etree.ElementTree as ET

import sumolib


# Classes needed to load the existing GA pickle.
class stopCandidate:
    pass


class Route:
    pass


class networkObj:
    pass


ROOT = Path(__file__).resolve().parents[2]

GA_FILE = (
    ROOT
    / "Saved Networks/Manila"
    / "Manila_GA_Route_networks_Area_W300_cov17_k17_t17_r17_b17_h17_S42.pkl"
)

GRAPH_FILE = (
    ROOT / "Saved Networks/Manila/stop_graph_Manila.pkl"
)

SUMO_NET_FILE = (
    ROOT / "SUMO/networks/Manila/Manila_routable.net.xml"
)

OUTPUT_DIR = ROOT / "SUMO/routes/Manila"

ROUTES_FILE = OUTPUT_DIR / "Manila_GA_routes.rou.xml"
MANIFEST_FILE = OUTPUT_DIR / "Manila_GA_route_manifest.json"


def load_pickle(path):
    with path.open("rb") as file:
        return pickle.load(file)


def flatten_route(route_obj):
    """Return road nodes and designated stop positions."""

    paths = route_obj.route

    if not paths:
        raise ValueError(
            f"Route {route_obj.route_id} has no paths."
        )

    nodes = [paths[0][0]]
    stop_indices = [0]

    for path in paths:
        if len(path) < 2:
            raise ValueError(
                f"Invalid path in {route_obj.route_id}: {path}"
            )

        if nodes[-1] != path[0]:
            raise ValueError(
                f"Disconnected stop-to-stop paths "
                f"in route {route_obj.route_id}."
            )

        nodes.extend(path[1:])
        stop_indices.append(len(nodes) - 1)

    return nodes, stop_indices


def select_edge_id(graph, u, v):
    """Reproduce the GA's parallel-edge selection rule."""

    candidates = graph.get_edge_data(u, v)

    if not candidates:
        raise ValueError(
            f"Missing directed road edge: {u} -> {v}"
        )

    edge_key, _ = min(
        candidates.items(),
        key=lambda item: (
            float(item[1].get("length", float("inf"))),
            str(item[0]),
        ),
    )

    return f"e_{u}_{v}_{edge_key}"


def export_routes():
    print("Loading selected GA results...")
    saved = load_pickle(GA_FILE)
    population = saved["population"]

    # The highest-scoring network previously validated.
    selected = population[0]

    print("Loading saved stop graph...")
    graph = load_pickle(GRAPH_FILE)

    print("Loading patched SUMO network...")
    sumo_net = sumolib.net.readNet(str(SUMO_NET_FILE))

    route_objects = (
        list(selected.routes)
        + list(selected.reversed_routes)
    )

    if len(route_objects) != 176:
        raise ValueError(
            f"Expected 176 route directions, "
            f"found {len(route_objects)}."
        )

    xml_root = ET.Element("routes")
    manifest = {}

    all_stops = set()
    total_traversals = 0

    for route_obj in route_objects:
        route_id = route_obj.route_id

        if route_obj is None or route_id in manifest:
            raise ValueError(
                f"Missing or duplicate route: {route_id}"
            )

        nodes, stop_indices = flatten_route(route_obj)

        edge_ids = []
        previous_edge = None

        for u, v in zip(nodes, nodes[1:]):
            edge_id = select_edge_id(graph, u, v)

            try:
                current_edge = sumo_net.getEdge(edge_id)
            except KeyError as error:
                raise ValueError(
                    f"{route_id}: SUMO edge missing: {edge_id}"
                ) from error

            if (
                previous_edge is not None
                and current_edge not in previous_edge.getOutgoing()
            ):
                raise ValueError(
                    f"{route_id}: Missing turning connection "
                    f"{previous_edge.getID()} -> {edge_id}"
                )

            edge_ids.append(edge_id)
            previous_edge = current_edge

        ET.SubElement(
            xml_root,
            "route",
            {
                "id": route_id,
                "edges": " ".join(edge_ids),
            },
        )

        stops = [
            {
                "node_id": str(nodes[index]),
                "road_node_index": index,
            }
            for index in stop_indices
        ]

        manifest[route_id] = {
            "stops": stops,
            "edge_count": len(edge_ids),
        }

        all_stops.update(
            stop["node_id"] for stop in stops
        )

        total_traversals += len(edge_ids)

    # Verify the selected solution before writing outputs.
    if len(all_stops) != 786:
        raise ValueError(
            f"Expected 786 unique stops, found {len(all_stops)}."
        )

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    ET.indent(xml_root, space="  ")
    ET.ElementTree(xml_root).write(
        ROUTES_FILE,
        encoding="utf-8",
        xml_declaration=True,
    )

    with MANIFEST_FILE.open("w", encoding="utf-8") as file:
        json.dump(manifest, file, indent=2)

    print("\n=== ROUTE EXPORT COMPLETE ===")
    print("Route definitions:", len(manifest))
    print("Forward routes:", len(selected.routes))
    print("Reverse routes:", len(selected.reversed_routes))
    print("Unique designated stops:", len(all_stops))
    print("Total road-edge traversals:", total_traversals)
    print("Routes file:", ROUTES_FILE)
    print("Manifest file:", MANIFEST_FILE)


if __name__ == "__main__":
    export_routes()
