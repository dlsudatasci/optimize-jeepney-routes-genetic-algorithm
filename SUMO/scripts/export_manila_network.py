
from pathlib import Path
from collections import Counter
import csv
import math
import pickle
import re
import xml.etree.ElementTree as ET

import osmnx as ox
from pyproj import Transformer


# Required for unpickling objects created in your GA notebook.
class stopCandidate:
    pass


class networkObj:
    pass


class Route:
    pass


PROJECT_ROOT = Path(__file__).resolve().parents[2]

ORIGINAL_GRAPH_PATH = PROJECT_ROOT / "map/Manila.graphml"

STOP_GRAPH_PATH = (
    PROJECT_ROOT
    / "Saved Networks/Manila/stop_graph_Manila.pkl"
)

OUTPUT_DIR = PROJECT_ROOT / "SUMO/networks/Manila"

# Manila is in UTM Zone 51N.
TARGET_CRS = "EPSG:32651"

# Initial topology-validation assumptions only.
# These must be reviewed before scientific simulation runs.
FALLBACK_SPEED_KMH = 40.0
PROTOTYPE_LANES = 1

ROAD_PRIORITY = {
    "trunk": 4,
    "primary": 3,
    "secondary": 2,
    "tertiary": 1,
}


def get_road_priority(highway):
    values = highway if isinstance(highway, list) else [highway]
    return max(
        (ROAD_PRIORITY.get(str(value), 1) for value in values),
        default=1,
    )


def parse_speed(raw_value):
    """Return maximum speed in m/s, or None if unavailable."""
    values = raw_value if isinstance(raw_value, list) else [raw_value]
    parsed = []

    for value in values:
        if value is None:
            continue

        match = re.fullmatch(
            r"\s*(\d+(?:\.\d+)?)\s*(mph|km/h|kmh|kph)?\s*",
            str(value),
            flags=re.IGNORECASE,
        )

        if not match:
            continue

        speed = float(match.group(1))

        if match.group(2) and match.group(2).lower() == "mph":
            speed *= 1.609344

        if speed > 0:
            parsed.append(speed / 3.6)

    return min(parsed) if parsed else None


def node_id(node):
    return f"n_{node}"


def edge_id(u, v, key):
    return f"e_{u}_{v}_{key}"


def write_xml(element, path):
    tree = ET.ElementTree(element)
    ET.indent(tree, space="  ")
    tree.write(
        path,
        encoding="utf-8",
        xml_declaration=True,
    )


def main():
    print("Loading original road graph...")
    original_graph = ox.load_graphml(ORIGINAL_GRAPH_PATH)

    print("Loading saved stop graph...")
    with STOP_GRAPH_PATH.open("rb") as file:
        graph = pickle.load(file)

    assert graph.is_directed()
    assert graph.is_multigraph()

    original_nodes = set(original_graph.nodes)
    inserted_nodes = set(graph.nodes) - original_nodes

    assert original_nodes.issubset(set(graph.nodes))

    transformer = Transformer.from_crs(
        graph.graph["crs"],
        TARGET_CRS,
        always_xy=True,
    )

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    nodes_root = ET.Element("nodes")
    edges_root = ET.Element("edges")

    projected_nodes = {}

    # ---------------------------------------------
    # Export nodes
    # ---------------------------------------------

    for node, data in graph.nodes(data=True):
        lon = float(data["x"])
        lat = float(data["y"])

        x, y = transformer.transform(lon, lat)

        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f"Invalid node coordinates: {node}")

        projected_nodes[node] = (x, y)

        attributes = {
            "id": node_id(node),
            "x": f"{x:.6f}",
            "y": f"{y:.6f}",
        }

        # These nodes represent stops inserted along roads,
        # rather than independently observed intersections.
        if node in inserted_nodes:
            if graph.in_degree(node) != 1 or graph.out_degree(node) != 1:
                raise ValueError(
                    f"Inserted stop node {node} does not have "
                    "one incoming and one outgoing edge."
                )

            attributes["type"] = "unregulated"

        ET.SubElement(nodes_root, "node", attributes)

    # ---------------------------------------------
    # Export edges
    # ---------------------------------------------

    summary = Counter()
    edge_records = []

    for u, v, key, data in graph.edges(keys=True, data=True):

        length = float(data["length"])

        if not math.isfinite(length) or length <= 0:
            raise ValueError(
                f"Invalid length for {(u, v, key)}: {length}"
            )

        speed = parse_speed(data.get("maxspeed"))

        if speed is None:
            speed = FALLBACK_SPEED_KMH / 3.6
            summary["fallback_speed_edges"] += 1

        if data.get("lanes") is not None:
            summary["edges_with_lane_data"] += 1

        # Use stored curved geometry where available.
        geometry = data.get("geometry")

        if geometry is not None:
            if geometry.geom_type != "LineString":
                raise ValueError(
                    f"Unexpected geometry for {(u, v, key)}: "
                    f"{geometry.geom_type}"
                )

            geographic_coords = list(geometry.coords)

            if len(geographic_coords) < 2:
                raise ValueError(
                    f"Insufficient geometry points: {(u, v, key)}"
                )

            coordinates = [
                transformer.transform(lon, lat)
                for lon, lat in geographic_coords
            ]

        else:
            coordinates = [
                projected_nodes[u],
                projected_nodes[v],
            ]
            summary["straight_geometry_edges"] += 1

        ux, uy = projected_nodes[u]
        vx, vy = projected_nodes[v]

        # Ensure geometry follows the directed edge.
        forward_distance = (
            math.hypot(coordinates[0][0] - ux, coordinates[0][1] - uy)
            + math.hypot(coordinates[-1][0] - vx, coordinates[-1][1] - vy)
        )

        reverse_distance = (
            math.hypot(coordinates[0][0] - vx, coordinates[0][1] - vy)
            + math.hypot(coordinates[-1][0] - ux, coordinates[-1][1] - uy)
        )

        if reverse_distance < forward_distance:
            coordinates.reverse()
            summary["reversed_edge_shapes"] += 1

        endpoint_error = (
            math.hypot(coordinates[0][0] - ux, coordinates[0][1] - uy)
            + math.hypot(coordinates[-1][0] - vx, coordinates[-1][1] - vy)
        )

        if endpoint_error > 5:
            raise ValueError(
                f"Edge geometry does not match its nodes: "
                f"{(u, v, key)}, error={endpoint_error:.2f}m"
            )

        # Align exactly with the exported node coordinates.
        coordinates[0] = projected_nodes[u]
        coordinates[-1] = projected_nodes[v]

        shape = " ".join(
            f"{x:.6f},{y:.6f}"
            for x, y in coordinates
        )

        sumo_id = edge_id(u, v, key)

        ET.SubElement(
            edges_root,
            "edge",
            {
                "id": sumo_id,
                "from": node_id(u),
                "to": node_id(v),
                "numLanes": str(PROTOTYPE_LANES),
                "speed": f"{speed:.6f}",
                "priority": str(
                    get_road_priority(data.get("highway"))
                ),
                "length": f"{length:.6f}",
                "shape": shape,
                "spreadType": "right",
            },
        )

        edge_records.append({
            "sumo_edge_id": sumo_id,
            "u": u,
            "v": v,
            "key": key,
            "source_edge_id": data.get("edge_id"),
            "length_m": length,
        })

        if length < 10:
            summary["edges_shorter_than_10m"] += 1

    # ---------------------------------------------
    # Save files
    # ---------------------------------------------

    write_xml(
        nodes_root,
        OUTPUT_DIR / "Manila.nod.xml",
    )

    write_xml(
        edges_root,
        OUTPUT_DIR / "Manila.edg.xml",
    )

    with (OUTPUT_DIR / "Manila_edge_map.csv").open(
        "w",
        newline="",
        encoding="utf-8",
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "sumo_edge_id",
                "u",
                "v",
                "key",
                "source_edge_id",
                "length_m",
            ],
        )
        writer.writeheader()
        writer.writerows(edge_records)

    print("\n=== EXPORT COMPLETE ===")
    print("Output directory:", OUTPUT_DIR)
    print("Nodes exported:", len(projected_nodes))
    print("Edges exported:", len(edge_records))
    print("Inserted stop nodes:", len(inserted_nodes))

    for key, value in sorted(summary.items()):
        print(f"{key}: {value}")

    print("\nPrototype assumptions:")
    print(f"  Lane count: {PROTOTYPE_LANES} per directed edge")
    print(f"  Fallback speed: {FALLBACK_SPEED_KMH} km/h")
    print("  No observed traffic-signal timing supplied")
    print("  No background vehicle demand supplied")


if __name__ == "__main__":
    main()
