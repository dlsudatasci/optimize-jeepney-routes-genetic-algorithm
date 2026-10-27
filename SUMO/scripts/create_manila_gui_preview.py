
from pathlib import Path
import shutil
import zipfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]

SOURCE_NETWORK = (
    ROOT / "SUMO/networks/Manila/Manila_routable.net.xml"
)

SOURCE_ROUTES = (
    ROOT / "SUMO/routes/Manila/Manila_GA_routes.rou.xml"
)

PREVIEW_DIR = ROOT / "SUMO/preview/Manila"

PREVIEW_NETWORK = PREVIEW_DIR / "Manila_preview.net.xml"
PREVIEW_ROUTES = PREVIEW_DIR / "Manila_preview.rou.xml"
PREVIEW_CONFIG = PREVIEW_DIR / "Manila_preview.sumocfg"

PREVIEW_ZIP = (
    ROOT / "SUMO/preview/Manila_GUI_preview.zip"
)

COLORS = [
    "255,140,0",
    "0,180,255",
    "50,220,100",
    "255,70,100",
    "180,100,255",
    "255,215,0",
    "0,220,200",
    "255,110,220",
]


def write_xml(root, destination):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(
        destination,
        encoding="utf-8",
        xml_declaration=True,
    )


def create_preview():
    PREVIEW_DIR.mkdir(parents=True, exist_ok=True)

    print("Copying SUMO road network...")
    shutil.copy2(SOURCE_NETWORK, PREVIEW_NETWORK)

    print("Loading GA route definitions...")
    source_root = ET.parse(SOURCE_ROUTES).getroot()
    routes = source_root.findall("route")

    if len(routes) != 176:
        raise ValueError(
            f"Expected 176 routes, found {len(routes)}."
        )

    preview_root = ET.Element("routes")

    # Temporary jeepney type for visualization.
    ET.SubElement(
        preview_root,
        "vType",
        {
            "id": "preview_jeepney",
            "vClass": "bus",
            "guiShape": "bus",
            "length": "6.0",
            "width": "2.1",
            "minGap": "2.0",
            "accel": "1.5",
            "decel": "4.5",
            "sigma": "0.5",
            "maxSpeed": "11.11",
        },
    )

    # Preserve the original GA edge sequences.
    for route in routes:
        ET.SubElement(
            preview_root,
            "route",
            dict(route.attrib),
        )

    # Add one illustrative jeepney per route.
    for index, route in enumerate(routes):
        route_id = route.attrib["id"]
        pair_id = int(route_id.split("-")[0])

        ET.SubElement(
            preview_root,
            "vehicle",
            {
                "id": f"preview_{route_id}",
                "type": "preview_jeepney",
                "route": route_id,
                "line": route_id,
                "depart": str(index * 2),
                "departLane": "free",
                "color": COLORS[pair_id % len(COLORS)],
            },
        )

    write_xml(preview_root, PREVIEW_ROUTES)

    # Portable SUMO configuration.
    config = ET.Element("configuration")

    inputs = ET.SubElement(config, "input")

    ET.SubElement(
        inputs,
        "net-file",
        {"value": PREVIEW_NETWORK.name},
    )

    ET.SubElement(
        inputs,
        "route-files",
        {"value": PREVIEW_ROUTES.name},
    )

    timing = ET.SubElement(config, "time")

    ET.SubElement(timing, "begin", {"value": "0"})
    ET.SubElement(timing, "end", {"value": "1800"})
    ET.SubElement(
        timing,
        "step-length",
        {"value": "1"},
    )

    write_xml(config, PREVIEW_CONFIG)

    # Package for local Windows viewing.
    print("Creating portable ZIP...")
    PREVIEW_ZIP.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    with zipfile.ZipFile(
        PREVIEW_ZIP,
        "w",
        compression=zipfile.ZIP_DEFLATED,
    ) as archive:
        for path in (
            PREVIEW_NETWORK,
            PREVIEW_ROUTES,
            PREVIEW_CONFIG,
        ):
            archive.write(path, arcname=path.name)

    print("\n=== GUI PREVIEW READY ===")
    print("GA routes:", len(routes))
    print("Preview jeepneys:", len(routes))
    print("Simulation duration: 1800 seconds")
    print("SUMO configuration:", PREVIEW_CONFIG)
    print("Portable ZIP:", PREVIEW_ZIP)


if __name__ == "__main__":
    create_preview()
