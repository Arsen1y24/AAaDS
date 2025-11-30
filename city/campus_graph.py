from typing import Dict
import math

from city.graph import CityGraph, NodeId

def build_campus_graph() -> "CityGraph":
    g = CityGraph()

    # Node coordinates
    coords = {
        "FH_W": (0.0, 0.0),
        "FH_M": (4.0, 0.0),
        "FH_E": (8.0, 0.0),

        # West side road
        "W_S": (0.0, 1.5),
        "W_M": (0.0, 3.5),
        "W_N": (0.0, 7.5),

        # East side road
        "E_S": (8.0, 1.5),
        "E_M": (8.0, 3.5),
        "E_N": (8.0, 7.5),

        # Steingutstraße
        "ST_W": (1.0, 8.0),
        "ST_M": (4.0, 8.0),
        "ST_E": (7.0, 8.0),

        # Central north–south axis
        "C_GATE_S": (4.0, 1.5),
        "C_MID":    (4.0, 4.0),
        "C_PARK":   (4.0, 6.5),

        # Inner ring
        "R_SW": (2.0, 2.0),
        "R_SE": (6.0, 2.0),
        "R_NE": (6.0, 6.0),
        "R_NW": (2.0, 6.0),

        # Internal campus points
        "C_MAIN":   (4.0, 3.8),
        "C_GREEN":  (3.6, 4.6),
        "KRUPP_S":  (5.5, 3.5),
        "KRUPP_N":  (5.5, 4.7),
        "PARK_N":   (4.0, 7.5),
    }

    for nid, (x, y) in coords.items():
        g.add_node(nid, x, y)

    # Add an edge with distance-based length
    def road(a: str, b: str, speed_limit: float, bidirectional: bool = True) -> None:
        na = g.nodes[a]
        nb = g.nodes[b]
        dist = math.hypot(na.x - nb.x, na.y - nb.y)
        g.add_edge(
            a, b,
            length=dist * 100.0,
            speed_limit=speed_limit,
            bidirectional=bidirectional
        )

    # Speed limits
    V_MAIN  = 13.9   # 50 km/h
    V_LOCAL = 8.3    # 30 km/h

    # Outer network
    road("FH_W", "FH_M", V_MAIN)
    road("FH_M", "FH_E", V_MAIN)

    road("FH_W", "W_S", V_MAIN)
    road("W_S", "W_M", V_MAIN)
    road("W_M", "W_N", V_MAIN)

    road("FH_E", "E_S", V_MAIN)
    road("E_S", "E_M", V_MAIN)
    road("E_M", "E_N", V_MAIN)

    road("W_N", "ST_W", V_MAIN)
    road("ST_W", "ST_M", V_MAIN)
    road("ST_M", "ST_E", V_MAIN)
    road("ST_E", "E_N", V_MAIN)

    road("ST_M", "C_PARK", V_LOCAL)
    road("C_PARK", "C_MID", V_LOCAL)

    # Inner ring
    road("R_SW", "R_SE", V_LOCAL)
    road("R_SE", "R_NE", V_LOCAL)
    road("R_NE", "R_NW", V_LOCAL)
    road("R_NW", "R_SW", V_LOCAL)

    # Ring–outer connectors
    road("FH_M", "C_GATE_S", V_LOCAL)
    road("C_GATE_S", "R_SW", V_LOCAL)
    road("C_GATE_S", "R_SE", V_LOCAL)

    road("W_S", "R_SW", V_LOCAL)
    road("E_S", "R_SE", V_LOCAL)
    road("W_N", "R_NW", V_LOCAL)
    road("E_N", "R_NE", V_LOCAL)

    # Central spine
    road("C_GATE_S", "C_MID", V_LOCAL)
    road("C_MID", "C_PARK", V_LOCAL)

    # Internal campus links
    road("R_SW", "C_MAIN", V_LOCAL)
    road("R_SE", "C_MAIN", V_LOCAL)
    road("C_MAIN", "C_GREEN", V_LOCAL)
    road("R_NW", "C_GREEN", V_LOCAL)
    road("R_NE", "C_GREEN", V_LOCAL)

    # Krupp College
    road("R_SE", "KRUPP_S", V_LOCAL)
    road("KRUPP_S", "KRUPP_N", V_LOCAL)
    road("KRUPP_N", "R_NE", V_LOCAL)

    # Park access
    road("C_PARK", "PARK_N", V_LOCAL)
    road("PARK_N", "ST_M", V_LOCAL)

    return g
