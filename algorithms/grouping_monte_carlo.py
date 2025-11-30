from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Tuple
import random

from city.graph import CityGraph, NodeId
from domain.models import Vehicle, DeliveryRequest, VehicleRoute, Plan
from algorithms.a_star import astar_shortest_path

INF = 10**18


def _compute_time(
    graph: CityGraph,
    src: NodeId,
    dst: NodeId,
) -> float:
    if src == dst:
        return 0.0
    path, cost = astar_shortest_path(graph, src, dst)
    if path is None:
        return INF
    return cost


def precompute_distances(
    graph: CityGraph,
    depot: NodeId,
    requests: List[DeliveryRequest],
) -> Dict[Tuple[NodeId, NodeId], float]:
    nodes: List[NodeId] = [depot] + [r.node for r in requests]
    dist: Dict[Tuple[NodeId, NodeId], float] = {}
    for a in nodes:
        for b in nodes:
            if (a, b) in dist:
                continue
            t = _compute_time(graph, a, b)
            dist[(a, b)] = t
            dist[(b, a)] = t
    return dist


def route_time(
    depot: NodeId,
    stops: List[DeliveryRequest],
    dist: Dict[Tuple[NodeId, NodeId], float],
    vehicle_capacity: int,
) -> float:
    """
    Compute route time given capacity:
    - start from the depot,
    - in one trip you can carry at most vehicle_capacity demand,
    - if there is not enough space for the next request -> return to depot and start a new trip.
    """
    if not stops:
        return 0.0
    if vehicle_capacity <= 0:
        return INF

    total = 0.0
    cur = depot
    cap_left = vehicle_capacity

    for r in stops:
        demand = getattr(r, "demand", 1)
        if demand <= 0:
            return INF

        # Not enough space — return to depot first
        if cap_left < demand:
            if cur != depot:
                total += dist[(cur, depot)]
                cur = depot
            cap_left = vehicle_capacity

        # Go to the request node
        total += dist[(cur, r.node)]
        cur = r.node
        cap_left -= demand

    # Return to the depot at the end
    if cur != depot:
        total += dist[(cur, depot)]

    return total




def build_tsp_route_nearest_neighbor(
    depot: NodeId,
    requests: List[DeliveryRequest],
    dist: Dict[Tuple[NodeId, NodeId], float],
) -> List[DeliveryRequest]:
    if not requests:
        return []

    remaining = list(requests)
    cur = depot
    route: List[DeliveryRequest] = []

    while remaining:
        best_idx = 0
        best_t = INF

        for i, r in enumerate(remaining):
            t = dist[(cur, r.node)]
            if t < best_t:
                best_t = t
                best_idx = i

        chosen = remaining.pop(best_idx)
        route.append(chosen)
        cur = chosen.node

    return route



State = Dict[str, List[DeliveryRequest]]


def init_state_random(
    vehicles: List[Vehicle],
    requests: List[DeliveryRequest],
    rng: random.Random,
) -> State:
    state: State = {v.id: [] for v in vehicles}

    shuffled = requests[:]
    rng.shuffle(shuffled)

    # simply distribute requests across vehicles uniformly/randomly
    for req in shuffled:
        v = rng.choice(vehicles)
        state[v.id].append(req)

    return state




def evaluate_state(
    depot: NodeId,
    vehicles: List[Vehicle],
    state: State,
    dist: Dict[Tuple[NodeId, NodeId], float],
) -> float:
    makespan = 0.0

    for v in vehicles:
        if v.capacity <= 0:
            return INF

        reqs = state[v.id]
        if not reqs:
            continue

        order = build_tsp_route_nearest_neighbor(depot, reqs, dist)
        cost = route_time(depot, order, dist, v.capacity)
        makespan = max(makespan, cost)

    return makespan




def random_move(
    state: State,
    vehicles: List[Vehicle],
    rng: random.Random,
) -> State:
    new_state: State = {vid: lst[:] for vid, lst in state.items()}
    v_ids = [v.id for v in vehicles]

    from_vid = rng.choice(v_ids)
    if not new_state[from_vid]:
        return new_state

    r_idx = rng.randrange(len(new_state[from_vid]))
    req = new_state[from_vid].pop(r_idx)

    to_candidates = [vid for vid in v_ids if vid != from_vid]
    if not to_candidates:
        new_state[from_vid].append(req)
        return new_state

    to_vid = rng.choice(to_candidates)
    new_state[to_vid].append(req)

    return new_state


@dataclass
class GroupingResult:
    plan: Plan
    makespan_estimate: float


class MonteCarloGroupingPlanner:
    """
    Monte Carlo planner that optimizes only the grouping of requests
    (i.e., which vehicle serves which set of requests).
    Inside each group, the visit order is built using a nearest-neighbor heuristic.
    """

    def __init__(self, rng: random.Random, iterations: int = 2000) -> None:
        self.rng = rng
        self.iterations = iterations

    def build_plan(
        self,
        graph: CityGraph,
        depot: NodeId,
        vehicles: List[Vehicle],
        requests: List[DeliveryRequest],
    ) -> GroupingResult:
        if not requests:
            routes = {
                v.id: VehicleRoute(vehicle_id=v.id)
                for v in vehicles
            }
            return GroupingResult(plan=Plan(depot=depot, routes=routes),
                                  makespan_estimate=0.0)

        # Precompute travel-time matrix
        dist = precompute_distances(graph, depot, requests)

        # Initialization
        state = init_state_random(vehicles, requests, self.rng)
        best_state = state
        best_cost = evaluate_state(depot, vehicles, state, dist)

        current_state = state
        current_cost = best_cost

        for it in range(self.iterations):
            candidate = random_move(current_state, vehicles, self.rng)
            cand_cost = evaluate_state(depot, vehicles, candidate, dist)

            # simple hill climbing: accept if not worse
            if cand_cost <= current_cost:
                current_state = candidate
                current_cost = cand_cost

                if cand_cost < best_cost:
                    best_cost = cand_cost
                    best_state = candidate

        # Build the final Plan from best_state
        routes: Dict[str, VehicleRoute] = {}
        for v in vehicles:
            reqs = best_state[v.id]
            ordered = build_tsp_route_nearest_neighbor(depot, reqs, dist)
            routes[v.id] = VehicleRoute(
                vehicle_id=v.id,
                stops=ordered,
            )

        plan = Plan(depot=depot, routes=routes)
        return GroupingResult(plan=plan, makespan_estimate=best_cost)
