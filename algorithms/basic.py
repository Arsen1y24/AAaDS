from dataclasses import dataclass
from typing import List, Dict, Optional

from city.graph import CityGraph, NodeId
from domain.models import Vehicle, DeliveryRequest, VehicleRoute, Plan
from algorithms.a_star import astar_shortest_path


@dataclass
class PlanCost:
    plan: Plan
    total_time: float


class GreedyVRPPlanner:
    """
    Simple baseline planner:
    – "capacity" is treated as the max load per ONE trip.
    – If there are more requests than capacity allows, the vehicle performs multiple trips:
      depot -> deliveries -> depot -> more deliveries -> depot.
    – Build_plan does not limit the total number of requests assigned to a vehicle,
      it only ensures that each request fits (demand <= capacity).
    """

    def _route_cost(
        self,
        graph: CityGraph,
        vehicle: Vehicle,
        stops: List[DeliveryRequest],
        depot: NodeId,
        return_to_depot: bool = True,
    ) -> float:
        """
        Computes the cost (time) of a route for the FULL list of stops,
        taking vehicle capacity into account: the vehicle may make
        multiple trips back to the depot.
        """
        cur = vehicle.start_node
        t = 0.0
        cap = vehicle.capacity
        if cap <= 0:
            return float("inf")

        cap_left = cap

        for req in stops:
            demand = getattr(req, "demand", 1)
            if demand <= 0:
                return float("inf")

            # If the next request does not fit -> go back to depot first
            if cap_left < demand:
                if cur != depot:
                    path, travel = astar_shortest_path(graph, cur, depot)
                    if path is None:
                        return float("inf")
                    t += travel
                    cur = depot
                cap_left = cap

            # Travel to the request node
            path, travel = astar_shortest_path(graph, cur, req.node)
            if path is None:
                return float("inf")
            t += travel
            cur = req.node
            cap_left -= demand

        # Return to depot at the end if requested
        if return_to_depot and cur != depot:
            path, travel = astar_shortest_path(graph, cur, depot)
            if path is None:
                return float("inf")
            t += travel

        return t

    def build_plan(
        self,
        graph: CityGraph,
        depot: NodeId,
        vehicles: List[Vehicle],
        requests: List[DeliveryRequest],
    ) -> PlanCost:

        # Safety: each request must fit into at least one vehicle
        for req in requests:
            demand = getattr(req, "demand", 1)
            if all(v.capacity < demand for v in vehicles):
                raise RuntimeError(
                    f"Request {req.id} has demand {demand}, "
                    f"but no vehicle can carry it in a single trip"
                )

        remaining = requests[:]
        routes: Dict[str, VehicleRoute] = {
            v.id: VehicleRoute(vehicle_id=v.id) for v in vehicles
        }

        while remaining:
            best_choice: Optional[tuple[str, DeliveryRequest, float]] = None

            for req in remaining:
                demand = getattr(req, "demand", 1)
                for v in vehicles:
                    # Vehicle must be able to carry this request at least in theory
                    if v.capacity < demand:
                        continue

                    stops = routes[v.id].stops + [req]
                    cost = self._route_cost(graph, v, stops, depot)
                    if best_choice is None or cost < best_choice[2]:
                        best_choice = (v.id, req, cost)

            if best_choice is None:
                # Should never happen because we checked demand <= max capacity
                raise RuntimeError("Cannot build feasible greedy plan (no vehicle can take remaining requests)")

            vid, chosen_req, _ = best_choice
            routes[vid].stops.append(chosen_req)
            remaining.remove(chosen_req)

        total_time = 0.0
        for v in vehicles:
            total_time += self._route_cost(graph, v, routes[v.id].stops, depot)

        return PlanCost(
            plan=Plan(depot=depot, routes=routes),
            total_time=total_time,
        )
