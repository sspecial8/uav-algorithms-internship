from dataclasses import dataclass
from typing import List

@dataclass
class DroneSpec:
    name: str
    max_range_km: float
    payload_kg: float
    flight_time_min: float
    max_wind_ms: float
    cost_per_flight: float
    drone_type: str  # 'multirotor', 'fixed-wing', 'vtol'

@dataclass
class MissionSpec:
    distance_km: float
    payload_required_kg: float
    min_flight_time_min: float
    expected_wind_ms: float
    budget_per_flight: float
    requires_hover: bool  # True if hovering is required

DRONE_FLEET = [
    DroneSpec("DJI Matrice 300 RTK", 15, 2.7, 55, 15, 500, "multirotor"),
    DroneSpec("DJI Agras T40",       7,  40,  22, 12, 800, "multirotor"),
    DroneSpec("Bayraktar Mini TB2",  150, 55, 600, 20, 2000, "fixed-wing"),
    DroneSpec("WingtraOne GEN II",   60, 0.4, 59, 18, 1500, "vtol"),
    DroneSpec("Autel EVO II",        9,  0.8, 40, 10, 300, "multirotor"),
]

def select_uav(mission: MissionSpec, fleet: List[DroneSpec]) -> DroneSpec:
    candidates = []

    for drone in fleet:
        # Hard constraints
        if drone.max_range_km < mission.distance_km:
            continue
        if drone.payload_kg < mission.payload_required_kg:
            continue
        if drone.flight_time_min < mission.min_flight_time_min:
            continue
        if drone.max_wind_ms < mission.expected_wind_ms:
            continue
        if drone.cost_per_flight > mission.budget_per_flight:
            continue
        if mission.requires_hover and drone.drone_type == "fixed-wing":
            continue

        candidates.append(drone)

    if not candidates:
        return None

    # Soft criteria scoring
    def score(d):
        range_margin = (d.max_range_km - mission.distance_km) / d.max_range_km
        payload_margin = (d.payload_kg - mission.payload_required_kg) / d.payload_kg
        cost_score = 1 - (d.cost_per_flight / mission.budget_per_flight)

        return 0.4 * cost_score + 0.3 * range_margin + 0.3 * payload_margin

    return max(candidates, key=score)


# --- Example ---
mission = MissionSpec(
    distance_km=12,
    payload_required_kg=1.5,
    min_flight_time_min=30,
    expected_wind_ms=10,
    budget_per_flight=1000,
    requires_hover=True
)

best = select_uav(mission, DRONE_FLEET)

print(f"Recommended UAV: {best.name if best else 'None found'}")