import math

_A = -1.4648375
_B = 93.75

SPEED_LIMIT_KM_HR = 60.0
SPEED_LIMIT_FLOW_THRESHOLD = 351.0  # veh/hr - below this, speed capped at limit
CAPACITY_FLOW_VEH_HR = 1500.0
CAPACITY_SPEED_KM_HR = 32.0
INTERSECTION_DELAY_S = 30.0

def flow_to_speed(flow_veh_per_hour):
    """
    Convert traffic flow (veh/hr) to speed (km/h) using the
    simplified fundamental diagram from the assignment spec.
    """
    
    flow = max(0.0, flow_veh_per_hour)
    if flow <= SPEED_LIMIT_FLOW_THRESHOLD:
        return SPEED_LIMIT_KM_HR
    
    discriminant = _B**2 + 4*_A*flow
    if discriminant < 0:
        return CAPACITY_SPEED_KM_HR
    
    sqrt_disc = math.sqrt(discriminant)
    if flow <= CAPACITY_FLOW_VEH_HR:
        speed = (-_B + sqrt_disc) / (2*_A)   # green branch
    else: 
        speed = (-_B - sqrt_disc) / (2*_A)   # red branch
    
    return max(1.0, min(speed, SPEED_LIMIT_KM_HR))

def travel_time_seconds(dist_km, flow_15min, n_intersections=1):
    """
    Estimate travel time in seconds for one edge.

    - Parameters:
    dist_km        : distance between the two SCATS sites in km
    flow_15min     : predicted traffic flow at the START site (veh/15min)
    n_intersections: controlled intersections on the link (default 1)

    - Returns:
    float  estimated travel time in seconds
    """
    
    flow_hr = flow_15min * 4
    speed = flow_to_speed(flow_hr)
    drive_s = (dist_km / speed) * 3600
    
    return drive_s + n_intersections * INTERSECTION_DELAY_S

if __name__ == '__main__':
    print(f"{'Flow (veh/hr)':>15}  {'Speed (km/h)':>12}")
    
    for f in [0, 100, 351, 600, 1000, 1500, 1800]:
        print(f"{f:>15}  {flow_to_speed(f):>12.2f}")
