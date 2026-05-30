import math


def flow_to_speed(flow_per_15min):
    flow = flow_per_15min * 4
    a = -1.4648375
    b = 93.75
    c = -flow

    discriminant = b ** 2 - 4 * a * c
    if discriminant < 0:
        return 1.0

    sqrt_disc = math.sqrt(discriminant)
    speed1 = (-b + sqrt_disc) / (2 * a)
    speed2 = (-b - sqrt_disc) / (2 * a)

    if flow <= 1500:
        speed = max(speed1, speed2)
    else:
        speed = min(speed1, speed2)

    return max(1.0, min(speed, 60))


def travel_time(distance_km, flow_per_15min):
    speed = flow_to_speed(flow_per_15min)
    return (distance_km / speed) * 3600 + 30  # +30s intersection delay
