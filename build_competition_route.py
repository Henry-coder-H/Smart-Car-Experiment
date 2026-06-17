import json
import math
import shutil


BASE_ROUTE = "exp_routes/birth13_to_big_2000.json"
PRIMARY_ROUTE = "exp_routes/birth13_right_big_5000.json"
BIG_RIGHT = "exp_routes/Big_right.json"
MID_RIGHT = "exp_routes/birth13_mid_right.json"
BIG_LOOP_RIGHT = "exp_routes/birth13_big_loop_right.json"


def save_route(path, route):
    with open(path, "w", encoding="utf-8") as file:
        json.dump(
            [{"x": round(point["x"], 3), "y": round(point["y"], 3)} for point in route],
            file,
            ensure_ascii=False,
            indent=2,
        )


def load_route(path):
    with open(path, "r", encoding="utf-8") as file:
        return [{"x": float(point["x"]), "y": float(point["y"])} for point in json.load(file)]


def validate_route(name, route):
    segments = [
        math.hypot(route[i + 1]["x"] - route[i]["x"], route[i + 1]["y"] - route[i]["y"])
        for i in range(len(route) - 1)
    ]
    max_heading_change = 0.0
    for i in range(len(route) - 2):
        dx1 = route[i + 1]["x"] - route[i]["x"]
        dy1 = route[i + 1]["y"] - route[i]["y"]
        dx2 = route[i + 2]["x"] - route[i + 1]["x"]
        dy2 = route[i + 2]["y"] - route[i + 1]["y"]
        if math.hypot(dx1, dy1) < 1e-6 or math.hypot(dx2, dy2) < 1e-6:
            continue
        h1 = math.atan2(dy1, dx1)
        h2 = math.atan2(dy2, dx2)
        max_heading_change = max(max_heading_change, abs((h2 - h1 + math.pi) % (2.0 * math.pi) - math.pi))

    print(
        name,
        "points", len(route),
        "length", round(route_length(route), 1),
        "max_segment", round(max(segments), 2),
        "max_heading_change_deg", round(math.degrees(max_heading_change), 2),
    )


def route_length(route):
    return sum(
        math.hypot(route[i + 1]["x"] - route[i]["x"], route[i + 1]["y"] - route[i]["y"])
        for i in range(len(route) - 1)
    )


# Use the original hand-authored map-following route as the safety source of truth.
# Do not synthesize roads from guessed geometry; the map has internal grass islands.
primary = load_route(BASE_ROUTE)

save_route(PRIMARY_ROUTE, primary)
validate_route(PRIMARY_ROUTE, primary)

for source, target in (
    ("exp_routes/Big.json", BIG_RIGHT),
    ("exp_routes/Big.json", BIG_LOOP_RIGHT),
    ("exp_routes/midInside.json", MID_RIGHT),
):
    try:
        route = load_route(source)
        save_route(target, route)
        validate_route(target, route)
    except FileNotFoundError:
        shutil.copyfile(PRIMARY_ROUTE, target)
        validate_route(target, primary)
