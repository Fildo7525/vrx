import argparse
import time
from dataclasses import dataclass
from tqdm import tqdm
from gz.transport13 import Node
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose


# There are 2 types of muslinge fields: With "buoys"and with "cylinder_buoys"
# 1. Buoys
# - The lines are 750m long
# - The buoys are spaced every 25m
# - The distance between lines is 12m
#
# 2. Cylinder_buoys
# - TBD
#

WORLD = "muslinge_world"

@dataclass
class Position:
    x: float
    y: float

@dataclass
class FieldParameters:
    line_length: float = 750.0
    buoy_spacing: float = 25.0
    line_distance: float = 12.0
    num_lines: int = 5

def generate_sdf_object(name: str,
                        x: float,
                        y: float,
                        z: float = 0,
                        uri: str = "https://fuel.gazebosim.org/1.0/openrobotics/models/mb_marker_buoy_red") -> str:
    return f"""
<sdf version="1.6">
    <include>
        <name>{name}</name>
        <uri>{uri}</uri>
        <pose>{x} {y} {z} 0 0 0</pose>
        <plugin name="vrx::PolyhedraBuoyancyDrag"
                        filename="libPolyhedraBuoyancyDrag.so">
            <fluid_density>1000</fluid_density>
            <fluid_level>0.0</fluid_level>
            <linear_drag>25.0</linear_drag>
            <angular_drag>2.0</angular_drag>
            <buoyancy name="collision_outer">
                <link_name>link</link_name>
                <pose>0 0 -0.3 0 0 0</pose>
                <geometry>
                    <cylinder>
                        <radius>0.325</radius>
                        <length>0.1</length>
                    </cylinder>
                </geometry>
            </buoyancy>
            <wavefield>
                <topic>/vrx/wavefield/parameters</topic>
            </wavefield>
        </plugin>
    </include>
</sdf>
"""

def caluclate_buoy_field(origin: Position,
                         params) -> list[tuple[float, float]]:
    """
    Calculate the positions of buoys in a field. The field consists of multiple lines of buoys.

    Args:
        origin (tuple[float, float]): The (x, y) coordinates of the center of the buoy field.
        line_length (float): The length of each line of buoys in meters. Default is 750.0.
        buoy_spacing (float): The spacing between buoys in meters. Default is 25.0.
        line_distance (float): The distance between lines of buoys in meters. Default is 12.0.
        num_lines (int): The number of lines of buoys. Default is 5.

    Returns:
        list[tuple[float, float]]: A list of (x, y) coordinates for each buoy in the field.
    """
    positions = []
    num_buoys_per_line = int(params.line_length / params.buoy_spacing) + 1

    for line in range(params.num_lines):
        y = origin.y + (line - params.num_lines // 2) * params.line_distance
        for buoy in range(num_buoys_per_line):
            x = origin.x - (params.line_length / 2) + buoy * params.buoy_spacing
            positions.append((x, y))

    return positions


def publish_buoy_field(world: str = "muslinge_world",
                           origin: Position = Position(0.0, 0.0),
                           params: FieldParameters = FieldParameters()):
    node = Node()
    request = EntityFactory()
    timeout = 5_000;

    positions = caluclate_buoy_field(origin, params)
    for (x, y) in tqdm(positions, desc="Publishing buoys"):
        name = f"buoy_red_{int(x)}_{int(y)}"
        sdf_string = generate_sdf_object(name, x, y)

        request.sdf = sdf_string
        ok, response = node.request(
            f"/world/{world}/create",
            request,
            EntityFactory,
            Boolean,
            timeout
        )
        time.sleep(0.1)

        if not ok or not response.data:
            print(f"Failed to spawn buoy at ({x}, {y})")

    print(f"Published {len(positions)} buoys to world '{world}'.")


def main():
    parser = argparse.ArgumentParser(description='Publish a field of buoys to a Gazebo world.')

    parser.add_argument('--world', type=str, default=WORLD,
                        help='The name of the world to publish the buoy field to.')
    parser.add_argument('--origin_x', type=float, default=-400.0,
                        help='The x coordinate of the center of the buoy field.')
    parser.add_argument('--origin_y', type=float, default=0.0,
                        help='The y coordinate of the center of the buoy field.')
    parser.add_argument('--line_length', type=float, default=750.0,
                        help='The length of each line of buoys in meters.')
    parser.add_argument('--buoy_spacing', type=float, default=25.0,
                        help='The spacing between buoys in meters.')
    parser.add_argument('--line_distance', type=float, default=12.0,
                        help='The distance between lines of buoys in meters.')
    parser.add_argument('--num_lines', type=int, default=5,
                        help='The number of lines of buoys.')

    args = parser.parse_args()
    params = FieldParameters(
        line_length=args.line_length,
        buoy_spacing=args.buoy_spacing,
        line_distance=args.line_distance,
        num_lines=args.num_lines
    )

    publish_buoy_field(args.world, Position(args.origin_x, args.origin_y), params)


if __name__ == '__main__':
    main()
