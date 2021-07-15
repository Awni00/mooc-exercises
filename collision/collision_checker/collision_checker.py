import itertools
import random
from typing import List

from aido_schemas import Context, FriendlyPose
from dt_protocols import (
    Circle,
    CollisionCheckQuery,
    CollisionCheckResult,
    MapDefinition,
    PlacedPrimitive,
    Rectangle,
)

import shapely.geometry
import shapely.affinity


__all__ = ["CollisionChecker"]


class CollisionChecker:
    params: MapDefinition

    def init(self, context: Context):
        context.info("init()")

    def on_received_set_params(self, context: Context, data: MapDefinition):
        context.info("initialized")
        self.params = data

    def on_received_query(self, context: Context, data: CollisionCheckQuery):
        collided = check_collision(
            Wcoll=self.params.environment, robot_body=self.params.body, robot_pose=data.pose
        )
        result = CollisionCheckResult(collided)
        context.write("response", result)


def pose_distance(pose1: FriendlyPose, pose2: FriendlyPose):
    x1, y1, x2, y2 = pose1.x, pose1.y, pose2.x, pose2.y
    return ((x1-x2)**2 + (y1-y2)**2)**(1/2)

def get_shapely_rect(pprim: PlacedPrimitive):
    rect = pprim.primitive
    
    origin = (pprim.pose.x, pprim.pose.y)

    # shapely_rect = shapely.geometry.box(rect.xmin, rect.ymin, rect.xmax, rect.ymax)
    # shapely_rect = shapely.affinity.translate(shapely_rect, xoff=pprim.pose.x, yoff=pprim.pose.y) # stackoverflow suggestion
    # shapely_rect = shapely.affinity.rotate(shapely_rect, pprim.pose.theta_deg, origin=origin)

    shapely_rect = shapely.geometry.box(rect.xmin, rect.ymin, rect.xmax, rect.ymax)
    shapely_rect = shapely.affinity.rotate(shapely_rect, pprim.pose.theta_deg, origin=(0,0))
    shapely_rect = shapely.affinity.translate(shapely_rect, xoff=pprim.pose.x, yoff=pprim.pose.y) # stackoverflow suggestion
    
    return shapely_rect

def get_shapely_circ(pprim: PlacedPrimitive):
    r = pprim.primitive.radius
    x = pprim.pose.x
    y = pprim.pose.y
    
    shapely_circ = shapely.geometry.Point(x, y).buffer(r)
    return shapely_circ

def rec_rec_collision(pprim1: PlacedPrimitive, pprim2: PlacedPrimitive) -> bool:

    shapely_rect1 = get_shapely_rect(pprim1)
    shapely_rect2 = get_shapely_rect(pprim2)
    
    aoi = shapely_rect1.intersection(shapely_rect2).area
    # print(aoi)
    
    return aoi > 0

def rec_circ_collision(pprim_rec: PlacedPrimitive, pprim_circ) -> bool:
    shapely_rect = get_shapely_rect(pprim_rec)
    shapely_circ = get_shapely_circ(pprim_circ)
    
    aoi = shapely_rect.intersection(shapely_circ).area
    # print(aoi)
    
    return aoi > 0

def circ_circ_collision(pprim_circ1, pprim_circ2):
    pose1 = pprim_circ1.pose
    pose2 = pprim_circ2.pose
    d = pprim_circ1.primitive.radius + pprim_circ2.primitive.radius
    
    return pose_distance(pose1, pose2) < d

def rototranslate_rb(rb, robot_pose):
    rb.pose = robot_pose
    return rb

def check_collision(
    Wcoll: List[PlacedPrimitive], robot_body: List[PlacedPrimitive], robot_pose: FriendlyPose
) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly

    rototranslated_robot: List[PlacedPrimitive] = [rototranslate_rb(rb, robot_pose) for rb in robot_body]

    collided = check_collision_list(rototranslated_robot, Wcoll)

    return collided


def check_collision_list(A: List[PlacedPrimitive], B: List[PlacedPrimitive]) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly

    for a, b in itertools.product(A, B):
        if check_collision_shape(a, b):
            return True

    return False


def check_collision_shape(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:    
    
    if isinstance(a.primitive, Circle) and isinstance(b.primitive, Circle):
        return circ_circ_collision(a, b)
    if isinstance(a.primitive, Circle) and isinstance(b.primitive, Rectangle):
        return rec_circ_collision(b, a)
    if isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Circle):
        return rec_circ_collision(a, b)
    if isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Rectangle):
        return rec_rec_collision(a, b) 
    
    print('None of the checks passed')
    
    return None