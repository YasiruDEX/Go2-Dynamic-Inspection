import bpy
import math
import mathutils
import os
from loguru import logger
import random
from generators.watch.utils import add_seconds_to_time_string
from artifacts import Artifact
from registry import registry
from generators.utils.blender_utils import (
    setup_blender_context,
    load_blend_file,
    get_available_exr_files,
)

_is_watch_initialized = False


def set_clock_time(target_hour, target_minute, target_second):
    second_angle = math.radians(target_second * 6)  # 6 degrees per second
    minute_angle = math.radians(
        target_minute * 6 + target_second * 0.1
    )  # 6 degrees per minute and seconds impact
    hour_angle = math.radians(
        (target_hour % 12) * 30 + target_minute * 0.5
    )  # 30 degrees per hour + minute influence

    # Find hand objects
    hour_hand = bpy.data.objects.get("hour")
    minute_hand = bpy.data.objects.get("minute")
    second_hand = bpy.data.objects.get("second")

    if hour_hand is None:
        logger.error("Hour hand not found")
        return None

    if minute_hand is None:
        logger.error("Minute hand not found")
        return None

    if second_hand is None:
        logger.error("Second hand not found")
        return None

    hour_hand.rotation_euler = (0, 0, -hour_angle)
    minute_hand.rotation_euler = (0, 0, -minute_angle)
    second_hand.rotation_euler = (0, 0, -second_angle)

    logger.info(
        f"Time set to: {target_hour:02d}:{target_minute:02d}:{target_second:02d}"
    )

    return (target_hour, target_minute, target_second)


def setup_env_lighting(exr_path):
    """Setup environment lighting"""
    world = bpy.context.scene.world
    if world is None:
        world = bpy.data.worlds.new("World")
        bpy.context.scene.world = world

    # enable nodes
    world.use_nodes = True
    nodes = world.node_tree.nodes

    # clear old nodes and add new background node
    nodes.clear()
    background_node = nodes.new(type="ShaderNodeBackground")
    output_node = nodes.new(type="ShaderNodeOutputWorld")

    # add environment texture node
    environment_texture_node = nodes.new(type="ShaderNodeTexEnvironment")
    exr_path = os.path.abspath(exr_path)
    environment_texture_node.image = bpy.data.images.load(exr_path)

    # link the nodes
    links = world.node_tree.links
    links.new(
        environment_texture_node.outputs["Color"], background_node.inputs["Color"]
    )
    links.new(background_node.outputs["Background"], output_node.inputs["Surface"])
    logger.success(f"Environment lighting setup complete: {os.path.basename(exr_path)}")


def set_camera_position(
    camera_name="Camera", target_name="watch", angle_offset=0, distance=2.5, height=1.0
):
    camera = bpy.data.objects.get(camera_name)
    if camera is None:
        logger.error(f"Camera not found: {camera_name}")
        return

    # Find watch object
    target = bpy.data.objects.get(target_name)

    if target is None:
        logger.error("Watch object not found")
        return

    angle_rad = math.radians(angle_offset)

    # Calculate camera position
    x_offset = distance * math.sin(angle_rad)
    y_offset = -distance * math.cos(angle_rad)

    new_position = mathutils.Vector(
        (
            target.location.x + x_offset,
            target.location.y + y_offset,
            target.location.z + height,
        )
    )
    camera.location = new_position

    direction = target.location - camera.location
    direction.normalize()

    rot_quat = direction.to_track_quat("-Z", "Y")
    camera.rotation_euler = rot_quat.to_euler()

    logger.info(f"Camera position set to: {camera.location}")


def render_from_multiple_angles():
    target = bpy.data.objects.get("watch")
    if target is None:
        logger.error("Target object not found")
        return

    cam = bpy.data.objects.get("Camera")
    if cam is None:
        logger.error("Camera not found")
        return

    angle = random.uniform(-12, 12)
    distance = random.uniform(0.3, 0.7)
    height = random.uniform(0.5, 1.1)

    set_camera_position(
        angle_offset=angle,
        distance=distance,
        height=height,
    )
    logger.info(
        f"Random camera params: angle={angle:.2f}, distance={distance:.2f}, height={height:.2f}"
    )


def init_blender():
    global _is_watch_initialized
    if _is_watch_initialized:
        logger.info("Blender already initialized")
        return
    _is_watch_initialized = True

    blend_file_path = "generators/blend_files/3_watch.blend"
    if not load_blend_file(blend_file_path):
        logger.error("Failed to load Blender file")
        raise Exception(f"Failed to load Blender file {blend_file_path}")
    setup_blender_context()


@registry.register(name="watch", tags={"watch"})
def generate(img_path="watch.png") -> Artifact:
    init_blender()
    ext = img_path.split(".")[-1]
    if ext in ["jpg", "jpeg"]:
        bpy.context.scene.render.image_settings.file_format = "JPEG"
    else:
        bpy.context.scene.render.image_settings.file_format = "PNG"
    hour, minute, second = (
        random.randint(0, 12),
        random.randint(0, 59),
        random.randint(0, 59),
    )
    time_str = f"{hour:02d}:{minute:02d}:{second:02d}"

    random_exr = random.choice(
        get_available_exr_files("generators/blend_files/exr_files")
    )
    if random_exr:
        setup_env_lighting(random_exr)

    set_clock_time(hour, minute, second)
    render_from_multiple_angles()
    bpy.context.scene.render.filepath = os.path.abspath(img_path)
    bpy.ops.render.render(write_still=True)
    time_minus_60s = add_seconds_to_time_string(time_str, -60)
    time_plus_60s = add_seconds_to_time_string(time_str, 60)

    evaluator_kwargs = {"interval": [time_minus_60s, time_plus_60s], "units": []}
    # print(evaluator_kwargs, theme)
    return Artifact(
        data=img_path,
        image_type="clock",
        design="Dial",
        evaluator_kwargs=evaluator_kwargs,
    )


if __name__ == "__main__":
    res = generate("watch.png")
    print(res)
