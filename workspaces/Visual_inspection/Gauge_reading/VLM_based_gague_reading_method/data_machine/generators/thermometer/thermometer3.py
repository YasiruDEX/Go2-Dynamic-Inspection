import bpy
import math
import mathutils
import os
from loguru import logger
import random
from artifacts import Artifact
from registry import registry

from generators.utils.blender_utils import (
    setup_blender_context,
    load_blend_file,
    get_available_exr_files,
)

_is_thermometer_3_initialized = False


def find_thermometer_object() -> bpy.types.Object | None:
    """Find thermometer object by keywords"""
    thermometer_keywords = ["thermometer"]
    for obj in bpy.data.objects:
        name_lower = obj.name.lower()
        for keyword in thermometer_keywords:
            if keyword.lower() in name_lower:
                logger.success(f"Found thermometer object: {obj.name}")
                return obj
    return None


def set_temperature(
    temperature, min_temp=-30, max_temp=50, min_scale=0.16, max_scale=1.39
):
    scale_y = min_scale + (temperature - min_temp) / (max_temp - min_temp) * (
        max_scale - min_scale
    )
    tube = bpy.data.objects.get("ThermoTube")
    if tube:
        tube.scale[1] = scale_y
        logger.info(f"ThermoTube position set to: {scale_y}")
    else:
        logger.error("ThermoTube not found")
    return scale_y


def set_camera_position(
    camera_name="Camera",
    target_name=None,
    angle_offset=0,
    distance=2.5,
    height=1.0,
):
    camera = bpy.data.objects.get(camera_name)
    if camera is None:
        logger.error(f"Camera not found: {camera_name}")
        return

    # Find thermometer object
    target = find_thermometer_object()
    if target is None:
        logger.error("Thermometer object not found")
        return
    logger.info(f"target: {target.location}")
    angle_rad = math.radians(angle_offset)

    x_offset = distance * math.cos(angle_rad)
    y_offset = distance * math.sin(angle_rad)
    offset = mathutils.Vector((0, 0.05, 0))
    look_at = target.location + offset

    new_position = mathutils.Vector(
        (look_at.x + x_offset, look_at.y + y_offset, look_at.z + height)
    )
    camera.location = new_position

    direction = look_at - camera.location
    direction.normalize()
    rot_quat = direction.to_track_quat("-Z", "Z")
    camera.rotation_euler = rot_quat.to_euler()

    logger.info(f"Camera position set to: {camera.location}")


def render_from_multiple_angles():
    target = find_thermometer_object()
    if target is None:
        logger.error("Target object not found")
        return

    cam = bpy.data.objects.get("Camera")
    if cam is None:
        logger.error("Camera not found")
        return

    angle = random.uniform(-10, 10)
    distance = random.uniform(0, 0.15)
    height = random.uniform(0.3, 0.5)

    set_camera_position(
        angle_offset=angle,
        distance=distance,
        height=height,
    )
    logger.info(
        f"Random camera params: angle={angle:.2f}, distance={distance:.2f}, height={height:.2f}"
    )


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


def init_blender():
    global _is_thermometer_3_initialized
    if _is_thermometer_3_initialized:
        logger.info("Blender already initialized")
        return
    _is_thermometer_3_initialized = True

    blend_file_path = "generators/blend_files/2_thermometer_4_hygometer.blend"
    if not load_blend_file(blend_file_path):
        logger.error("Failed to load Blender file")
        raise Exception(f"Failed to load Blender file {blend_file_path}")
    setup_blender_context()


@registry.register(name="thermometer_3", tags={"thermometer"})
def generate(img_path: str) -> Artifact:
    init_blender()
    ext = img_path.split(".")[-1]
    if ext in ["jpg", "jpeg"]:
        bpy.context.scene.render.image_settings.file_format = "JPEG"
    else:
        bpy.context.scene.render.image_settings.file_format = "PNG"
    random_exr = random.choice(
        get_available_exr_files("generators/blend_files/exr_files")
    )
    if random_exr:
        setup_env_lighting(random_exr)

    num = random.uniform(-30, 50)
    set_temperature(num)
    logger.info(f"Temperature set to: {num}")
    render_from_multiple_angles()
    bpy.context.scene.render.filepath = os.path.abspath(img_path)
    bpy.ops.render.render(write_still=True)

    evaluator_kwargs = {
        "intervals": [
            [max(-30, num - 1), min(num + 1, 50)],
            [max(-20, int((num - 1) * 1.8 + 32)), min(int((num + 1) * 1.8 + 32), 120)],
        ],
        "units": [["Celsius", "°C"], ["fahrenheit", "°F"]],
    }

    return Artifact(
        data=img_path,
        image_type="thermometer",
        design="Linear",
        evaluator="multi_interval_matching",
        evaluator_kwargs=evaluator_kwargs,
    )


if __name__ == "__main__":
    res = generate("thermometer_3.png")
    print(res)
