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

_is_ruler_initialized = False


def find_ruler_object() -> bpy.types.Object | None:
    """Find ruler object by keywords"""
    ruler_keywords = ["rule"]
    for obj in bpy.data.objects:
        name_lower = obj.name.lower()
        for keyword in ruler_keywords:
            if keyword.lower() in name_lower:
                logger.success(f"Found ruler object: {obj.name}")
                return obj
    return None


def find_pen_object() -> bpy.types.Object | None:
    """Find pen object by keywords"""
    pen_keywords = ["pen"]
    for obj in bpy.data.objects:
        name_lower = obj.name.lower()
        for keyword in pen_keywords:
            if keyword.lower() in name_lower:
                logger.success(f"Found pen object: {obj.name}")
                return obj
    return None


def set_pen_position(x_position=0, origin=0):
    pen = find_pen_object()
    if pen is None:
        logger.error("Pen object not found")
        return
    pen.location[0] = origin + x_position
    logger.info(f"Pen position set to: {x_position}")
    return x_position


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

    # Find ruler object
    target = find_ruler_object()
    if target is None:
        logger.error("Ruler object not found")
        return
    logger.info(f"target: {target.location}")
    angle_rad = math.radians(angle_offset)

    x_offset = distance * math.sin(angle_rad)
    offest = mathutils.Vector((-0.15, 0, 0))
    look_at = target.location + offest
    new_position = mathutils.Vector(
        (look_at.x + x_offset, look_at.y - height, look_at.z)
    )
    camera.location = new_position

    direction = look_at - camera.location
    direction.normalize()
    rot_quat = direction.to_track_quat("-Z", "Y")
    camera.rotation_euler = rot_quat.to_euler()

    logger.info(f"Camera position set to: {camera.location}")


def render_from_multiple_angles():
    target = find_ruler_object()
    if target is None:
        logger.error("Target object not found")
        return

    cam = bpy.data.objects.get("Camera")
    if cam is None:
        logger.error("Camera not found")
        return

    angle = random.uniform(-8, 8)
    distance = random.uniform(0.1, 0.2)
    height = random.uniform(0.6, 0.9)

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
    global _is_ruler_initialized
    if _is_ruler_initialized:
        logger.info("Blender already initialized")
        return
    _is_ruler_initialized = True

    blend_file_path = "generators/blend_files/5_ruler.blend"
    if not load_blend_file(blend_file_path):
        logger.error("Failed to load Blender file")
        raise Exception(f"Failed to load Blender file {blend_file_path}")
    setup_blender_context()


@registry.register(name="ruler", tags={"ruler"})
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
    pen = find_pen_object()
    if pen is None:
        logger.error("Pen object not found")
        return
    pen.location = mathutils.Vector((-0.1513, 0.0000, -0.0257))
    origin = pen.location[0]
    num = random.uniform(-0.05, 0.05)
    set_pen_position(num, origin)
    logger.info(f"Delta pen position set to: {num}")
    render_from_multiple_angles()
    bpy.context.scene.render.filepath = os.path.abspath(img_path)
    bpy.ops.render.render(write_still=True)

    evaluator_kwargs = {
        "interval": [16.00, 16.75],
        "units": ["cm", "Centimeter"],
    }
    # print(evaluator_kwargs, theme)
    ruler_questions = [
        "The ruler is graduated in centimeters. Based on the reading, how long is this pen?",
        "What is the length of the pen shown on the ruler?",
        "How long is the pen according to the ruler?",
        "What length does the ruler show for this pen?",
        "Based on the ruler measurement, what is the length of the pen?",
        "The ruler shows the length of a pen. What is that length?",
        "What is the measured length of the pen using the ruler?",
        "What does the ruler indicate as the length of this pen?",
        "The ruler is measuring a pen. What is its length?",
    ]

    # 随机选择一个问题
    custom_question = random.choice(ruler_questions)

    return Artifact(
        data=img_path,
        image_type="ruler",
        design="Linear",
        evaluator_kwargs=evaluator_kwargs,
        question=custom_question,
    )


if __name__ == "__main__":
    res = generate("ruler.png")
    print(res)
