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
    apply_texture_to_existing_object,
)

_is_measuring_cylinder_initialized = False


def adjust_liquid_height(volume):
    default_volume = 69
    cylinder = bpy.data.objects.get("Liquid_Cylinder")
    if cylinder is None:
        logger.error("cylinder not found")
        return
    cylinder.scale[2] = volume / default_volume
    bpy.context.view_layer.update()
    logger.info(f"Liquid height set to: {volume} ml")
    logger.info(f"Liquid scale set to: {cylinder.scale[2]}")
    return volume


def setup_measuring_cylinder_material():
    # create metallic material for gauge body
    img_path = os.path.join(os.path.dirname(__file__), "textures", "lab_cylinder_a.png")
    img_path_2 = os.path.join(os.path.dirname(__file__), "textures", "lab_beaker_a.png")
    apply_texture_to_existing_object("lab_cylinder_a", img_path, "lab_cylinder_a.png")
    apply_texture_to_existing_object("lab_beaker_a", img_path_2, "lab_beaker_a.png")

    logger.success("Measuring cylinder material setup complete")
    return True


def set_camera_position(
    camera_name="Camera",
    target_name="lab_cylinder_a",
    angle_offset=0,
    distance=2.5,
    height=1.0,
):
    camera = bpy.data.objects.get(camera_name)
    if camera is None:
        logger.error(f"Camera not found: {camera_name}")
        return

    # Find measuring cylinder object
    target = bpy.data.objects.get(target_name)

    if target is None:
        logger.error("Measuring cylinder not found")
        return

    angle_rad = math.radians(angle_offset)

    x_offset = distance * math.sin(angle_rad)
    y_offset = distance * math.cos(angle_rad)
    offset = mathutils.Vector((0, -15, 9))
    look_at = target.location + offset

    new_position = mathutils.Vector(
        (look_at.x + x_offset, look_at.y - y_offset, look_at.z + height)
    )

    camera.location = new_position

    direction = look_at - camera.location
    direction.normalize()

    rot_quat = direction.to_track_quat("-Z", "Y")
    camera.rotation_euler = rot_quat.to_euler()

    logger.info(f"Camera position set to: {camera.location}")


def render_from_multiple_angles():
    target = bpy.data.objects.get("lab_cylinder_a")

    if target is None:
        logger.error("Measuring cylinder not found")
        return

    cam = bpy.data.objects.get("Camera")
    if cam is None:
        logger.error("Camera not found")
        return

    angle = random.uniform(-10, 10)
    distance = random.uniform(22, 30)
    height = random.uniform(8, 10)

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
    global _is_measuring_cylinder_initialized
    if _is_measuring_cylinder_initialized:
        logger.info("Blender already initialized")
        return
    _is_measuring_cylinder_initialized = True

    blend_file_path = "generators/blend_files/8_cylinder_old.blend"
    if not load_blend_file(blend_file_path):
        logger.error("Failed to load Blender file")
        raise Exception(f"Failed to load Blender file {blend_file_path}")
    setup_blender_context()

    setup_measuring_cylinder_material()


@registry.register(name="blender_measuring_cylinder", tags={"measuring_cylinder"})
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

    num = random.uniform(60, 90)
    logger.info(f"Liquid height: {num}")

    adjust_liquid_height(num)
    render_from_multiple_angles()
    bpy.context.scene.render.filepath = os.path.abspath(img_path)
    bpy.ops.render.render(write_still=True)

    evaluator_kwargs = {
        "interval": [max(0, int(num) - 1), min(int(num) + 1, 100)],
        "units": ["ml"],
    }
    # print(evaluator_kwargs, theme)
    return Artifact(
        data=img_path,
        image_type="measuring_cylinder",
        design="Dial",
        evaluator="interval_matching",
        evaluator_kwargs=evaluator_kwargs,
    )


if __name__ == "__main__":
    res = generate("measuring_cylinder.png")
    print(res)
