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
    create_principled_material,
    setup_material_properties,
    add_image_texture,
    apply_material_to_object,
)

_is_wind_gauge_initialized = False


def set_pointer_by_windspeed(speed):
    speed = max(0, min(40, speed))
    angle = math.radians((speed / 40) * 100)
    needle = bpy.data.objects.get("Body1")
    if needle is None:
        logger.error("needle not found")
        return

    needle.rotation_euler[1] = angle
    bpy.context.view_layer.update()
    logger.info(f"Needle position set to: {angle} degrees")
    return angle


def setup_wind_gauge_material():
    # create metallic material for gauge body
    body_material, body_principled = create_principled_material("BodyMaterial")
    setup_material_properties(body_principled)
    texture_path = os.path.join(
        os.path.dirname(__file__), "textures", "wind_gauge_texture.jpg"
    )
    add_image_texture(body_material, texture_path)
    # apply materials to objects
    apply_material_to_object("Body2", body_material)
    logger.success("Wind gauge material setup complete")
    return True


def set_camera_position(
    camera_name="Camera",
    target_name="Body1.001",
    angle_offset=0,
    distance=2.5,
    height=1.0,
):
    camera = bpy.data.objects.get(camera_name)
    if camera is None:
        logger.error(f"Camera not found: {camera_name}")
        return

    # Find pressure gauge object
    target = bpy.data.objects.get(target_name)

    if target is None:
        logger.error("Wind gauge not found")
        return

    angle_rad = math.radians(angle_offset)

    x_offset = distance * math.sin(angle_rad)
    y_offset = distance * math.cos(angle_rad)
    offset = mathutils.Vector((0, 0, 5))
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
    target = bpy.data.objects.get("Body1.001")

    if target is None:
        logger.error("Wind gauge not found")
        return

    cam = bpy.data.objects.get("Camera")
    if cam is None:
        logger.error("Camera not found")
        return

    angle = random.uniform(-10, 10)
    distance = random.uniform(15, 30)
    height = random.uniform(0, 10)

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
    global _is_wind_gauge_initialized
    if _is_wind_gauge_initialized:
        logger.info("Blender already initialized")
        return
    _is_wind_gauge_initialized = True

    blend_file_path = "generators/blend_files/7_Windgauge.blend"
    if not load_blend_file(blend_file_path):
        logger.error("Failed to load Blender file")
        raise Exception(f"Failed to load Blender file {blend_file_path}")
    setup_blender_context()

    setup_wind_gauge_material()


@registry.register(name="wind_gauge", tags={"wind_gauge"})
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

    speed = random.uniform(0, 40)
    logger.info(f"Wind speed: {speed}")

    set_pointer_by_windspeed(speed)
    render_from_multiple_angles()
    bpy.context.scene.render.filepath = os.path.abspath(img_path)
    bpy.ops.render.render(write_still=True)

    evaluator_kwargs = {
        "interval": [max(0, int(speed) - 2), min(int(speed) + 2, 40)],
        "units": ["mph"],
    }
    # print(evaluator_kwargs, theme)
    return Artifact(
        data=img_path,
        image_type="wind_gauge",
        design="Dial",
        evaluator="interval_matching",
        evaluator_kwargs=evaluator_kwargs,
    )


if __name__ == "__main__":
    res = generate("wind_gauge.png")
    print(res)
