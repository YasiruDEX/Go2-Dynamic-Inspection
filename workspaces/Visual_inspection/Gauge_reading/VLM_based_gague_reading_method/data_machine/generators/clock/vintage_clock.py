import bpy
import math
import mathutils
import os
from loguru import logger
import random
from generators.clock.utils import add_seconds_to_time_string
from artifacts import Artifact
from registry import registry
from generators.utils.blender_utils import setup_blender_context, load_blend_file

_is_clock4_initialized = False


def find_hand_object(keywords: list[str]) -> bpy.types.Object | None:
    """Find hand object by keywords"""
    for obj in bpy.data.objects:
        name_lower = obj.name.lower()
        for keyword in keywords:
            if keyword.lower() in name_lower:
                logger.success(f"Found hand object: {obj.name}")
                return obj
    return None


def find_clock_object() -> bpy.types.Object | None:
    """Find clock object by keywords"""
    clock_keywords = ["clock", "alarm", "钟", "表", "wall"]
    for obj in bpy.data.objects:
        name_lower = obj.name.lower()
        for keyword in clock_keywords:
            if keyword.lower() in name_lower:
                logger.success(f"Found clock object: {obj.name}")
                return obj
    return None


def set_clock_time(target_hour, target_minute, target_second):
    second_angle = math.radians(target_second * 6)  # 6 degrees per second
    minute_angle = math.radians(
        target_minute * 6 + target_second * 0.1
    )  # 6 degrees per minute + seconds influence
    hour_angle = math.radians(
        (target_hour % 12) * 30 + target_minute * 0.5
    )  # 30 degrees per hour + minute influence

    # Find hand objects
    hour_hand = find_hand_object(["hour", "时针", "hour_hand"])
    minute_hand = find_hand_object(["minute", "分针", "minute_hand"])
    second_hand = find_hand_object(["second", "秒针", "second_hand"])

    if hour_hand is None:
        logger.error("Hour hand not found")
        for obj in bpy.data.objects:
            print(f"  - {obj.name}")
        return None

    if minute_hand is None:
        logger.error("Minute hand not found")
        return None

    if second_hand is None:
        logger.error("Second hand not found")
        return None

    hour_hand.rotation_euler = (0, hour_angle, 0)
    minute_hand.rotation_euler = (0, minute_angle, 0)
    second_hand.rotation_euler = (0, second_angle, 0)

    logger.info(
        f"Time set to: {target_hour:02d}:{target_minute:02d}:{target_second:02d}"
    )

    return (target_hour, target_minute, target_second)


def setup_lighting():
    # Clear existing lights
    for obj in bpy.data.objects:
        if obj.type == "LIGHT":
            bpy.data.objects.remove(obj, do_unlink=True)

    def create_light(name, location, energy=1000):
        light_data = bpy.data.lights.new(name=name, type="AREA")
        light_data.energy = energy
        light_obj = bpy.data.objects.new(name=name, object_data=light_data)
        bpy.context.collection.objects.link(light_obj)
        light_obj.location = location
        return light_obj

    create_light("KeyLight", location=(5, -3, 6), energy=1500)
    create_light("FillLight", location=(-4, 2, 4), energy=1000)
    create_light("BackLight", location=(0, -6, 5), energy=500)
    logger.success("Lighting setup complete")


def set_camera_position(
    camera_name="Camera", target_name=None, angle_offset=0, distance=2.5, height=1.0
):
    camera = bpy.data.objects.get(camera_name)
    if camera is None:
        logger.error(f"Camera not found: {camera_name}")
        return

    # Find clock object
    if target_name is None:
        target = find_clock_object()
    else:
        target = bpy.data.objects.get(target_name)

    if target is None:
        logger.error("Clock object not found")
        return

    angle_rad = math.radians(angle_offset)

    # Calculate camera position
    x_offset = distance * math.sin(angle_rad)
    y_offset = -distance * math.cos(angle_rad)
    offset = mathutils.Vector((0, 0, 0.2))
    look_at = target.location + offset

    new_position = mathutils.Vector(
        (look_at.x + x_offset, look_at.y + y_offset, look_at.z + height)
    )
    camera.location = new_position

    direction = look_at - camera.location
    direction.normalize()

    rot_quat = direction.to_track_quat("-Z", "Y")
    camera.rotation_euler = rot_quat.to_euler()

    logger.info(f"Camera position set to: {camera.location}")


def render_from_multiple_angles():
    target = find_clock_object()

    if target is None:
        logger.error("Target object not found")
        return

    cam = bpy.data.objects.get("Camera")
    if cam is None:
        logger.error("Camera not found")
        return

    angle = random.uniform(-10, 10)
    distance = random.uniform(1.8, 2.8)
    height = random.uniform(-1.0, 1.0)

    set_camera_position(
        angle_offset=angle,
        distance=distance,
        height=height,
    )
    logger.info(
        f"Random camera params: angle={angle:.2f}, distance={distance:.2f}, height={height:.2f}"
    )


def init_blender():
    global _is_clock4_initialized
    if _is_clock4_initialized:
        logger.info("Blender already initialized")
        return
    _is_clock4_initialized = True

    blend_file_path = "generators/blend_files/clock.blend"
    if not load_blend_file(blend_file_path):
        logger.error("Failed to load Blender file")
        raise Exception(f"Failed to load Blender file {blend_file_path}")
    setup_blender_context()
    setup_lighting()


@registry.register(name="vintage_clock", tags={"clock"})
def generate(img_path="clock.png") -> Artifact:
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

    set_clock_time(hour, minute, second)
    render_from_multiple_angles()
    bpy.context.scene.render.filepath = os.path.abspath(img_path)
    bpy.ops.render.render(write_still=True)
    time_minus_1s = add_seconds_to_time_string(time_str, -1)
    time_plus_1s = add_seconds_to_time_string(time_str, 1)

    evaluator_kwargs = {"interval": [time_minus_1s, time_plus_1s], "units": []}
    # print(evaluator_kwargs, theme)
    return Artifact(
        data=img_path,
        image_type="clock",
        design="Dial",
        evaluator_kwargs=evaluator_kwargs,
    )


if __name__ == "__main__":
    res = generate("clock2.jpg")
    print(res)
