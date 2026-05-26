import bpy
import os
import glob
from pathlib import Path
from loguru import logger


_DATA_MACHINE_ROOT = Path(__file__).resolve().parents[2]


def resolve_path(path_like):
    """Resolve resource paths relative to the data_machine package."""
    path = Path(path_like)
    if not path.is_absolute():
        path = _DATA_MACHINE_ROOT / path
    return path


def setup_blender_context(
    image_height=720, image_width=1280, resolution_percentage=100, clear=False
):
    """
    Initialize Blender context
    Args:
        image_height: int, the height of the image
        image_width: int, the width of the image
        resolution_percentage: int, the percentage of the resolution
    """
    # Clear default scene
    if clear:
        bpy.ops.wm.read_homefile(app_template="")

    # Set rendering settings
    scene = bpy.context.scene
    scene.render.resolution_x = image_width
    scene.render.resolution_y = image_height
    scene.render.resolution_percentage = resolution_percentage

    print("Blender context initialized")


def load_blend_file(filepath):
    resolved_path = resolve_path(filepath)
    if not resolved_path.exists():
        logger.error(f"File not found: {resolved_path}")
        return False

    try:
        bpy.ops.wm.open_mainfile(filepath=str(resolved_path))
        logger.success(f"File loaded: {resolved_path}")
        return True
    except Exception as e:
        logger.error(f"File load failed: {e}")
        return False


def get_available_exr_files(base_path):
    """Get available EXR files in the base path"""
    base_dir = resolve_path(base_path)
    exr_pattern = str(base_dir / "*.exr")
    exr_files = glob.glob(exr_pattern)

    if not exr_files:
        logger.error(f"No EXR files found in {base_dir}")

    return exr_files


def create_principled_material(name="CustomMaterial"):
    material = bpy.data.materials.new(name=name)
    material.use_nodes = True

    # Get the material's node tree
    nodes = material.node_tree.nodes
    links = material.node_tree.links

    # clear existing nodes
    nodes.clear()

    # add principled bsdf node
    principled = nodes.new(type="ShaderNodeBsdfPrincipled")
    principled.location = (0, 0)

    # add material output node
    output = nodes.new(type="ShaderNodeOutputMaterial")
    output.location = (400, 0)

    # link principled to output
    links.new(principled.outputs["BSDF"], output.inputs["Surface"])

    return material, principled


def setup_material_properties(principled_node):
    principled_node.inputs["Base Color"].default_value = (1, 1, 0, 1)
    principled_node.inputs["Metallic"].default_value = 1.0
    principled_node.inputs["Roughness"].default_value = 0.553
    principled_node.inputs["IOR"].default_value = 1.450
    principled_node.inputs["Alpha"].default_value = 1.0


def add_image_texture(material, image_path=None, texture_node_name="ImageTexture"):
    nodes = material.node_tree.nodes
    links = material.node_tree.links

    principled = None
    for node in nodes:
        if node.type == "BSDF_PRINCIPLED":
            principled = node
            break

    if not principled:
        logger.error("Principled BSDF node not found")
        return

    tex_image = nodes.new(type="ShaderNodeTexImage")
    tex_image.name = texture_node_name
    tex_image.location = (-300, 0)

    if image_path:
        try:
            image = bpy.data.images.load(image_path)
            tex_image.image = image
            logger.info(f"Loaded image texture: {image_path}")
        except Exception as e:
            logger.error(f"Failed to load image texture: {image_path} - {e}")
            return None

    # links texture to base color
    links.new(tex_image.outputs["Color"], principled.inputs["Base Color"])
    return tex_image


def apply_material_to_object(obj_name, material):
    obj = bpy.data.objects.get(obj_name)
    if not obj:
        logger.error(f"Object not found: {obj_name}")
        return False

    if obj.type != "MESH":
        logger.error(f"Object is not a mesh: {obj_name}")
        return False

    # clear existing materials and add new one
    obj.data.materials.clear()
    obj.data.materials.append(material)

    logger.info(f"Material applied to {obj_name}")
    return True


def apply_texture_to_existing_object(obj_name, img_path, img_filename):
    # Get the object
    obj = bpy.data.objects.get(obj_name)
    if not obj:
        logger.error(f"Object '{obj_name}' not found.")
        return False

    # Ensure the object has at least one material
    if not obj.data.materials:
        logger.error(f"Object '{obj_name}' has no material.")
        return False
    mat = obj.data.materials[0]  # use the first material

    # Ensure the material uses nodes
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links

    # Create an Image Texture node
    tex_node = nodes.new(type="ShaderNodeTexImage")
    tex_node.location = (-300, 300)

    # Load the image
    if not os.path.exists(img_path):
        logger.error(f"Image file not exists: {img_path}")
        return False

    img = bpy.data.images.load(img_path)
    tex_node.image = img

    # Find Principled BSDF node
    principled = None
    for node in nodes:
        if node.type == "BSDF_PRINCIPLED":
            principled = node
            break
    if not principled:
        logger.error(f"No Principled BSDF found in '{obj_name}' material.")
        return False

    # Connect Image Texture color to Base Color
    links.new(tex_node.outputs["Color"], principled.inputs["Base Color"])
    logger.success(f"Applied '{img_filename}' to '{obj_name}'")
    return True
