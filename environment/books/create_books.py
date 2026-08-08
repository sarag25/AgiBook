"""
Script for Blender 5.1.2 to create books with texture (cover, retro and spine) and export in the .glb format
"""

import bpy      # import Blender Python API
import bmesh    # import Blender Python API
import os

def script_dir():
    """
    Directory of the script currently open in Blender's text editor.
    Only valid when this file is the entry script: when imported from
    another script (e.g. create_scene.py), pass images_dir explicitly.
    """
    return os.path.dirname(os.path.abspath(bpy.context.space_data.text.filepath))

BOOKS = [
    {
        "name":  "it_book",
        "size":  (0.155, 0.055, 0.235),
        "cover": "stephen_king/it_cover.jpg",
        "retro": "stephen_king/it_retro.jpg",
        "spine": "stephen_king/it_spine.jpg",
    },
    {
        "name":  "hunger_games_book",
        "size":  (0.200, 0.070, 0.205),
        "cover": "hunger_games/hunger_games_trilogia_cover.jpg",
        "retro": "hunger_games/hunger_games_trilogia_retro.jpg",
        "spine": "hunger_games/hunger_games_trilogia_spine.jpg",
    },
    {
        "name":  "werther_book",
        "size":  (0.120, 0.010, 0.190),
        "cover": "werther/werther_cover.jpg",
        "retro": "werther/werther_retro.jpg",
        "spine": "werther/werther_spine.jpg",
    },
    {
        "name":  "never_flinch_book",
        "size":  (0.155, 0.039, 0.230),
        "cover": "stephen_king/lotteria_innocenti_cover.jpg",
        "retro": "stephen_king/lotteria_innocenti_retro.jpg",
        "spine": "stephen_king/lotteria_innocenti_spine.jpg",
    },
    {
        "name":  "ballata_usignolo_book",
        "size":  (0.150, 0.045, 0.210),
        "cover": "hunger_games/hunger_games_ballata_cover.jpg",
        "retro": "hunger_games/hunger_games_ballata_retro.jpg",
        "spine": "hunger_games/hunger_games_ballata_spine.jpg",
    },
    {
        "name":  "alba_mietitura_book",
        "size":  (0.150, 0.038, 0.210),
        "cover": "hunger_games/hunger_games_mietitura_cover.jpg",
        "retro": "hunger_games/hunger_games_mietitura_retro.jpg",
        "spine": "hunger_games/hunger_games_mietitura_spine.jpg",
    },
    {
        "name":  "black_widow_book",
        "size":  (0.170, 0.023, 0.260),
        "cover": "comics/black_widow_cover.jpg",
        "retro": "comics/black_widow_retro.jpg",
        "spine": "comics/black_widow_spine.jpg",
    },
    {
        "name":  "fantasticos_4_book",
        "size":  (0.170, 0.022, 0.260),
        "cover": "comics/fantasticos_4_cover.jpg",
        "retro": "comics/fantasticos_4_retro.jpg",
        "spine": "comics/fantasticos_4_spine.jpg",
    },
    {
        "name":  "enciclopedia_animali_book",
        "size":  (0.230, 0.029, 0.280),
        "cover": "enciclopedia/enciclopedia_animali_cover.jpg",
        "retro": "enciclopedia/enciclopedia_animali_retro.jpg",
        "spine": "enciclopedia/enciclopedia_animali_spine.jpg",
    },
    {
        "name":  "enciclopedia_terra_vol1_book",
        "size":  (0.230, 0.027, 0.280),
        "cover": "enciclopedia/enciclopedia_terra_vol1_cover.jpg",
        "retro": "enciclopedia/enciclopedia_terra_vol1_retro.jpg",
        "spine": "enciclopedia/enciclopedia_terra_vol1_spine.jpg",
    },
    {
        "name":  "enciclopedia_terra_vol2_book",
        "size":  (0.230, 0.027, 0.280),
        "cover": "enciclopedia/enciclopedia_terra_vol2_cover.jpg",
        "retro": "enciclopedia/enciclopedia_terra_vol2_retro.jpg",
        "spine": "enciclopedia/enciclopedia_terra_vol2_spine.jpg",
    },
    {
        "name":  "cane_stelle_book",
        "size":  (0.148, 0.017, 0.210),
        "cover": "manga/cane_manga_cover.jpg",
        "retro": "manga/cane_manga_retro.jpg",
        "spine": "manga/cane_manga_spine.jpg",
        "manga": True,
    },
    {
        "name":  "cane_stelle_racconti_book",
        "size":  (0.148, 0.019, 0.210),
        "cover": "manga/cane_racconti_manga_cover.jpg",
        "retro": "manga/cane_racconti_manga_retro.jpg",
        "spine": "manga/cane_racconti_manga_spine.jpg",
        "manga": True,
    },
    {
        "name":  "emma_book",
        "size":  (0.110, 0.019, 0.180),
        "cover": "inglese/emma_cover.jpg",
        "retro": "inglese/emma_retro.jpg",
        "spine": "inglese/emma_spine.jpg",
    },
    {
        "name":  "cats_cradle_book",
        "size":  (0.133, 0.015, 0.203),
        "cover": "inglese/cat's_cradle_cover.jpg",
        "retro": "inglese/cat's_cradle_retro.jpg",
        "spine": "inglese/cat's_cradle_spine.jpg",
    },
]


def load_image(filename, images_dir):
    """
    Load an image from images_dir and return a Blender image object.
    """
    path = os.path.join(images_dir, filename)
    img = bpy.data.images.load(path, check_existing=False)
    return img


def make_material(mat_name, image):
    """
    Create a Blender material with an image texture.
    """
    mat = bpy.data.materials.new(name=mat_name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()
    tex  = nodes.new("ShaderNodeTexImage")
    tex.image = image
    tex.location = (-300, 0)
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.location = (0, 0)
    out  = nodes.new("ShaderNodeOutputMaterial")
    out.location = (300, 0)
    links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    return mat


def make_plain_material(mat_name):
    """
    Create a plain white material with no texture.
    """
    mat = bpy.data.materials.new(name=mat_name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.inputs["Base Color"].default_value = (1.0, 1.0, 1.0, 1.0)
    bsdf.inputs["Roughness"].default_value = 1.0
    out = nodes.new("ShaderNodeOutputMaterial")
    links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    return mat


def normalize(vals):
    """
    Normalize a list of values to the range [0, 1].
    """
    lo, hi = min(vals), max(vals)
    span = hi - lo
    if span < 1e-9:
        return [0.5] * len(vals)
    return [(v - lo) / span for v in vals]


def assign_uvs(obj, is_manga=False):
    """
    Orientation of the book: cover on +Y, back on -Y.
    Spine on +X (-X for manga), as assigned by assign_materials.
    Top +Z and Bottom -Z.
    UV calculated with min/max on real vertices and no dependency on sx/sy/sz.
    """
    me = obj.data
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.faces.ensure_lookup_table()

    uv = bm.loops.layers.uv.verify()

    for face in bm.faces:
        n    = face.normal
        ax   = max(range(3), key=lambda k: abs(n[k]))
        verts = [l.vert.co for l in face.loops]

        if ax == 1:
            # Faces Y big: cover (+Y) and back (-Y)
            raw_u = [v.x for v in verts]
            raw_v = [v.z for v in verts]
            us = normalize(raw_u)
            vs = normalize(raw_v)
            if n[1] > 0:
                # For the cover, flip U to have the image correctly oriented
                us = [1.0 - u for u in us]

        elif ax == 0:
            # Faces X small: spine (+X; -X for manga) and pages
            raw_u = [v.y for v in verts]
            raw_v = [v.z for v in verts]
            us = normalize(raw_u)
            vs = normalize(raw_v)
            if is_manga and n[0] < 0:
                # Manga spine sits on -X, the mirror face of the default
                # +X spine: without this flip the spine texture (title
                # text included) is shown mirrored
                us = [1.0 - u for u in us]

        else:
            # Top (+Z) and bottom (-Z) faces
            raw_u = [v.x for v in verts]
            raw_v = [v.y for v in verts]
            us = normalize(raw_u)
            vs = normalize(raw_v)

        for loop, u, v in zip(face.loops, us, vs):
            loop[uv].uv = (u, v)

    bm.to_mesh(me)
    bm.free()
    me.update()


def assign_materials(obj, is_manga=False):
    """
    Assign materials to the book's faces based on their normals.
    Slot materials:
        - 0 = cover  (+Y)
        - 1 = retro  (-Y)
        - 2 = spine  (+X; -X for manga)
        - 3 = neutro (pages, top, bottom)
    """
    for poly in obj.data.polygons:
        n  = poly.normal
        ax = max(range(3), key=lambda k: abs(n[k]))
        print(f"  faccia {poly.index}: normale=({n.x:.2f},{n.y:.2f},{n.z:.2f}) ax={ax}")
        if ax == 1:
            poly.material_index = 0 if n[1] > 0 else 1
        elif ax == 0:
            spine_side = n[0] < 0 if is_manga else n[0] > 0
            poly.material_index = 2 if spine_side else 3
        else:
            poly.material_index = 3
        print(f"    → materiale {poly.material_index}")


def create_book(book, location=(0, 0, 0), images_dir=None, export_dir=None):
    """
    Create a book mesh in Blender with the specified textures.
    If export_dir is given, also export the book there as a .glb file.
    """
    print(f"\n=== {book['name']} ===")
    sx, sy, sz = book["size"]

    if images_dir is None:
        images_dir = script_dir()

    img_cover = load_image(book["cover"], images_dir)
    img_retro = load_image(book["retro"], images_dir)
    img_spine = load_image(book["spine"], images_dir)

    mat_cover = make_material(book["name"] + "_cover", img_cover)
    mat_retro = make_material(book["name"] + "_retro", img_retro)
    mat_spine = make_material(book["name"] + "_spine", img_spine)
    mat_plain = make_plain_material(book["name"] + "_plain")

    bpy.ops.mesh.primitive_cube_add(size=1, location=location)
    obj = bpy.context.active_object
    obj.name = book["name"]
    obj.scale = (sx, sy, sz)
    bpy.ops.object.transform_apply(scale=True)

    is_manga = book.get("manga", False)

    if not obj.data.uv_layers:
        obj.data.uv_layers.new(name="UVMap")

    assign_uvs(obj, is_manga=is_manga)

    obj.data.materials.clear()
    for mat in [mat_cover, mat_retro, mat_spine, mat_plain]:
        obj.data.materials.append(mat)

    assign_materials(obj, is_manga=is_manga)

    if export_dir is not None:
        out_path = os.path.join(export_dir, book["name"] + ".glb")
        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj

        bpy.ops.export_scene.gltf(
            filepath=out_path,
            use_selection=True,
            export_format="GLB",
            export_texcoords=True,
            export_materials="EXPORT",
            export_apply=True,
        )
        print(f"  → {out_path}")

    return obj

def main():
    """
    Create all books defined in the BOOKS list and export them as .glb files.
    """
    images_dir = script_dir()
    export_dir = os.path.normpath(
        os.path.join(images_dir, "..", "..", "src", "agibot_x2_pkg", "meshes", "books")
    )
    os.makedirs(export_dir, exist_ok=True)

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    for i, book_def in enumerate(BOOKS):
        create_book(book_def, location=(i * 0.4, 0, 0),
                    images_dir=images_dir, export_dir=export_dir)

    print("\nCreation done! File .glb saved in:", export_dir)

if __name__ == "__main__":
    main()