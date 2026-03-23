from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

from lightwheel.srl.from_usd.skinned_mesh_partition import (
    BakedMeshChunk,
    bake_chunk_to_body_frame,
    load_skinned_mesh_data,
    partition_triangles_by_body,
)
from lightwheel.srl.from_usd.protomotions_bundle_export import (
    export_protomotions_bundle,
)
from lightwheel.srl.from_usd.template_mjcf_visual_swap import (
    VisualGeomSpec,
    inject_visual_geoms,
    load_template_mjcf,
)


def _parse_floats(value: str | None, expected_length: int, default: tuple[float, ...]) -> np.ndarray:
    if value is None:
        return np.array(default, dtype=float)
    parts = [float(part) for part in value.split()]
    if len(parts) != expected_length:
        raise ValueError(f"Expected {expected_length} values but got '{value}'.")
    return np.array(parts, dtype=float)


def _quat_wxyz_to_matrix(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = quat
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _body_local_transform(body: ET.Element) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, 3] = _parse_floats(body.get("pos"), 3, (0.0, 0.0, 0.0))
    transform[:3, :3] = _quat_wxyz_to_matrix(
        _parse_floats(body.get("quat"), 4, (1.0, 0.0, 0.0, 0.0))
    )
    return transform


def _body_world_transforms(root_body: ET.Element) -> dict[str, np.ndarray]:
    transforms: dict[str, np.ndarray] = {}

    def walk(body: ET.Element, parent_transform: np.ndarray) -> None:
        body_name = body.get("name")
        local_transform = _body_local_transform(body)
        world_transform = parent_transform @ local_transform
        if body_name is not None:
            transforms[body_name] = world_transform
        for child in body.findall("body"):
            walk(child, world_transform)

    walk(root_body, np.eye(4, dtype=float))
    return transforms


def _template_body_world_transforms(template_model) -> dict[str, np.ndarray]:
    transforms: dict[str, np.ndarray] = {}
    for body in template_model.worldbody.findall("body"):
        transforms.update(_body_world_transforms(body))
    return transforms


def _resolve_joint_to_body_mapping(
    joint_names: list[str],
    joint_to_body_mapping: dict[str, str],
) -> dict[str, str]:
    parent_by_joint = {joint_name: _parent_joint_name(joint_name) for joint_name in joint_names}
    resolved: dict[str, str] = {}

    for joint_name in joint_names:
        current = joint_name
        while current is not None:
            if current in joint_to_body_mapping:
                resolved[joint_name] = joint_to_body_mapping[current]
                break

            leaf_name = current.split("/")[-1]
            if leaf_name in joint_to_body_mapping:
                resolved[joint_name] = joint_to_body_mapping[leaf_name]
                break

            current = parent_by_joint[current]
        else:
            raise KeyError(f"Could not resolve a body mapping for joint '{joint_name}'.")

    return resolved


def _parent_joint_name(joint_name: str) -> str | None:
    if "/" not in joint_name:
        return None
    return joint_name.rsplit("/", 1)[0]


def _face_slices(face_vertex_counts: np.ndarray) -> list[tuple[int, int]]:
    slices: list[tuple[int, int]] = []
    offset = 0
    for count in face_vertex_counts:
        slices.append((offset, int(count)))
        offset += int(count)
    return slices


def _extract_body_chunk(mesh, body_name: str, face_indices: list[int], target_world_transform: np.ndarray):
    face_slices = _face_slices(mesh.face_vertex_counts)
    used_vertices: list[int] = []
    seen_vertices: set[int] = set()
    new_counts: list[int] = []
    new_indices: list[int] = []

    for face_index in face_indices:
        start, count = face_slices[face_index]
        face_vertex_indices = mesh.face_vertex_indices[start:start + count]
        new_counts.append(count)
        for vertex_index in face_vertex_indices:
            vertex_index = int(vertex_index)
            if vertex_index not in seen_vertices:
                seen_vertices.add(vertex_index)
                used_vertices.append(vertex_index)

    index_map = {old_index: new_index for new_index, old_index in enumerate(used_vertices)}

    for face_index in face_indices:
        start, count = face_slices[face_index]
        face_vertex_indices = mesh.face_vertex_indices[start:start + count]
        for vertex_index in face_vertex_indices:
            new_indices.append(index_map[int(vertex_index)])

    chunk = BakedMeshChunk(
        body_name=body_name,
        material_name=None,
        points=mesh.points[used_vertices],
        face_vertex_counts=np.array(new_counts, dtype=int),
        face_vertex_indices=np.array(new_indices, dtype=int),
        normals=None,
        uvs=None,
    )
    return bake_chunk_to_body_frame(chunk, target_world_transform)


def _write_obj(path: Path, chunk: BakedMeshChunk) -> None:
    with path.open("w", encoding="utf-8") as file:
        file.write(f"o {chunk.body_name}\n")
        for point in chunk.points:
            file.write(f"v {point[0]} {point[1]} {point[2]}\n")

        offset = 0
        for count in chunk.face_vertex_counts:
            indices = chunk.face_vertex_indices[offset:offset + count]
            face_line = " ".join(str(int(index) + 1) for index in indices)
            file.write(f"f {face_line}\n")
            offset += int(count)


@dataclass
class ConversionReport:
    body_names: list[str]
    joint_names: list[str]
    motor_names: list[str]
    exported_triangle_count_by_body: dict[str, int]
    resolved_joint_to_body_mapping: dict[str, str]
    forced_assignment_face_count: int
    empty_body_warnings: list[str]
    protomotions_bundle_dir: Path | None = None
    protomotions_mjcf_path: Path | None = None
    protomotions_usd_path: Path | None = None
    protomotions_manifest_path: Path | None = None


def convert_skinned_visual_to_template_mjcf(
    template_mjcf_path: Path,
    template_usd_path: Path | None,
    visual_usd_path: Path,
    output_dir: Path,
    joint_to_body_mapping: dict[str, str],
    keep_template_visuals: bool = False,
    emit_protomotions_bundle: bool = False,
    robot_name: str | None = None,
    protomotions_root: Path | None = None,
    protomotions_python: str | None = None,
) -> tuple[Path, ConversionReport]:
    if emit_protomotions_bundle and robot_name is None:
        raise ValueError("robot_name is required when emit_protomotions_bundle=True.")
    if emit_protomotions_bundle and protomotions_root is None:
        raise ValueError("protomotions_root is required when emit_protomotions_bundle=True.")

    output_dir.mkdir(parents=True, exist_ok=True)
    visuals_dir = output_dir / "visuals"
    visuals_dir.mkdir(parents=True, exist_ok=True)

    template_model = load_template_mjcf(template_mjcf_path)
    template_body_names = template_model.body_names.copy()
    template_joint_names = template_model.joint_names.copy()
    template_motor_names = template_model.motor_names.copy()
    mesh = load_skinned_mesh_data(visual_usd_path, mesh_path="/World/biped_demo_meters/Body_Mesh")
    resolved_mapping = _resolve_joint_to_body_mapping(mesh.joint_names, joint_to_body_mapping)
    partitioned = partition_triangles_by_body(mesh, resolved_mapping)
    body_world_transforms = _template_body_world_transforms(template_model)

    visual_specs: list[VisualGeomSpec] = []
    triangle_counts: dict[str, int] = {}
    for body_name, chunk in partitioned.items():
        target_world_transform = body_world_transforms.get(body_name, np.eye(4, dtype=float))
        baked_chunk = _extract_body_chunk(mesh, body_name, chunk.face_indices, target_world_transform)
        obj_path = visuals_dir / f"{body_name}.obj"
        _write_obj(obj_path, baked_chunk)
        triangle_counts[body_name] = len(chunk.face_indices)
        visual_specs.append(
            VisualGeomSpec(
                body_name=body_name,
                geom_name=f"{body_name}_visual",
                mesh_name=f"{body_name}_visual",
                mesh_file=str(obj_path.relative_to(output_dir)),
                pos=(0.0, 0.0, 0.0),
                quat=(1.0, 0.0, 0.0, 0.0),
                material_name=None,
            )
        )

    inject_visual_geoms(
        template_model,
        visual_specs,
        keep_template_visuals=keep_template_visuals,
    )

    output_xml = output_dir / f"{template_mjcf_path.stem}_visual_swap.xml"
    ET.indent(template_model.tree, space="  ")
    template_model.tree.write(output_xml, encoding="utf-8", xml_declaration=True)

    if template_model.body_names != template_body_names:
        raise RuntimeError("Template body ordering changed during visual swap conversion.")
    if template_model.joint_names != template_joint_names:
        raise RuntimeError("Template joint ordering changed during visual swap conversion.")
    if template_model.motor_names != template_motor_names:
        raise RuntimeError("Template motor ordering changed during visual swap conversion.")

    report = ConversionReport(
        body_names=template_model.body_names,
        joint_names=template_model.joint_names,
        motor_names=template_model.motor_names,
        exported_triangle_count_by_body=triangle_counts,
        resolved_joint_to_body_mapping=resolved_mapping,
        forced_assignment_face_count=0,
        empty_body_warnings=[],
    )

    if emit_protomotions_bundle:
        bundle_paths = export_protomotions_bundle(
            output_dir=output_dir,
            output_mjcf_path=output_xml,
            visuals_dir=visuals_dir,
            visual_usd_path=visual_usd_path,
            template_mjcf_path=template_mjcf_path,
            template_usd_path=template_usd_path,
            protomotions_root=protomotions_root,
            robot_name=robot_name,
            body_names=report.body_names,
            joint_names=report.joint_names,
            motor_names=report.motor_names,
            triangle_counts=report.exported_triangle_count_by_body,
            joint_to_body_mapping=report.resolved_joint_to_body_mapping,
            protomotions_python=protomotions_python,
        )
        report.protomotions_bundle_dir = bundle_paths.bundle_dir
        report.protomotions_mjcf_path = bundle_paths.bundled_mjcf_path
        report.protomotions_usd_path = bundle_paths.usd_path
        report.protomotions_manifest_path = bundle_paths.manifest_path

    return output_xml, report
