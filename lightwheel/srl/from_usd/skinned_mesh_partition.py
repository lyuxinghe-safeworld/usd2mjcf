from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom, UsdShade, UsdSkel


@dataclass
class FaceMaterialRange:
    material_name: str | None
    face_indices: list[int]


@dataclass
class SkinMeshData:
    points: np.ndarray
    face_vertex_counts: np.ndarray
    face_vertex_indices: np.ndarray
    joint_names: list[str]
    joint_indices: np.ndarray
    joint_weights: np.ndarray
    geom_bind_transform: np.ndarray
    joint_world_bind_transforms: dict[str, np.ndarray]
    normals: np.ndarray | None
    uvs: np.ndarray | None
    material_ranges: list[FaceMaterialRange]


@dataclass
class PartitionedMeshChunk:
    body_name: str
    face_indices: list[int] = field(default_factory=list)


@dataclass
class BakedMeshChunk:
    body_name: str
    material_name: str | None
    points: np.ndarray
    face_vertex_counts: np.ndarray
    face_vertex_indices: np.ndarray
    normals: np.ndarray | None
    uvs: np.ndarray | None


def _joint_parent_name(joint_name: str) -> str | None:
    if "/" not in joint_name:
        return None
    return joint_name.rsplit("/", 1)[0]


def _material_ranges_for_mesh(mesh: UsdGeom.Mesh) -> list[FaceMaterialRange]:
    subsets = UsdGeom.Subset.GetAllGeomSubsets(mesh)
    if subsets:
        ranges: list[FaceMaterialRange] = []
        for subset in subsets:
            binding = UsdShade.MaterialBindingAPI(subset.GetPrim())
            material = binding.ComputeBoundMaterial()[0]
            material_name = None
            if material and material.GetPrim():
                material_name = material.GetPrim().GetName()
            ranges.append(
                FaceMaterialRange(
                    material_name=material_name,
                    face_indices=list(subset.GetIndicesAttr().Get()),
                )
            )
        return ranges

    face_count = len(mesh.GetFaceVertexCountsAttr().Get())
    binding = UsdShade.MaterialBindingAPI(mesh.GetPrim())
    material = binding.ComputeBoundMaterial()[0]
    material_name = None
    if material and material.GetPrim():
        material_name = material.GetPrim().GetName()
    return [FaceMaterialRange(material_name=material_name, face_indices=list(range(face_count)))]


def _world_rest_transforms(
    joint_names: list[str],
    rest_transforms: list,
) -> dict[str, np.ndarray]:
    local_by_joint = {
        joint_name: np.array(rest_transform, dtype=float)
        for joint_name, rest_transform in zip(joint_names, rest_transforms)
    }
    world_by_joint: dict[str, np.ndarray] = {}

    for joint_name in joint_names:
        parent_name = _joint_parent_name(joint_name)
        local_transform = local_by_joint[joint_name]
        if parent_name is None:
            world_by_joint[joint_name] = local_transform
            continue
        world_by_joint[joint_name] = world_by_joint[parent_name] @ local_transform

    return world_by_joint


def load_skinned_mesh_data(
    usd_path: Path | str,
    mesh_path: str,
) -> SkinMeshData:
    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise ValueError(f"Could not open USD stage '{usd_path}'.")

    mesh_prim = stage.GetPrimAtPath(mesh_path)
    if not mesh_prim or not mesh_prim.IsA(UsdGeom.Mesh):
        raise ValueError(f"Mesh '{mesh_path}' not found or is not a UsdGeom.Mesh.")

    mesh = UsdGeom.Mesh(mesh_prim)
    binding = UsdSkel.BindingAPI(mesh_prim)
    skeleton_targets = binding.GetSkeletonRel().GetTargets()
    if not skeleton_targets:
        raise ValueError(f"Mesh '{mesh_path}' is not bound to a skeleton.")

    skeleton_prim = stage.GetPrimAtPath(str(skeleton_targets[0]))
    skeleton = UsdSkel.Skeleton(skeleton_prim)
    joint_names = [str(joint) for joint in skeleton.GetJointsAttr().Get()]

    joint_indices_primvar = binding.GetJointIndicesPrimvar()
    joint_weights_primvar = binding.GetJointWeightsPrimvar()
    element_size = joint_indices_primvar.GetElementSize()
    joint_indices = np.array(joint_indices_primvar.Get(), dtype=int).reshape((-1, element_size))
    joint_weights = np.array(joint_weights_primvar.Get(), dtype=float).reshape((-1, element_size))

    primvars_api = UsdGeom.PrimvarsAPI(mesh)
    st_primvar = primvars_api.GetPrimvar("st")
    uvs = None
    if st_primvar and st_primvar.IsDefined():
        uvs = np.array(st_primvar.Get(), dtype=float)

    normals = None
    normals_attr = mesh.GetNormalsAttr().Get()
    if normals_attr:
        normals = np.array(normals_attr, dtype=float)

    rest_transforms = skeleton.GetRestTransformsAttr().Get() or []

    return SkinMeshData(
        points=np.array(mesh.GetPointsAttr().Get(), dtype=float),
        face_vertex_counts=np.array(mesh.GetFaceVertexCountsAttr().Get(), dtype=int),
        face_vertex_indices=np.array(mesh.GetFaceVertexIndicesAttr().Get(), dtype=int),
        joint_names=joint_names,
        joint_indices=joint_indices,
        joint_weights=joint_weights,
        geom_bind_transform=np.array(binding.GetGeomBindTransformAttr().Get(), dtype=float),
        joint_world_bind_transforms=_world_rest_transforms(joint_names, rest_transforms),
        normals=normals,
        uvs=uvs,
        material_ranges=_material_ranges_for_mesh(mesh),
    )


def collapse_joint_mapping_to_parent(
    joint_name: str,
    explicit_mapping: dict[str, str],
    parent_by_joint: dict[str, str | None],
) -> str:
    current_joint = joint_name
    while current_joint is not None:
        mapped_body = explicit_mapping.get(current_joint)
        if mapped_body is not None:
            return mapped_body
        current_joint = parent_by_joint.get(current_joint)

    raise KeyError(f"Could not collapse joint '{joint_name}' to a mapped parent.")


def partition_triangles_by_body(
    mesh: SkinMeshData,
    joint_to_body: dict[str, str],
) -> dict[str, PartitionedMeshChunk]:
    result: dict[str, PartitionedMeshChunk] = {}
    face_offset = 0

    for face_index, vertex_count in enumerate(mesh.face_vertex_counts):
        vertex_indices = mesh.face_vertex_indices[face_offset:face_offset + vertex_count]
        face_offset += vertex_count

        body_weights: dict[str, float] = {}
        for vertex_index in vertex_indices:
            for joint_index, joint_weight in zip(
                mesh.joint_indices[vertex_index],
                mesh.joint_weights[vertex_index],
            ):
                body_name = joint_to_body.get(mesh.joint_names[int(joint_index)])
                if body_name is None:
                    continue
                body_weights[body_name] = body_weights.get(body_name, 0.0) + float(joint_weight)

        if not body_weights:
            raise KeyError(f"No mapped body weights found for face {face_index}.")

        # Pelvis influences often bleed into neighboring limbs at skinning seams.
        # Treat it as a fallback body when a more specific mapped body is present.
        if len(body_weights) > 1 and "Pelvis" in body_weights:
            body_weights.pop("Pelvis")

        target_body = max(
            body_weights.items(),
            key=lambda item: (item[1], item[0]),
        )[0]
        if target_body not in result:
            result[target_body] = PartitionedMeshChunk(body_name=target_body)
        result[target_body].face_indices.append(face_index)

    return result


def bake_chunk_to_body_frame(
    chunk: BakedMeshChunk,
    target_world_transform: np.ndarray,
) -> BakedMeshChunk:
    target_world_inverse = np.linalg.inv(target_world_transform)
    homogeneous_points = np.concatenate(
        [chunk.points, np.ones((chunk.points.shape[0], 1), dtype=chunk.points.dtype)],
        axis=1,
    )
    baked_points = (target_world_inverse @ homogeneous_points.T).T[:, :3]

    baked_normals = None
    if chunk.normals is not None:
        baked_normals = (target_world_inverse[:3, :3] @ chunk.normals.T).T

    return BakedMeshChunk(
        body_name=chunk.body_name,
        material_name=chunk.material_name,
        points=baked_points,
        face_vertex_counts=chunk.face_vertex_counts.copy(),
        face_vertex_indices=chunk.face_vertex_indices.copy(),
        normals=baked_normals,
        uvs=None if chunk.uvs is None else chunk.uvs.copy(),
    )
