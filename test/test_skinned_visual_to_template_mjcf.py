from pathlib import Path

import numpy as np
import pytest

from lightwheel.srl.from_usd.skinned_mesh_partition import (
    BakedMeshChunk,
    FaceMaterialRange,
    SkinMeshData,
    bake_chunk_to_body_frame,
    collapse_joint_mapping_to_parent,
    load_skinned_mesh_data,
    partition_triangles_by_body,
)
from lightwheel.srl.from_usd.template_mjcf_visual_swap import (
    VisualGeomSpec,
    inject_visual_geoms,
    load_template_mjcf,
)


def test_load_template_mjcf_preserves_body_joint_and_motor_order(tmp_path: Path):
    template_path = tmp_path / "template.xml"
    template_path.write_text(
        """
        <mujoco model="humanoid">
          <worldbody>
            <body name="Pelvis">
              <freejoint name="Pelvis"/>
              <geom name="pelvis_geom" type="box" size="1 1 1" density="1000"/>
              <body name="Torso">
                <joint name="Torso_x" type="hinge" axis="1 0 0"/>
                <joint name="Torso_y" type="hinge" axis="0 1 0"/>
                <joint name="Torso_z" type="hinge" axis="0 0 1"/>
                <geom name="torso_geom" type="capsule" size="1"/>
              </body>
            </body>
          </worldbody>
          <actuator>
            <motor name="Torso_x" joint="Torso_x" gear="500"/>
            <motor name="Torso_y" joint="Torso_y" gear="500"/>
            <motor name="Torso_z" joint="Torso_z" gear="500"/>
          </actuator>
        </mujoco>
        """.strip()
    )

    model = load_template_mjcf(template_path)

    assert model.body_names == ["Pelvis", "Torso"]
    assert model.joint_names == ["Torso_x", "Torso_y", "Torso_z"]
    assert model.motor_names == ["Torso_x", "Torso_y", "Torso_z"]


def test_inject_visual_geoms_keeps_template_structure_and_disables_contacts(
    tmp_path: Path,
):
    template_path = tmp_path / "template.xml"
    template_path.write_text(
        """
        <mujoco model="humanoid">
          <asset/>
          <worldbody>
            <body name="Pelvis">
              <geom
                name="pelvis_geom"
                type="box"
                size="1 1 1"
                density="1000"
                contype="1"
                conaffinity="1"
              />
            </body>
          </worldbody>
          <actuator/>
        </mujoco>
        """.strip()
    )

    model = load_template_mjcf(template_path)
    inject_visual_geoms(
        model,
        [
            VisualGeomSpec(
                body_name="Pelvis",
                geom_name="Pelvis_visual",
                mesh_name="Pelvis_visual",
                mesh_file="visuals/Pelvis_visual.obj",
                pos=(0.0, 0.0, 0.0),
                quat=(1.0, 0.0, 0.0, 0.0),
                material_name=None,
            )
        ],
        keep_template_visuals=False,
    )

    pelvis_geoms = model.body_to_geoms["Pelvis"]
    original = [geom for geom in pelvis_geoms if geom.get("name") == "pelvis_geom"][0]
    injected = [geom for geom in pelvis_geoms if geom.get("name") == "Pelvis_visual"][0]

    assert original.get("density") == "1000"
    assert original.get("rgba") == "0 0 0 0"
    assert injected.get("type") == "mesh"
    assert injected.get("contype") == "0"
    assert injected.get("conaffinity") == "0"
    assert injected.get("mesh") == "Pelvis_visual"


def test_partition_triangles_by_triangle_majority_weight():
    mesh = SkinMeshData(
        points=np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [1.0, 1.0, 0.0],
            ],
            dtype=float,
        ),
        face_vertex_counts=np.array([3, 3], dtype=int),
        face_vertex_indices=np.array([0, 1, 2, 1, 3, 2], dtype=int),
        joint_names=["Pelvis", "L_UpLeg", "L_LoLeg"],
        joint_indices=np.array([[0, 1], [0, 1], [0, 2], [0, 2]], dtype=int),
        joint_weights=np.array([[0.1, 0.9], [0.2, 0.8], [0.4, 0.6], [0.8, 0.2]], dtype=float),
        geom_bind_transform=np.eye(4),
        joint_world_bind_transforms={},
        normals=None,
        uvs=None,
        material_ranges=[FaceMaterialRange(material_name="Body", face_indices=[0, 1])],
    )
    joint_to_body = {"Pelvis": "Pelvis", "L_UpLeg": "L_Hip", "L_LoLeg": "L_Knee"}

    result = partition_triangles_by_body(mesh, joint_to_body)

    assert result["L_Hip"].face_indices == [0]
    assert result["L_Knee"].face_indices == [1]


def test_collapse_unmapped_joint_to_nearest_mapped_parent():
    parent_by_joint = {
        "Root": None,
        "Root/Pelvis": "Root",
        "Root/Pelvis/L_UpLeg": "Root/Pelvis",
        "Root/Pelvis/L_UpLeg/L_UpLegTwist1": "Root/Pelvis/L_UpLeg",
    }
    explicit_mapping = {
        "Root": "Pelvis",
        "Root/Pelvis": "Pelvis",
        "Root/Pelvis/L_UpLeg": "L_Hip",
    }

    collapsed = collapse_joint_mapping_to_parent(
        "Root/Pelvis/L_UpLeg/L_UpLegTwist1",
        explicit_mapping,
        parent_by_joint,
    )

    assert collapsed == "L_Hip"


def test_bake_chunk_to_body_frame_transforms_points_into_target_local_frame():
    chunk = BakedMeshChunk(
        body_name="Pelvis",
        material_name="Body",
        points=np.array([[2.0, 0.0, 0.0]], dtype=float),
        face_vertex_counts=np.array([3], dtype=int),
        face_vertex_indices=np.array([0, 0, 0], dtype=int),
        normals=None,
        uvs=None,
    )
    target_world = np.eye(4)
    target_world[0, 3] = 1.0

    baked = bake_chunk_to_body_frame(chunk, target_world)

    assert baked.points.tolist() == [[1.0, 0.0, 0.0]]


def test_load_skinned_mesh_data_reads_biped_usd_real_asset():
    usd_path = Path("/home/lyuxinghe/code/Characters/Biped_Setup.usd")
    if not usd_path.exists():
        pytest.skip("Real asset not available in this environment.")

    mesh = load_skinned_mesh_data(usd_path, mesh_path="/World/biped_demo_meters/Body_Mesh")

    assert mesh.points.shape[0] > 0
    assert mesh.face_vertex_counts.shape[0] > 0
    assert len(mesh.joint_names) == 81
