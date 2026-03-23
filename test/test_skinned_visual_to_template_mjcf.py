import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

import lightwheel.srl.from_usd.skinned_visual_to_template_mjcf as skinned_visual_module
from lightwheel.srl.from_usd.skinned_visual_to_template_mjcf import (
    convert_skinned_visual_to_template_mjcf,
)
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


def _write_minimal_template_xml(path: Path) -> None:
    path.write_text(
        """
        <mujoco model="humanoid">
          <asset/>
          <worldbody>
            <body name="Pelvis">
              <freejoint name="Pelvis"/>
              <geom name="pelvis_geom" type="box" size="1 1 1" density="1000"/>
            </body>
          </worldbody>
          <actuator/>
        </mujoco>
        """.strip(),
        encoding="utf-8",
    )


def _simple_skin_mesh() -> SkinMeshData:
    return SkinMeshData(
        points=np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=float,
        ),
        face_vertex_counts=np.array([3], dtype=int),
        face_vertex_indices=np.array([0, 1, 2], dtype=int),
        joint_names=["Root", "Pelvis"],
        joint_indices=np.array([[0, 1], [0, 1], [0, 1]], dtype=int),
        joint_weights=np.array([[0.1, 0.9], [0.2, 0.8], [0.3, 0.7]], dtype=float),
        geom_bind_transform=np.eye(4),
        joint_world_bind_transforms={"Root": np.eye(4), "Pelvis": np.eye(4)},
        normals=None,
        uvs=None,
        material_ranges=[FaceMaterialRange(material_name="Body", face_indices=[0])],
    )


def _write_fake_protomotions_root(root: Path) -> Path:
    script_path = root / "data" / "scripts" / "convert_mjcf_to_usd.py"
    script_path.parent.mkdir(parents=True, exist_ok=True)
    script_path.write_text(
        "import sys\nPath = None\n",
        encoding="utf-8",
    )
    return script_path


def _patch_simple_mesh(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        skinned_visual_module,
        "load_skinned_mesh_data",
        lambda *_args, **_kwargs: _simple_skin_mesh(),
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


def test_end_to_end_conversion_preserves_template_joint_and_motor_order(
    tmp_path: Path,
):
    visual_usd_path = Path("/home/lyuxinghe/code/Characters/Biped_Setup.usd")
    if not visual_usd_path.exists():
        pytest.skip("Real asset not available in this environment.")

    template_path = tmp_path / "template.xml"
    template_path.write_text(
        """
        <mujoco model="humanoid">
          <asset/>
          <worldbody>
            <body name="Pelvis">
              <freejoint name="Pelvis"/>
              <geom name="pelvis_geom" type="box" size="1 1 1" density="1000"/>
            </body>
          </worldbody>
          <actuator/>
        </mujoco>
        """.strip()
    )

    output_xml, report = convert_skinned_visual_to_template_mjcf(
        template_mjcf_path=template_path,
        template_usd_path=None,
        visual_usd_path=visual_usd_path,
        output_dir=tmp_path / "out",
        joint_to_body_mapping={"Root": "Pelvis", "Pelvis": "Pelvis"},
        keep_template_visuals=False,
    )

    assert output_xml.exists()
    assert report.body_names == ["Pelvis"]
    assert "Pelvis" in report.exported_triangle_count_by_body


def test_output_dimensions_match_template_when_mujoco_available(tmp_path: Path):
    mujoco = pytest.importorskip("mujoco")
    visual_usd_path = Path("/home/lyuxinghe/code/Characters/Biped_Setup.usd")
    if not visual_usd_path.exists():
        pytest.skip("Real asset not available in this environment.")

    template_path = tmp_path / "template.xml"
    template_path.write_text(
        """
        <mujoco model="humanoid">
          <asset/>
          <worldbody>
            <body name="Pelvis">
              <freejoint name="Pelvis"/>
              <geom name="pelvis_geom" type="box" size="1 1 1" density="1000"/>
            </body>
          </worldbody>
          <actuator/>
        </mujoco>
        """.strip()
    )

    output_xml, _ = convert_skinned_visual_to_template_mjcf(
        template_mjcf_path=template_path,
        template_usd_path=None,
        visual_usd_path=visual_usd_path,
        output_dir=tmp_path / "out",
        joint_to_body_mapping={"Root": "Pelvis", "Pelvis": "Pelvis"},
        keep_template_visuals=False,
    )

    template_model = mujoco.MjModel.from_xml_path(template_path.as_posix())
    output_model = mujoco.MjModel.from_xml_path(output_xml.as_posix())

    assert output_model.nq == template_model.nq
    assert output_model.nv == template_model.nv
    assert output_model.nu == template_model.nu


def test_bundle_export_writes_expected_files(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    _patch_simple_mesh(monkeypatch)
    template_path = tmp_path / "template.xml"
    _write_minimal_template_xml(template_path)

    protomotions_root = tmp_path / "ProtoMotions"
    _write_fake_protomotions_root(protomotions_root)

    def fake_run(cmd, check, **_kwargs):
        usd_path = Path(cmd[3])
        usd_path.parent.mkdir(parents=True, exist_ok=True)
        usd_path.write_text("#usda 1.0\n", encoding="utf-8")
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", fake_run)

    output_xml, report = convert_skinned_visual_to_template_mjcf(
        template_mjcf_path=template_path,
        template_usd_path=None,
        visual_usd_path=tmp_path / "visual.usd",
        output_dir=tmp_path / "out",
        joint_to_body_mapping={"Root": "Pelvis", "Pelvis": "Pelvis"},
        keep_template_visuals=False,
        emit_protomotions_bundle=True,
        robot_name="biped_smpl_visual_swap",
        protomotions_root=protomotions_root,
    )

    bundle_dir = tmp_path / "out" / "protomotions_bundle"
    assert output_xml.exists()
    assert (bundle_dir / "assets" / "mjcf" / "biped_smpl_visual_swap.xml").is_file()
    assert (bundle_dir / "assets" / "usd" / "biped_smpl_visual_swap" / "biped_smpl_visual_swap.usda").is_file()
    assert (bundle_dir / "assets" / "usd" / "biped_smpl_visual_swap" / "config.yaml").is_file()
    assert (bundle_dir / "robot_configs" / "biped_smpl_visual_swap.py").is_file()
    assert (bundle_dir / "factory_registration_snippet.py").is_file()
    assert (bundle_dir / "install_instructions.md").is_file()
    assert (bundle_dir / "manifest.json").is_file()
    assert report.protomotions_bundle_dir == bundle_dir


def test_bundle_export_invokes_protomotions_mjcf_to_usd(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    _patch_simple_mesh(monkeypatch)
    template_path = tmp_path / "template.xml"
    _write_minimal_template_xml(template_path)
    protomotions_root = tmp_path / "ProtoMotions"
    script_path = _write_fake_protomotions_root(protomotions_root)
    recorded_commands = []

    def fake_run(cmd, check, **_kwargs):
        recorded_commands.append(cmd)
        usd_path = Path(cmd[3])
        usd_path.parent.mkdir(parents=True, exist_ok=True)
        usd_path.write_text("#usda 1.0\n", encoding="utf-8")
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", fake_run)

    convert_skinned_visual_to_template_mjcf(
        template_mjcf_path=template_path,
        template_usd_path=None,
        visual_usd_path=tmp_path / "visual.usd",
        output_dir=tmp_path / "out",
        joint_to_body_mapping={"Root": "Pelvis", "Pelvis": "Pelvis"},
        emit_protomotions_bundle=True,
        robot_name="biped_smpl_visual_swap",
        protomotions_root=protomotions_root,
    )

    assert len(recorded_commands) == 1
    assert recorded_commands[0][1] == script_path.as_posix()
    assert recorded_commands[0][2].endswith("assets/mjcf/biped_smpl_visual_swap.xml")
    assert recorded_commands[0][3].endswith(
        "assets/usd/biped_smpl_visual_swap/biped_smpl_visual_swap.usda"
    )


def test_bundle_export_uses_requested_protomotions_python(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    _patch_simple_mesh(monkeypatch)
    template_path = tmp_path / "template.xml"
    _write_minimal_template_xml(template_path)
    protomotions_root = tmp_path / "ProtoMotions"
    _write_fake_protomotions_root(protomotions_root)
    recorded_commands = []

    def fake_run(cmd, check, **_kwargs):
        recorded_commands.append(cmd)
        usd_path = Path(cmd[3])
        usd_path.parent.mkdir(parents=True, exist_ok=True)
        usd_path.write_text("#usda 1.0\n", encoding="utf-8")
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", fake_run)

    convert_skinned_visual_to_template_mjcf(
        template_mjcf_path=template_path,
        template_usd_path=None,
        visual_usd_path=tmp_path / "visual.usd",
        output_dir=tmp_path / "out",
        joint_to_body_mapping={"Root": "Pelvis", "Pelvis": "Pelvis"},
        emit_protomotions_bundle=True,
        robot_name="biped_smpl_visual_swap",
        protomotions_root=protomotions_root,
        protomotions_python="/custom/protomotions/python",
    )

    assert recorded_commands[0][0] == "/custom/protomotions/python"


def test_bundle_export_requires_protomotions_root(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    _patch_simple_mesh(monkeypatch)
    template_path = tmp_path / "template.xml"
    _write_minimal_template_xml(template_path)

    with pytest.raises(ValueError):
        convert_skinned_visual_to_template_mjcf(
            template_mjcf_path=template_path,
            template_usd_path=None,
            visual_usd_path=tmp_path / "visual.usd",
            output_dir=tmp_path / "out",
            joint_to_body_mapping={"Root": "Pelvis", "Pelvis": "Pelvis"},
            emit_protomotions_bundle=True,
            robot_name="biped_smpl_visual_swap",
            protomotions_root=None,
        )


def test_bundle_manifest_and_robot_config_contents(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    _patch_simple_mesh(monkeypatch)
    template_path = tmp_path / "template.xml"
    _write_minimal_template_xml(template_path)
    protomotions_root = tmp_path / "ProtoMotions"
    _write_fake_protomotions_root(protomotions_root)

    def fake_run(cmd, check, **_kwargs):
        usd_path = Path(cmd[3])
        usd_path.parent.mkdir(parents=True, exist_ok=True)
        usd_path.write_text("#usda 1.0\n", encoding="utf-8")
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", fake_run)

    _, report = convert_skinned_visual_to_template_mjcf(
        template_mjcf_path=template_path,
        template_usd_path=None,
        visual_usd_path=tmp_path / "visual.usd",
        output_dir=tmp_path / "out",
        joint_to_body_mapping={"Root": "Pelvis", "Pelvis": "Pelvis"},
        emit_protomotions_bundle=True,
        robot_name="biped_smpl_visual_swap",
        protomotions_root=protomotions_root,
    )

    bundle_dir = tmp_path / "out" / "protomotions_bundle"
    manifest = json.loads((bundle_dir / "manifest.json").read_text(encoding="utf-8"))
    robot_config_text = (bundle_dir / "robot_configs" / "biped_smpl_visual_swap.py").read_text(
        encoding="utf-8"
    )

    assert manifest["robot_name"] == "biped_smpl_visual_swap"
    assert manifest["body_names"] == report.body_names
    assert manifest["joint_names"] == report.joint_names
    assert manifest["motor_names"] == report.motor_names
    assert "class BipedSmplVisualSwapRobotConfig" in robot_config_text
    assert 'asset_file_name="mjcf/biped_smpl_visual_swap.xml"' in robot_config_text
    assert 'usd_asset_file_name="usd/biped_smpl_visual_swap/biped_smpl_visual_swap.usda"' in robot_config_text


def test_cli_help_runs_from_repo_root():
    cli_path = Path("test/skinned_visual_to_template_mjcf_test.py")
    completed = subprocess.run(
        [sys.executable, cli_path.as_posix(), "--help"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr


def test_cli_help_includes_protomotions_bundle_flags():
    cli_path = Path("test/skinned_visual_to_template_mjcf_test.py")
    completed = subprocess.run(
        [sys.executable, cli_path.as_posix(), "--help"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert "--emit-protomotions-bundle" in completed.stdout
    assert "--robot-name" in completed.stdout
    assert "--protomotions-root" in completed.stdout
    assert "--protomotions-python" in completed.stdout
