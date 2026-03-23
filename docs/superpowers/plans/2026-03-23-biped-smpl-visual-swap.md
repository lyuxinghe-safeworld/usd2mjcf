# Biped-to-SMPL Visual Swap Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a new conversion path that preserves the SMPL MJCF physics template exactly while swapping in Biped USD visuals, so the existing motion-tracking controller can run unchanged.

**Architecture:** Keep the existing rigid-body `UsdToMjcf` pipeline untouched. Add a separate template-preserving pipeline that loads the SMPL MJCF as XML, loads the Biped `UsdSkel` mesh and skinning data, partitions triangles onto existing SMPL bodies, exports per-body visual meshes, and injects those meshes as non-physical geoms into a cloned MJCF template.

**Tech Stack:** Python, `pxr` (`Usd`, `UsdGeom`, `UsdShade`, `UsdSkel`), `xml.etree.ElementTree`, `numpy`, existing OBJ/MTL export helpers in this repo, `pytest` for tests, optional `mujoco` bindings for structural dimension verification.

---

## Spec Reference

- `docs/superpowers/specs/2026-03-23-biped-smpl-visual-swap-design.md`

## File Structure

### New files

- `lightwheel/srl/from_usd/template_mjcf_visual_swap.py`
  - Parse and clone the template MJCF.
  - Index bodies, joints, motors, and physics geoms.
  - Hide template physics geoms visually without changing physics.
  - Inject new visual-only mesh geoms and mesh/material/texture assets.
  - Emit structural reports used by tests and smoke runs.

- `lightwheel/srl/from_usd/skinned_mesh_partition.py`
  - Load a skinned mesh and material bindings from a `UsdSkel` stage.
  - Collapse Biped joints onto approved SMPL body names.
  - Partition triangles by mapped skin weights.
  - Bake partitioned chunks into target SMPL body local frames.

- `lightwheel/srl/from_usd/skinned_visual_to_template_mjcf.py`
  - Own the end-to-end orchestration API.
  - Call the USD loader/partitioner.
  - Export mesh assets and copied textures.
  - Call the MJCF template injector.
  - Save the final XML and report.

- `test/test_skinned_visual_to_template_mjcf.py`
  - Unit and integration tests for the new pipeline.
  - Synthetic MJCF and synthetic skin-data fixtures built inside the test file.
  - Optional MuJoCo dimension checks via `pytest.importorskip("mujoco")`.

- `test/skinned_visual_to_template_mjcf_test.py`
  - CLI entrypoint parallel to `test/usd2mjcf_test.py`.
  - Accepts template MJCF, template USD, visual USD, and output dir arguments.

### Modified files

- `lightwheel/srl/from_usd/__init__.py`
  - Export the new public entrypoint.

- `requirements.txt`
  - Add `pytest` so the repo can run the new test suite predictably.

### Existing files to reference during implementation

- `lightwheel/srl/from_usd/usd_to_obj.py`
  - Reuse or mirror material/OBJ export behavior where practical.

- `lightwheel/srl/from_usd/_from_usd_helper.py`
  - Reuse material extraction and texture-copy logic where practical.

- `utils/add_collision.py`
  - Reference for `xml.etree.ElementTree` mutation patterns already used in the repo.

## Chunk 1: Template MJCF Parsing And Injection

### Task 1: Add the test dependency and scaffold template tests

**Files:**
- Modify: `requirements.txt`
- Create: `test/test_skinned_visual_to_template_mjcf.py`

- [ ] **Step 1: Write the failing dependency change**

Add this line to `requirements.txt`:

```text
pytest
```

- [ ] **Step 2: Write the failing template-parsing test**

Add this test skeleton to `test/test_skinned_visual_to_template_mjcf.py`:

```python
from pathlib import Path

from lightwheel.srl.from_usd.template_mjcf_visual_swap import load_template_mjcf


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
```

- [ ] **Step 3: Run the test to verify it fails**

Run:

```bash
python -m pytest test/test_skinned_visual_to_template_mjcf.py::test_load_template_mjcf_preserves_body_joint_and_motor_order -v
```

Expected: FAIL with `ModuleNotFoundError` or `ImportError` for `template_mjcf_visual_swap`.

- [ ] **Step 4: Commit the red test**

```bash
git add requirements.txt test/test_skinned_visual_to_template_mjcf.py
git commit -m "test: add template MJCF visual swap scaffolding"
```

### Task 2: Implement template parsing, hiding, and injection helpers

**Files:**
- Create: `lightwheel/srl/from_usd/template_mjcf_visual_swap.py`
- Modify: `test/test_skinned_visual_to_template_mjcf.py`

- [ ] **Step 1: Add the next failing test for visual injection**

Append this test:

```python
from lightwheel.srl.from_usd.template_mjcf_visual_swap import (
    VisualGeomSpec,
    inject_visual_geoms,
    load_template_mjcf,
)


def test_inject_visual_geoms_keeps_template_structure_and_disables_contacts(tmp_path: Path):
    template_path = tmp_path / "template.xml"
    template_path.write_text(
        """
        <mujoco model="humanoid">
          <asset/>
          <worldbody>
            <body name="Pelvis">
              <geom name="pelvis_geom" type="box" size="1 1 1" density="1000" contype="1" conaffinity="1"/>
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
```

- [ ] **Step 2: Run the focused tests and confirm they fail**

Run:

```bash
python -m pytest test/test_skinned_visual_to_template_mjcf.py::test_load_template_mjcf_preserves_body_joint_and_motor_order test/test_skinned_visual_to_template_mjcf.py::test_inject_visual_geoms_keeps_template_structure_and_disables_contacts -v
```

Expected: FAIL because `load_template_mjcf`, `VisualGeomSpec`, and `inject_visual_geoms` are not implemented.

- [ ] **Step 3: Implement the XML template module**

Create `lightwheel/srl/from_usd/template_mjcf_visual_swap.py` with focused helpers like:

```python
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET


@dataclass
class VisualGeomSpec:
    body_name: str
    geom_name: str
    mesh_name: str
    mesh_file: str
    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]
    material_name: str | None


@dataclass
class TemplateMjcfModel:
    tree: ET.ElementTree
    root: ET.Element
    asset: ET.Element
    worldbody: ET.Element
    actuator: ET.Element
    body_by_name: dict[str, ET.Element]
    body_names: list[str]
    joint_names: list[str]
    motor_names: list[str]
    body_to_geoms: dict[str, list[ET.Element]]


def load_template_mjcf(path: Path) -> TemplateMjcfModel:
    tree = ET.parse(path)
    root = tree.getroot()
    asset = root.find("asset") or ET.SubElement(root, "asset")
    worldbody = root.find("worldbody")
    actuator = root.find("actuator") or ET.SubElement(root, "actuator")
    if worldbody is None:
        raise ValueError("Template MJCF is missing <worldbody>.")
    ...


def inject_visual_geoms(
    model: TemplateMjcfModel,
    visual_specs: list[VisualGeomSpec],
    keep_template_visuals: bool = False,
) -> None:
    ...
```

Implementation requirements:

- Index bodies in traversal order.
- Index joints and motors in traversal order.
- Build `body_to_geoms`.
- When `keep_template_visuals` is false, set existing physics geoms to `rgba="0 0 0 0"` instead of deleting them.
- Add `<mesh>` entries under `<asset>`.
- Add injected `<geom type="mesh">` entries under the target body with:
  - `contype="0"`
  - `conaffinity="0"`
  - no `density`

- [ ] **Step 4: Run the focused tests and confirm they pass**

Run:

```bash
python -m pytest test/test_skinned_visual_to_template_mjcf.py::test_load_template_mjcf_preserves_body_joint_and_motor_order test/test_skinned_visual_to_template_mjcf.py::test_inject_visual_geoms_keeps_template_structure_and_disables_contacts -v
```

Expected: PASS.

- [ ] **Step 5: Commit the template module**

```bash
git add lightwheel/srl/from_usd/template_mjcf_visual_swap.py test/test_skinned_visual_to_template_mjcf.py requirements.txt
git commit -m "feat: add template MJCF visual swap helpers"
```

## Chunk 2: Skinned Mesh Loading, Mapping, And Baking

### Task 3: Write failing tests for mapping and triangle partitioning

**Files:**
- Create: `lightwheel/srl/from_usd/skinned_mesh_partition.py`
- Modify: `test/test_skinned_visual_to_template_mjcf.py`

- [ ] **Step 1: Add a failing pure-data partition test**

Append this test:

```python
import numpy as np

from lightwheel.srl.from_usd.skinned_mesh_partition import (
    FaceMaterialRange,
    SkinMeshData,
    partition_triangles_by_body,
)


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
```

- [ ] **Step 2: Add a failing unmapped-joint collapse test**

Append this test:

```python
from lightwheel.srl.from_usd.skinned_mesh_partition import collapse_joint_mapping_to_parent


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
```

- [ ] **Step 3: Run the two new tests and confirm they fail**

Run:

```bash
python -m pytest test/test_skinned_visual_to_template_mjcf.py::test_partition_triangles_by_triangle_majority_weight test/test_skinned_visual_to_template_mjcf.py::test_collapse_unmapped_joint_to_nearest_mapped_parent -v
```

Expected: FAIL because `skinned_mesh_partition.py` does not exist.

- [ ] **Step 4: Commit the red tests**

```bash
git add test/test_skinned_visual_to_template_mjcf.py
git commit -m "test: add partitioning expectations for visual swap"
```

### Task 4: Implement the skinned mesh data model and baking helpers

**Files:**
- Create: `lightwheel/srl/from_usd/skinned_mesh_partition.py`
- Modify: `test/test_skinned_visual_to_template_mjcf.py`

- [ ] **Step 1: Add one failing bake test**

Append this test:

```python
from lightwheel.srl.from_usd.skinned_mesh_partition import BakedMeshChunk, bake_chunk_to_body_frame


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
```

- [ ] **Step 2: Run the mapping and bake tests and confirm they fail**

Run:

```bash
python -m pytest test/test_skinned_visual_to_template_mjcf.py::test_partition_triangles_by_triangle_majority_weight test/test_skinned_visual_to_template_mjcf.py::test_collapse_unmapped_joint_to_nearest_mapped_parent test/test_skinned_visual_to_template_mjcf.py::test_bake_chunk_to_body_frame_transforms_points_into_target_local_frame -v
```

Expected: FAIL because the dataclasses and helper functions are missing.

- [ ] **Step 3: Implement `skinned_mesh_partition.py`**

Create the module with a clear data model:

```python
from dataclasses import dataclass

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


def collapse_joint_mapping_to_parent(...): ...
def load_skinned_mesh_data(...): ...
def partition_triangles_by_body(...): ...
def bake_chunk_to_body_frame(...): ...
```

Implementation requirements:

- Read the mesh only from `/World/biped_demo_meters` or an explicit mesh path argument.
- Ignore `/World/CharacterAnimation`.
- Resolve unmapped joints by walking to the nearest mapped parent.
- Partition faces by summed mapped weights over all vertices in the triangle.
- Preserve per-face material ownership.
- Convert baked points into target-body local coordinates with inverse target world transforms.

- [ ] **Step 4: Add one real-USD loader smoke test**

Append this test:

```python
from pathlib import Path

from lightwheel.srl.from_usd.skinned_mesh_partition import load_skinned_mesh_data


def test_load_skinned_mesh_data_reads_biped_usd_real_asset():
    usd_path = Path("/home/lyuxinghe/code/Characters/Biped_Setup.usd")
    if not usd_path.exists():
        pytest.skip("Real asset not available in this environment.")

    mesh = load_skinned_mesh_data(usd_path, mesh_path="/World/biped_demo_meters/Body_Mesh")

    assert mesh.points.shape[0] > 0
    assert mesh.face_vertex_counts.shape[0] > 0
    assert len(mesh.joint_names) == 81
```

- [ ] **Step 5: Run the chunk test set and confirm it passes**

Run:

```bash
python -m pytest test/test_skinned_visual_to_template_mjcf.py -k "partition or collapse or bake or load_skinned_mesh_data" -v
```

Expected: PASS, with the real-asset test either PASS or SKIP.

- [ ] **Step 6: Commit the partitioning module**

```bash
git add lightwheel/srl/from_usd/skinned_mesh_partition.py test/test_skinned_visual_to_template_mjcf.py
git commit -m "feat: add skinned mesh partitioning for template visual swap"
```

## Chunk 3: End-To-End Conversion, CLI, And Verification

### Task 5: Write the failing end-to-end converter test

**Files:**
- Create: `lightwheel/srl/from_usd/skinned_visual_to_template_mjcf.py`
- Modify: `test/test_skinned_visual_to_template_mjcf.py`

- [ ] **Step 1: Add a failing end-to-end test with synthetic data**

Append this test:

```python
from lightwheel.srl.from_usd.skinned_visual_to_template_mjcf import convert_skinned_visual_to_template_mjcf


def test_end_to_end_conversion_preserves_template_joint_and_motor_order(tmp_path: Path):
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
        visual_usd_path=Path("/home/lyuxinghe/code/Characters/Biped_Setup.usd"),
        output_dir=tmp_path / "out",
        joint_to_body_mapping={"Root": "Pelvis", "Pelvis": "Pelvis"},
        keep_template_visuals=False,
    )

    assert output_xml.exists()
    assert report.body_names == ["Pelvis"]
    assert "Pelvis" in report.exported_triangle_count_by_body
```

- [ ] **Step 2: Add the optional MuJoCo dimension verification test**

Append this test:

```python
def test_output_dimensions_match_template_when_mujoco_available(tmp_path: Path):
    mujoco = pytest.importorskip("mujoco")
    ...
    template_model = mujoco.MjModel.from_xml_path(template_path.as_posix())
    output_model = mujoco.MjModel.from_xml_path(output_xml.as_posix())

    assert output_model.nq == template_model.nq
    assert output_model.nv == template_model.nv
    assert output_model.nu == template_model.nu
```

- [ ] **Step 3: Run the end-to-end tests and confirm they fail**

Run:

```bash
python -m pytest test/test_skinned_visual_to_template_mjcf.py::test_end_to_end_conversion_preserves_template_joint_and_motor_order test/test_skinned_visual_to_template_mjcf.py::test_output_dimensions_match_template_when_mujoco_available -v
```

Expected: FAIL because the orchestrator module does not exist.

- [ ] **Step 4: Commit the red integration test**

```bash
git add test/test_skinned_visual_to_template_mjcf.py
git commit -m "test: add end-to-end visual swap expectations"
```

### Task 6: Implement the orchestrator and package export

**Files:**
- Create: `lightwheel/srl/from_usd/skinned_visual_to_template_mjcf.py`
- Modify: `lightwheel/srl/from_usd/__init__.py`
- Modify: `test/test_skinned_visual_to_template_mjcf.py`

- [ ] **Step 1: Implement the public conversion entrypoint**

Create `lightwheel/srl/from_usd/skinned_visual_to_template_mjcf.py` with the orchestration API:

```python
from dataclasses import dataclass
from pathlib import Path

from lightwheel.srl.from_usd.skinned_mesh_partition import ...
from lightwheel.srl.from_usd.template_mjcf_visual_swap import ...


@dataclass
class ConversionReport:
    body_names: list[str]
    joint_names: list[str]
    motor_names: list[str]
    exported_triangle_count_by_body: dict[str, int]
    forced_assignment_face_count: int
    empty_body_warnings: list[str]


def convert_skinned_visual_to_template_mjcf(
    template_mjcf_path: Path,
    template_usd_path: Path | None,
    visual_usd_path: Path,
    output_dir: Path,
    joint_to_body_mapping: dict[str, str],
    keep_template_visuals: bool = False,
) -> tuple[Path, ConversionReport]:
    ...
```

Implementation requirements:

- Keep template body, joint, and motor ordering untouched.
- Export OBJ assets under `output_dir / "visuals"`.
- Reuse current material-copy logic where practical.
- Return a `ConversionReport` used by tests and smoke runs.

- [ ] **Step 2: Export the public API**

Update `lightwheel/srl/from_usd/__init__.py` to expose the new entrypoint:

```python
from .skinned_visual_to_template_mjcf import convert_skinned_visual_to_template_mjcf
```

- [ ] **Step 3: Run the focused end-to-end tests and confirm they pass**

Run:

```bash
python -m pytest test/test_skinned_visual_to_template_mjcf.py::test_end_to_end_conversion_preserves_template_joint_and_motor_order -v
```

Expected: PASS.

- [ ] **Step 4: Commit the orchestrator**

```bash
git add lightwheel/srl/from_usd/skinned_visual_to_template_mjcf.py lightwheel/srl/from_usd/__init__.py test/test_skinned_visual_to_template_mjcf.py
git commit -m "feat: add end-to-end skinned visual swap pipeline"
```

### Task 7: Add the CLI wrapper and run full verification

**Files:**
- Create: `test/skinned_visual_to_template_mjcf_test.py`
- Modify: `test/test_skinned_visual_to_template_mjcf.py`

- [ ] **Step 1: Add the failing CLI smoke test**

Append this test:

```python
import subprocess
import sys


def test_cli_writes_output_xml(tmp_path: Path):
    cli_path = Path("test/skinned_visual_to_template_mjcf_test.py")
    assert cli_path.exists()
```

- [ ] **Step 2: Run the CLI smoke test and confirm it fails**

Run:

```bash
python -m pytest test/test_skinned_visual_to_template_mjcf.py::test_cli_writes_output_xml -v
```

Expected: FAIL because the CLI script does not exist.

- [ ] **Step 3: Implement the CLI wrapper**

Create `test/skinned_visual_to_template_mjcf_test.py`:

```python
import argparse
from pathlib import Path

from lightwheel.srl.from_usd.skinned_visual_to_template_mjcf import convert_skinned_visual_to_template_mjcf


def main():
    parser = argparse.ArgumentParser(description="Swap skinned USD visuals onto an MJCF template")
    parser.add_argument("--template-mjcf", required=True)
    parser.add_argument("--template-usd", default=None)
    parser.add_argument("--visual-usd", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--keep-template-visuals", action="store_true")
    args = parser.parse_args()
    ...


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run the full automated test suite**

Run:

```bash
python -m pytest test/test_skinned_visual_to_template_mjcf.py -v
```

Expected: PASS, with MuJoCo-specific checks allowed to SKIP.

- [ ] **Step 5: Run the real-asset smoke command**

Run:

```bash
/home/lyuxinghe/miniforge3/bin/conda run -n usd2mjcf python test/skinned_visual_to_template_mjcf_test.py \
  --template-mjcf /home/lyuxinghe/code/CLoSD/closd/data/robot_cache/smpl_humanoid_0.xml \
  --template-usd /home/lyuxinghe/code/human_motion_isaacsim/third_party/ProtoMotions/protomotions/data/assets/usd/smpl_humanoid.usda \
  --visual-usd /home/lyuxinghe/code/Characters/Biped_Setup.usd \
  --output-dir /tmp/biped_smpl_visual_swap
```

Expected:

- `/tmp/biped_smpl_visual_swap` contains a new XML file.
- `/tmp/biped_smpl_visual_swap/visuals` contains exported mesh assets.
- The reported body, joint, and motor names match the SMPL template exactly.

- [ ] **Step 6: Commit the CLI and verification pass**

```bash
git add test/skinned_visual_to_template_mjcf_test.py test/test_skinned_visual_to_template_mjcf.py
git commit -m "feat: add CLI for skinned visual swap conversion"
```

## Final Verification Checklist

- [ ] `python -m pytest test/test_skinned_visual_to_template_mjcf.py -v`
- [ ] Real-asset smoke conversion succeeds in the `usd2mjcf` conda env.
- [ ] Output XML keeps the SMPL body, joint, and motor ordering exactly.
- [ ] Output XML keeps original physics geoms and hides them visually instead of removing them.
- [ ] New Biped geoms are mesh-only visuals with zero contact participation.
- [ ] Triangle-count report does not show empty major visible bodies.

## Handoff Notes

- Keep the existing `UsdToMjcf` rigid-body converter untouched.
- Do not pull logic from the stale `CLoSD/scripts` converters.
- If the real-asset smoke test reveals a missing texture or material edge case, fix that inside the new pipeline instead of weakening the invariant checks.
- If `mujoco` bindings are unavailable, keep the structural XML checks mandatory and let the model-dimension check skip.
