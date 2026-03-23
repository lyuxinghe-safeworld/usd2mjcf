# ProtoMotions Bundle Export Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the skinned visual swap converter so one command can emit a ProtoMotions-ready bundle containing the swapped MJCF, ProtoMotions robot-config scaffolding, a factory registration snippet, install instructions, a manifest, and a USD sidecar generated from the bundled MJCF.

**Architecture:** Keep the existing visual-swap pipeline as the source of truth for MJCF generation, then add a separate bundle-export layer that copies the resulting MJCF into a ProtoMotions-style asset tree and writes the extra metadata files. Use ProtoMotions' own MJCF-to-USD conversion script as a subprocess for Isaac Lab USD generation so this repo does not reimplement that conversion.

**Tech Stack:** Python, `pathlib`, `json`, `subprocess`, `xml.etree.ElementTree`, `pytest`, USD/PXR, existing `usd2mjcf` conversion helpers, ProtoMotions MJCF-to-USD script.

---

## File Structure

- Create: `lightwheel/srl/from_usd/protomotions_bundle_export.py`
  Responsibility: build the ProtoMotions bundle layout, generate metadata files, invoke ProtoMotions MJCF-to-USD conversion, and return bundle paths.
- Modify: `lightwheel/srl/from_usd/skinned_visual_to_template_mjcf.py`
  Responsibility: extend the top-level conversion API to optionally emit the ProtoMotions bundle and include bundle metadata in the report.
- Modify: `lightwheel/srl/from_usd/__init__.py`
  Responsibility: export any new bundle-export entrypoints needed by tests or CLI callers.
- Modify: `test/skinned_visual_to_template_mjcf_test.py`
  Responsibility: extend the CLI with ProtoMotions bundle flags and report bundle outputs.
- Modify: `test/test_skinned_visual_to_template_mjcf.py`
  Responsibility: add TDD coverage for bundle files, manifest contents, generated config text, and CLI behavior.
- Modify: `test/README_skinned_visual_to_template_mjcf.md`
  Responsibility: document bundle generation and how to register the generated model in ProtoMotions.

## Chunk 1: Bundle Export Core

### Task 1: Add failing tests for bundle artifact generation

**Files:**
- Modify: `test/test_skinned_visual_to_template_mjcf.py`
- Test: `test/test_skinned_visual_to_template_mjcf.py`

- [ ] **Step 1: Write a failing test for ProtoMotions bundle file layout**

```python
def test_bundle_export_writes_expected_files(tmp_path):
    output_xml, report = convert_skinned_visual_to_template_mjcf(
        template_mjcf_path=TEMPLATE_MJCF_PATH,
        template_usd_path=TEMPLATE_USD_PATH,
        visual_usd_path=VISUAL_USD_PATH,
        output_dir=tmp_path,
        joint_to_body_mapping=DEFAULT_BIPED_TO_SMPL_MAPPING,
        emit_protomotions_bundle=True,
        robot_name="biped_smpl_visual_swap",
        protomotions_root=PROTOMOTIONS_ROOT,
    )

    bundle_dir = tmp_path / "protomotions_bundle"
    assert (bundle_dir / "assets/mjcf/biped_smpl_visual_swap.xml").is_file()
    assert (bundle_dir / "robot_configs/biped_smpl_visual_swap.py").is_file()
    assert (bundle_dir / "factory_registration_snippet.py").is_file()
    assert (bundle_dir / "install_instructions.md").is_file()
    assert (bundle_dir / "manifest.json").is_file()
```

- [ ] **Step 2: Run the new test and verify it fails**

Run: `python -m pytest test/test_skinned_visual_to_template_mjcf.py::test_bundle_export_writes_expected_files -v`

Expected: FAIL because `emit_protomotions_bundle` and bundle files do not exist yet.

- [ ] **Step 3: Create the bundle exporter module**

Implement `lightwheel/srl/from_usd/protomotions_bundle_export.py` with:

```python
@dataclass
class ProtoMotionsBundlePaths:
    bundle_dir: Path
    bundled_mjcf_path: Path
    usd_dir: Path
    usd_path: Path
    usd_config_path: Path
    robot_config_path: Path
    factory_snippet_path: Path
    install_instructions_path: Path
    manifest_path: Path


def export_protomotions_bundle(... ) -> ProtoMotionsBundlePaths:
    ...
```

Key behaviors:
- Create the bundle directory tree.
- Copy the generated MJCF to `assets/mjcf/<robot_name>.xml`.
- Write `robot_configs/<robot_name>.py`.
- Write `factory_registration_snippet.py`.
- Write `install_instructions.md`.
- Write `manifest.json`.

- [ ] **Step 4: Add minimal string/template helpers in the new module**

Implement focused helpers:

```python
def _robot_config_python(robot_name: str, asset_file_name: str, usd_asset_file_name: str) -> str: ...
def _factory_registration_snippet(robot_name: str, class_name: str) -> str: ...
def _install_instructions_markdown(robot_name: str) -> str: ...
def _manifest_payload(...) -> dict: ...
```

- [ ] **Step 5: Wire the bundle exporter into the main conversion function**

Modify `lightwheel/srl/from_usd/skinned_visual_to_template_mjcf.py`:

```python
def convert_skinned_visual_to_template_mjcf(
    ...,
    emit_protomotions_bundle: bool = False,
    robot_name: str | None = None,
    protomotions_root: Path | None = None,
) -> tuple[Path, ConversionReport]:
    ...
```

Add validation:
- `robot_name` required when `emit_protomotions_bundle=True`
- `protomotions_root` required when `emit_protomotions_bundle=True`

- [ ] **Step 6: Extend the conversion report with bundle metadata**

Update the report dataclass so tests can assert on bundle outputs:

```python
@dataclass
class ConversionReport:
    ...
    protomotions_bundle_dir: Path | None = None
    protomotions_mjcf_path: Path | None = None
    protomotions_usd_path: Path | None = None
    protomotions_manifest_path: Path | None = None
```

- [ ] **Step 7: Run the focused test again**

Run: `python -m pytest test/test_skinned_visual_to_template_mjcf.py::test_bundle_export_writes_expected_files -v`

Expected: PASS for file layout, but likely further failures for missing USD generation or manifest assertions in later tests.

- [ ] **Step 8: Commit the core bundle-export scaffold**

```bash
git add lightwheel/srl/from_usd/protomotions_bundle_export.py \
        lightwheel/srl/from_usd/skinned_visual_to_template_mjcf.py \
        test/test_skinned_visual_to_template_mjcf.py
git commit -m "feat: add ProtoMotions bundle export scaffolding"
```

### Task 2: Add strict ProtoMotions USD generation

**Files:**
- Modify: `lightwheel/srl/from_usd/protomotions_bundle_export.py`
- Modify: `test/test_skinned_visual_to_template_mjcf.py`
- Test: `test/test_skinned_visual_to_template_mjcf.py`

- [ ] **Step 1: Write a failing test for USD conversion invocation**

```python
def test_bundle_export_invokes_protomotions_mjcf_to_usd(monkeypatch, tmp_path):
    calls = []

    def fake_run(cmd, check):
        calls.append(cmd)
        expected_output = tmp_path / "protomotions_bundle/assets/usd/test_robot/test_robot.usda"
        expected_output.parent.mkdir(parents=True, exist_ok=True)
        expected_output.write_text("#usda 1.0\n", encoding="utf-8")
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(subprocess, "run", fake_run)
    ...
    assert "convert_mjcf_to_usd.py" in " ".join(calls[0])
```

- [ ] **Step 2: Run the new USD test and verify it fails**

Run: `python -m pytest test/test_skinned_visual_to_template_mjcf.py::test_bundle_export_invokes_protomotions_mjcf_to_usd -v`

Expected: FAIL because no subprocess-based USD generation exists yet.

- [ ] **Step 3: Implement ProtoMotions MJCF-to-USD subprocess orchestration**

Add a focused helper:

```python
def _generate_usd_with_protomotions(
    protomotions_root: Path,
    mjcf_path: Path,
    usd_path: Path,
) -> None:
    script_path = protomotions_root / "data/scripts/convert_mjcf_to_usd.py"
    subprocess.run([...], check=True)
```

Implementation notes:
- Validate `script_path` exists before launching.
- Write USD output into `assets/usd/<robot_name>/<robot_name>.usda`.
- Fail with a clear `ValueError` or `RuntimeError` if prerequisites are missing.

- [ ] **Step 4: Write the USD sidecar `config.yaml`**

Emit a file shaped like ProtoMotions' existing generated configs:

```yaml
asset_path: <bundled mjcf absolute path>
usd_dir: <usd parent dir absolute path>
usd_file_name: <robot_name>.usda
force_usd_conversion: true
make_instanceable: true
import_inertia_tensor: true
fix_base: false
import_sites: false
self_collision: false
link_density: 0.0
```

- [ ] **Step 5: Add a failing test for strict failure behavior**

```python
def test_bundle_export_requires_protomotions_root_for_bundle(tmp_path):
    with pytest.raises(ValueError):
        convert_skinned_visual_to_template_mjcf(
            ...,
            emit_protomotions_bundle=True,
            robot_name="test_robot",
            protomotions_root=None,
        )
```

- [ ] **Step 6: Run the targeted bundle tests**

Run:

```bash
python -m pytest test/test_skinned_visual_to_template_mjcf.py \
  -k "bundle_export or protomotions_root" -v
```

Expected: PASS for new bundle-path and failure-behavior tests.

- [ ] **Step 7: Commit USD bundle generation**

```bash
git add lightwheel/srl/from_usd/protomotions_bundle_export.py \
        lightwheel/srl/from_usd/skinned_visual_to_template_mjcf.py \
        test/test_skinned_visual_to_template_mjcf.py
git commit -m "feat: generate ProtoMotions USD bundle assets"
```

## Chunk 2: CLI and Surface Area

### Task 3: Extend the CLI for ProtoMotions bundle generation

**Files:**
- Modify: `test/skinned_visual_to_template_mjcf_test.py`
- Modify: `lightwheel/srl/from_usd/__init__.py`
- Modify: `test/test_skinned_visual_to_template_mjcf.py`
- Test: `test/test_skinned_visual_to_template_mjcf.py`

- [ ] **Step 1: Write a failing CLI test for bundle flags**

```python
def test_cli_help_includes_protomotions_bundle_flags():
    completed = subprocess.run(
        [sys.executable, "test/skinned_visual_to_template_mjcf_test.py", "--help"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    assert "--emit-protomotions-bundle" in completed.stdout
    assert "--robot-name" in completed.stdout
    assert "--protomotions-root" in completed.stdout
```

- [ ] **Step 2: Run the CLI help test and verify it fails**

Run: `python -m pytest test/test_skinned_visual_to_template_mjcf.py::test_cli_help_includes_protomotions_bundle_flags -v`

Expected: FAIL because the flags are not in argparse yet.

- [ ] **Step 3: Add the new CLI flags**

Update `test/skinned_visual_to_template_mjcf_test.py` to parse:
- `--emit-protomotions-bundle`
- `--robot-name`
- `--protomotions-root`

Pass them through to `convert_skinned_visual_to_template_mjcf(...)`.

- [ ] **Step 4: Print bundle paths in the CLI output**

Add optional prints:

```python
if report.protomotions_bundle_dir is not None:
    print(f"protomotions_bundle_dir={report.protomotions_bundle_dir}")
    print(f"protomotions_mjcf_path={report.protomotions_mjcf_path}")
    print(f"protomotions_usd_path={report.protomotions_usd_path}")
```

- [ ] **Step 5: Re-export the public entrypoint if needed**

Update `lightwheel/srl/from_usd/__init__.py` so tests and future callers can
import the main conversion function without reaching into private modules.

- [ ] **Step 6: Run the CLI tests**

Run:

```bash
python -m pytest test/test_skinned_visual_to_template_mjcf.py \
  -k "cli_help" -v
```

Expected: PASS for the help/CLI coverage.

- [ ] **Step 7: Commit the CLI extension**

```bash
git add test/skinned_visual_to_template_mjcf_test.py \
        lightwheel/srl/from_usd/__init__.py \
        test/test_skinned_visual_to_template_mjcf.py
git commit -m "feat: add ProtoMotions bundle CLI options"
```

## Chunk 3: Documentation and End-to-End Verification

### Task 4: Update README and registration instructions

**Files:**
- Modify: `test/README_skinned_visual_to_template_mjcf.md`
- Test: manual doc verification

- [ ] **Step 1: Update the README command example to include bundle flags**

Add a full example:

```bash
/home/lyuxinghe/miniforge3/bin/conda run -n usd2mjcf python test/skinned_visual_to_template_mjcf_test.py \
  --template-mjcf ... \
  --template-usd ... \
  --visual-usd ... \
  --output-dir ... \
  --emit-protomotions-bundle \
  --robot-name biped_smpl_visual_swap \
  --protomotions-root /home/lyuxinghe/code/ProtoMotions
```

- [ ] **Step 2: Document ProtoMotions registration**

Describe how to:
- copy `assets/mjcf/<robot_name>.xml` into `protomotions/data/assets/mjcf/`
- copy `assets/usd/<robot_name>/` into `protomotions/data/assets/usd/<robot_name>/`
- copy `robot_configs/<robot_name>.py` into `protomotions/robot_configs/`
- paste `factory_registration_snippet.py` into `protomotions/robot_configs/factory.py`

- [ ] **Step 3: Add a smoke-test command for ProtoMotions**

Document:

```bash
python examples/tutorial/2_load_robot.py --robot biped_smpl_visual_swap --simulator isaacgym
```

and, for Isaac Lab if available:

```bash
python examples/tutorial/2_load_robot.py --robot biped_smpl_visual_swap --simulator isaaclab
```

- [ ] **Step 4: Commit the README update**

```bash
git add test/README_skinned_visual_to_template_mjcf.md
git commit -m "docs: add ProtoMotions bundle registration guide"
```

### Task 5: Run final verification

**Files:**
- Verify: `lightwheel/srl/from_usd/protomotions_bundle_export.py`
- Verify: `lightwheel/srl/from_usd/skinned_visual_to_template_mjcf.py`
- Verify: `test/skinned_visual_to_template_mjcf_test.py`
- Verify: `test/test_skinned_visual_to_template_mjcf.py`
- Verify: `test/README_skinned_visual_to_template_mjcf.md`

- [ ] **Step 1: Run the focused test module**

Run:

```bash
/home/lyuxinghe/miniforge3/bin/conda run -n usd2mjcf \
  python -m pytest test/test_skinned_visual_to_template_mjcf.py -v
```

Expected: all non-environment-gated tests pass.

- [ ] **Step 2: Run the CLI bundle command**

Run:

```bash
/home/lyuxinghe/miniforge3/bin/conda run -n usd2mjcf python test/skinned_visual_to_template_mjcf_test.py \
  --template-mjcf /home/lyuxinghe/code/CLoSD/closd/data/robot_cache/smpl_humanoid_0.xml \
  --template-usd /home/lyuxinghe/code/human_motion_isaacsim/third_party/ProtoMotions/protomotions/data/assets/usd/smpl_humanoid.usda \
  --visual-usd /home/lyuxinghe/code/Characters/Biped_Setup.usd \
  --output-dir /tmp/biped_smpl_visual_swap_bundle \
  --emit-protomotions-bundle \
  --robot-name biped_smpl_visual_swap \
  --protomotions-root /home/lyuxinghe/code/ProtoMotions
```

Expected:
- exit code `0`
- generated swapped MJCF under `/tmp/biped_smpl_visual_swap_bundle`
- generated `protomotions_bundle/` tree with MJCF, USD, manifest, config module, snippet, and instructions

- [ ] **Step 3: If the environment lacks Isaac Lab conversion support, state that explicitly**

Record the exact missing dependency or failing converter command instead of
claiming full bundle success.

- [ ] **Step 4: Commit the finished feature**

```bash
git add lightwheel/srl/from_usd/protomotions_bundle_export.py \
        lightwheel/srl/from_usd/skinned_visual_to_template_mjcf.py \
        lightwheel/srl/from_usd/__init__.py \
        test/skinned_visual_to_template_mjcf_test.py \
        test/test_skinned_visual_to_template_mjcf.py \
        test/README_skinned_visual_to_template_mjcf.md
git commit -m "feat: export ProtoMotions-ready visual swap bundles"
```
