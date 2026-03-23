# `skinned_visual_to_template_mjcf_test.py`

This script swaps a skinned USD character onto an existing MJCF physics template.
It is not the generic direct USD-to-MJCF converter; it preserves the template
MJCF physics structure and only replaces the visible body meshes.

Current scope:

- It is built for the `Biped_Setup.usd` to `smpl_humanoid_0.xml` workflow.
- It keeps the SMPL MJCF body tree, joints, motors, and physics geoms.
- It adds Biped-derived OBJ visual meshes onto those existing SMPL bodies.
- It uses the default Biped-to-SMPL joint mapping baked into the script.
- It can optionally emit a ProtoMotions-ready bundle under the output directory.

## Command

Run it from the repo root:

```bash
/home/lyuxinghe/miniforge3/bin/conda run -n usd2mjcf python test/skinned_visual_to_template_mjcf_test.py \
  --template-mjcf /home/lyuxinghe/code/CLoSD/closd/data/robot_cache/smpl_humanoid_0.xml \
  --template-usd /home/lyuxinghe/code/human_motion_isaacsim/third_party/ProtoMotions/protomotions/data/assets/usd/smpl_humanoid.usda \
  --visual-usd /home/lyuxinghe/code/Characters/Biped_Setup.usd \
  --output-dir /home/lyuxinghe/code/Characters_mjcf/Biped
```

To also emit a ProtoMotions-ready bundle:

```bash
/home/lyuxinghe/miniforge3/bin/conda run -n usd2mjcf python test/skinned_visual_to_template_mjcf_test.py \
  --template-mjcf /home/lyuxinghe/code/CLoSD/closd/data/robot_cache/smpl_humanoid_0.xml \
  --template-usd /home/lyuxinghe/code/human_motion_isaacsim/third_party/ProtoMotions/protomotions/data/assets/usd/smpl_humanoid.usda \
  --visual-usd /home/lyuxinghe/code/Characters/Biped_Setup.usd \
  --output-dir /home/lyuxinghe/code/Characters_mjcf/Biped \
  --emit-protomotions-bundle \
  --robot-name biped_smpl_visual_swap \
  --protomotions-root /home/lyuxinghe/code/ProtoMotions \
  --protomotions-python /path/to/isaaclab-enabled/python
```

## Arguments

- `--template-mjcf`: The MJCF physics template to preserve.
- `--template-usd`: The matching USD template. It is accepted for interface compatibility, although the current implementation does not need it yet.
- `--visual-usd`: The skinned USD visual source.
- `--output-dir`: Directory where the converted MJCF and exported OBJ files are written.
- `--keep-template-visuals`: Optional flag. If set, the original template geoms are left visible instead of being hidden with transparent `rgba`.
- `--emit-protomotions-bundle`: Optional flag. If set, also emit a ProtoMotions-ready bundle under `<output-dir>/protomotions_bundle`.
- `--robot-name`: Required with `--emit-protomotions-bundle`. Controls the generated ProtoMotions asset and config names.
- `--protomotions-root`: Required with `--emit-protomotions-bundle`. Path to a ProtoMotions checkout containing `data/scripts/convert_mjcf_to_usd.py`.
- `--protomotions-python`: Optional path to the Python executable that should run ProtoMotions' MJCF-to-USD converter. Use this when the current interpreter does not have Isaac Lab installed.

## Output

The script writes:

- `<output-dir>/<template-name>_visual_swap.xml`
- `<output-dir>/visuals/*.obj`

When `--emit-protomotions-bundle` is used, it also writes:

- `<output-dir>/protomotions_bundle/assets/mjcf/<robot-name>.xml`
- `<output-dir>/protomotions_bundle/assets/mjcf/visuals/*.obj`
- `<output-dir>/protomotions_bundle/assets/usd/<robot-name>/<robot-name>.usda`
- `<output-dir>/protomotions_bundle/assets/usd/<robot-name>/config.yaml`
- `<output-dir>/protomotions_bundle/robot_configs/<robot-name>.py`
- `<output-dir>/protomotions_bundle/factory_registration_snippet.py`
- `<output-dir>/protomotions_bundle/install_instructions.md`
- `<output-dir>/protomotions_bundle/manifest.json`

It also prints:

- `output_xml=...`
- `body_names=[...]`
- `joint_names=[...]`
- `motor_names=[...]`

When bundle export is enabled, it also prints:

- `protomotions_bundle_dir=...`
- `protomotions_mjcf_path=...`
- `protomotions_usd_path=...`
- `protomotions_manifest_path=...`

## What It Preserves

- The SMPL body hierarchy
- The SMPL joint names and order
- The SMPL motor names and order
- The original SMPL physics geoms

## What It Changes

- Adds new mesh geoms for Biped visuals
- Hides the original template physics geoms visually unless `--keep-template-visuals` is used

## Registering In ProtoMotions

After generating the bundle, copy the assets into your ProtoMotions checkout:

```bash
cp /path/to/output/protomotions_bundle/assets/mjcf/biped_smpl_visual_swap.xml \
  /home/lyuxinghe/code/ProtoMotions/protomotions/data/assets/mjcf/

mkdir -p /home/lyuxinghe/code/ProtoMotions/protomotions/data/assets/usd/biped_smpl_visual_swap
cp -r /path/to/output/protomotions_bundle/assets/usd/biped_smpl_visual_swap/* \
  /home/lyuxinghe/code/ProtoMotions/protomotions/data/assets/usd/biped_smpl_visual_swap/

cp /path/to/output/protomotions_bundle/robot_configs/biped_smpl_visual_swap.py \
  /home/lyuxinghe/code/ProtoMotions/protomotions/robot_configs/
```

Then open:

- `protomotions_bundle/factory_registration_snippet.py`

and paste that `elif robot_name == "...":` block into:

- `ProtoMotions/protomotions/robot_configs/factory.py`

Smoke-test the registration:

```bash
cd /home/lyuxinghe/code/ProtoMotions
python examples/tutorial/2_load_robot.py --robot biped_smpl_visual_swap --simulator isaacgym
```

If you are using Isaac Lab and the generated USD asset is available:

```bash
cd /home/lyuxinghe/code/ProtoMotions
python examples/tutorial/2_load_robot.py --robot biped_smpl_visual_swap --simulator isaaclab
```

## Current Limitations

- The mapping is currently hard-coded for the Biped-to-SMPL swap path.
- Material and texture export is not complete yet; the script currently writes per-body OBJ geometry and injects mesh geoms, but does not yet rebuild a full MTL/texture asset pipeline.
- `Biped_Setup.usd` references animation payloads under `/World/CharacterAnimation`. If those payload files are missing, USD may print warnings during loading. The conversion still succeeds as long as `/World/biped_demo_meters` is present.
- The optional MuJoCo dimension check in the test suite is skipped unless the `mujoco` Python package is installed in the `usd2mjcf` environment.
- ProtoMotions bundle export requires an environment that can run ProtoMotions' `data/scripts/convert_mjcf_to_usd.py` converter. If Isaac Lab dependencies are unavailable, bundle export will fail instead of emitting an incomplete USD bundle.
- By default, bundle export runs the ProtoMotions converter with the current Python interpreter. If your `usd2mjcf` environment does not contain Isaac Lab, pass `--protomotions-python /path/to/isaaclab-enabled/python`.

## Files

- Script: `test/skinned_visual_to_template_mjcf_test.py`
- Conversion core: `lightwheel/srl/from_usd/skinned_visual_to_template_mjcf.py`
- ProtoMotions bundle helpers: `lightwheel/srl/from_usd/protomotions_bundle_export.py`
- Template XML helpers: `lightwheel/srl/from_usd/template_mjcf_visual_swap.py`
- Skinning and partition helpers: `lightwheel/srl/from_usd/skinned_mesh_partition.py`
