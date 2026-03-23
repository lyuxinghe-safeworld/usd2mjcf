# ProtoMotions Bundle Export Design

## Goal

Extend the existing skinned-visual-to-template-MJCF conversion flow so a single
CLI command can emit a self-contained ProtoMotions-ready bundle in the chosen
output directory.

The bundle must preserve the existing SMPL physics/control assumptions while
swapping the visible character mesh to `Biped_Setup.usd`, and it must not write
directly into the ProtoMotions checkout.

## Context

The current converter in this repo builds a visual-swapped MJCF by keeping the
SMPL template body tree, joint ordering, motor ordering, and physics geoms while
replacing the visible meshes with body-local OBJ assets derived from a skinned
USD mesh.

ProtoMotions custom-robot support requires, at minimum:

1. An MJCF asset.
2. A robot config module.
3. A factory registration entry.

For Isaac Lab usage, ProtoMotions also expects:

1. A USD asset path (`usd_asset_file_name`).
2. A `usd_bodies_root_prim_path`.
3. A generated USD asset that matches the MJCF.

The pretrained SMPL motion-tracker path resolves to Isaac Lab and expects both
MJCF and USD asset paths, so the bundle exporter must treat USD generation as a
first-class deliverable.

## Requirements

### Functional Requirements

The updated CLI must:

1. Continue generating the visual-swapped MJCF and per-body OBJ meshes.
2. Optionally emit a ProtoMotions bundle under the conversion output directory.
3. Copy the generated MJCF into a ProtoMotions-style asset layout.
4. Generate a robot-config Python module for the new robot name.
5. Generate a factory registration snippet for manual paste into ProtoMotions.
6. Generate install instructions for copying the bundle into a ProtoMotions
   checkout.
7. Generate a machine-readable manifest capturing conversion metadata.
8. Generate an Isaac Lab USD asset and matching `config.yaml` beside it when the
   ProtoMotions bundle export option is used.

### Non-Functional Requirements

1. The ProtoMotions bundle export must not mutate files under the ProtoMotions
   repository.
2. The exported robot config must preserve SMPL controller compatibility by
   reusing SMPL naming/control assumptions.
3. Failure behavior must be strict: incomplete ProtoMotions bundles should not be
   emitted silently.
4. Existing non-bundled CLI usage must remain valid.

## Proposed Approach

### High-Level Flow

The CLI remains the single entrypoint.

When bundle export is disabled:

1. Run the current visual-swap conversion.
2. Write the MJCF plus OBJ visuals to the chosen output directory.

When bundle export is enabled:

1. Run the current visual-swap conversion.
2. Create `protomotions_bundle/` inside the chosen output directory.
3. Copy the final MJCF to `protomotions_bundle/assets/mjcf/<robot_name>.xml`.
4. Generate `protomotions_bundle/robot_configs/<robot_name>.py`.
5. Generate `protomotions_bundle/factory_registration_snippet.py`.
6. Generate `protomotions_bundle/install_instructions.md`.
7. Generate `protomotions_bundle/manifest.json`.
8. Invoke ProtoMotions' MJCF-to-USD conversion script against the bundled MJCF.
9. Write the generated USD to
   `protomotions_bundle/assets/usd/<robot_name>/<robot_name>.usda`.
10. Write the matching USD sidecar config to
    `protomotions_bundle/assets/usd/<robot_name>/config.yaml`.

### Generated Bundle Layout

```text
<output-dir>/
  smpl_humanoid_0_visual_swap.xml
  visuals/
    *.obj
  protomotions_bundle/
    manifest.json
    install_instructions.md
    factory_registration_snippet.py
    assets/
      mjcf/
        <robot_name>.xml
      usd/
        <robot_name>/
          <robot_name>.usda
          config.yaml
    robot_configs/
      <robot_name>.py
```

## Detailed Design

### Robot Config Strategy

The generated ProtoMotions robot config should be a thin subclass of
`SmplRobotConfig`, not a full inlined copy of the existing SMPL config.

Reasons:

1. The pretrained controller is trained against the SMPL kinematic and control
   structure.
2. The visual-swap path intentionally preserves the SMPL MJCF structure.
3. Subclassing minimizes drift from ProtoMotions' own SMPL configuration.

The generated subclass should override only the asset-path-facing fields and keep
the rest of the SMPL behavior unchanged.

### USD Generation Strategy

ProtoMotions already ships an MJCF-to-USD converter script for Isaac Lab usage.
This exporter should orchestrate that script rather than reimplement Isaac Lab
USD conversion logic inside this repo.

The bundle exporter should therefore accept a `protomotions_root` path and use it
to call ProtoMotions' conversion script in a subprocess, but always point the
output USD path into the conversion output directory.

### Manifest Contents

The manifest should record:

1. Robot name.
2. Source visual USD path.
3. Template MJCF path.
4. Template USD path, if provided.
5. Generated output MJCF path.
6. Bundled ProtoMotions MJCF path.
7. Bundled ProtoMotions USD path.
8. Body names.
9. Joint names.
10. Motor names.
11. Exported triangle counts by body.
12. The resolved joint-to-body mapping.

### Failure Behavior

When ProtoMotions bundle export is enabled, the command should fail if:

1. USD generation cannot be invoked.
2. USD conversion exits non-zero.
3. The generated MJCF does not preserve template body/joint/motor ordering.
4. Any required bundle artifact cannot be written.

No partial success should be reported for bundle export.

## CLI Changes

Add the following CLI options:

1. `--emit-protomotions-bundle`
2. `--robot-name <name>`
3. `--protomotions-root <path>`

Behavior:

1. `--emit-protomotions-bundle` enables bundle generation.
2. `--robot-name` controls generated filenames and the class/snippet content.
3. `--protomotions-root` is required when bundle export is enabled.

## Testing Strategy

### Unit and Integration Coverage

Add tests that verify:

1. Bundle files are written to the expected relative paths.
2. The generated robot-config file contains the expected asset path overrides.
3. The generated factory snippet references the requested robot name.
4. The generated manifest matches the conversion report.
5. The CLI accepts the new bundle-related flags.

### Optional Environment-Dependent Coverage

Add an environment-gated test that exercises real ProtoMotions USD conversion
only when the required external tooling is available.

## Out of Scope

The following are intentionally excluded:

1. Auto-patching the ProtoMotions checkout.
2. Auto-registering the generated robot in ProtoMotions.
3. Retargeting scripts or motion-conversion scripts for the new robot.
4. Reworking SMPL control gains or body naming.
5. Generalizing the converter to arbitrary character rigs beyond the existing
   Biped-to-SMPL workflow.
