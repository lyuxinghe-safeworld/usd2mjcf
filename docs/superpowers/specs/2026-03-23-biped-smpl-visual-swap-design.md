# Biped-to-SMPL Visual Swap Design

## Goal

Convert `/home/lyuxinghe/code/Characters/Biped_Setup.usd` into an MJCF that preserves the SMPL physics skeleton, joint names, motor names, hierarchy, and dynamics from `/home/lyuxinghe/code/CLoSD/closd/data/robot_cache/smpl_humanoid_0.xml` exactly, while replacing the rendered appearance with visuals derived from the Biped USD.

The converted MJCF must work with the existing downstream motion-tracking controller and loading code without any runtime code changes.

## Constraints

- Keep the SMPL physics model unchanged.
- Do not add or remove bodies, joints, degrees of freedom, or actuators.
- Do not rename any SMPL body, joint, or motor.
- Do not modify collision shapes, inertials, joint axes, joint limits, damping, stiffness, armature, or actuator gears.
- Unmatched Biped skeleton parts must not affect physics.
- Do not rely on `/home/lyuxinghe/code/CLoSD/scripts/convert_usd_to_mjcf.py`.
- Do not rely on `/home/lyuxinghe/code/CLoSD/scripts/usd_mesh_to_mjcf_visual.py`.

## Source Model Facts

### SMPL template

- `/home/lyuxinghe/code/human_motion_isaacsim/third_party/ProtoMotions/protomotions/data/assets/usd/smpl_humanoid.usda` and `/home/lyuxinghe/code/CLoSD/closd/data/robot_cache/smpl_humanoid_0.xml` describe the same articulated template.
- The SMPL XML contains a fixed body tree rooted at `Pelvis`, followed by the expected bodies:
  - `L_Hip`, `L_Knee`, `L_Ankle`, `L_Toe`
  - `R_Hip`, `R_Knee`, `R_Ankle`, `R_Toe`
  - `Torso`, `Spine`, `Chest`, `Neck`, `Head`
  - `L_Thorax`, `L_Shoulder`, `L_Elbow`, `L_Wrist`, `L_Hand`
  - `R_Thorax`, `R_Shoulder`, `R_Elbow`, `R_Wrist`, `R_Hand`
- The SMPL XML contains 69 hinge joints and 69 matching motors.
- The existing SMPL `<geom>` elements are physics geoms, not disposable visual geoms. They carry `density`, `contype`, and `conaffinity`, so they must remain in the output for collision and inertia.

### Biped visual source

- `/home/lyuxinghe/code/Characters/Biped_Setup.usd` is a `UsdSkel` asset, not a physics-authored USD.
- It contains one `UsdSkel.Root`, one `UsdSkel.Skeleton`, and one skinned mesh at `/World/biped_demo_meters/Body_Mesh`.
- The mesh uses standard skinning primvars:
  - `primvars:skel:jointIndices`
  - `primvars:skel:jointWeights`
- The Biped skeleton contains 81 joints, including torso segments, twist bones, and finger chains.
- The file also references animation payloads under `/World/CharacterAnimation/Animation`; these are not needed for visual extraction and should be ignored by this pipeline.

## Chosen Approach

Use per-body visual partitioning onto the fixed SMPL physics template.

Alternatives that were rejected:

- Attach the full Biped mesh to one body, such as `Pelvis`.
  - Stable but visually poor.
- Rebuild the Biped as a new articulated MJCF.
  - Highest risk because it would break controller assumptions.

The chosen approach keeps the controller-facing MJCF structure identical to SMPL and only swaps visuals.

## Approved Joint-to-Body Mapping

The Biped skeleton is mapped onto the existing SMPL bodies as follows:

- `Root`, `Pelvis` -> `Pelvis`
- `L_UpLeg`, `L_UpLegTwist1` -> `L_Hip`
- `L_LoLeg`, `L_LoLegTwist1` -> `L_Knee`
- `L_Ankle` -> `L_Ankle`
- `L_Ball` -> `L_Toe`
- `R_UpLeg`, `R_UpLegTwist1` -> `R_Hip`
- `R_LoLeg`, `R_LoLegTwist1` -> `R_Knee`
- `R_Ankle` -> `R_Ankle`
- `R_Ball` -> `R_Toe`
- `Spine1` -> `Torso`
- `Spine2` -> `Spine`
- `Spine3`, `Chest` -> `Chest`
- `Neck1`, `Neck2` -> `Neck`
- `Head` -> `Head`
- `L_Clavicle` -> `L_Thorax`
- `L_UpArm`, `L_UpArmTwist1` -> `L_Shoulder`
- `L_LoArm`, `L_LoArmTwist1` -> `L_Elbow`
- `L_Wrist` -> `L_Wrist`
- all left finger joints -> `L_Hand`
- `R_Clavicle` -> `R_Thorax`
- `R_UpArm`, `R_UpArmTwist1` -> `R_Shoulder`
- `R_LoArm`, `R_LoArmTwist1` -> `R_Elbow`
- `R_Wrist` -> `R_Wrist`
- all right finger joints -> `R_Hand`

Unmatched or extra Biped joints never create new MJCF bodies or joints. They are collapsed into the nearest approved mapped body.

## Conversion Flow

This conversion is a template-preserving visual swap, not a rigid-body USD-to-MJCF conversion.

1. Read the SMPL MJCF template from `/home/lyuxinghe/code/CLoSD/closd/data/robot_cache/smpl_humanoid_0.xml`.
2. Read the corresponding SMPL USD from `/home/lyuxinghe/code/human_motion_isaacsim/third_party/ProtoMotions/protomotions/data/assets/usd/smpl_humanoid.usda`.
3. Read the Biped visual source from `/home/lyuxinghe/code/Characters/Biped_Setup.usd`, but only use the skinned character subtree under `/World/biped_demo_meters`.
4. Load the Biped mesh, joint list, bind transforms, joint indices, joint weights, normals, UVs, and material assignments.
5. Apply the approved joint-to-body mapping so every Biped influence resolves to one existing SMPL body.
6. Partition the Biped mesh into visual chunks by SMPL body.
7. Bake each chunk into the local frame of its target SMPL body in the SMPL rest pose.
8. Clone the SMPL MJCF template.
9. Preserve all original SMPL physics geoms, but make them visually transparent.
10. Add new mesh assets and visual-only mesh geoms for the Biped chunks under the existing SMPL bodies.
11. Write the new MJCF plus exported mesh assets, materials, and textures.

## Mesh Partitioning Rules

### Body assignment

- Partition by triangle, not by vertex.
- For each triangle:
  - Look up all joint influences on its three vertices.
  - Map each Biped joint influence to an SMPL body using the approved table.
  - Sum mapped weights per target SMPL body across the triangle.
  - Assign the triangle to the SMPL body with the highest summed weight.
- This triangle-majority rule avoids cracks from per-vertex body switching.

### Materials

- Preserve Biped material assignments during export.
- If the source mesh uses material subsets, split the output by `(target_body, source_material_subset)` so visual appearance survives the rebinding.
- If material extraction fails, still export geometry and fall back to a plain material.

### Unmatched parts

- Twist bones are folded into the parent limb body.
- Fingers are folded into `L_Hand` or `R_Hand`.
- Extra torso or neck segments are folded into the nearest approved torso or neck body.
- No unmatched part contributes collision, inertia, or actuation.

## Output Invariants

The output MJCF must satisfy all of the following:

- The `<body>` hierarchy is identical to the SMPL template.
- The `<joint>` set, names, ordering, and attributes are identical to the SMPL template.
- The `<motor>` set, names, ordering, and attributes are identical to the SMPL template.
- The number of degrees of freedom is identical to the SMPL template.
- The existing SMPL physics geoms remain present and still determine collisions and inertia.
- The newly added Biped geoms are visual-only:
  - `type="mesh"`
  - `contype="0"`
  - `conaffinity="0"`
  - no density contribution
- The original SMPL physics geoms should be hidden visually, for example by setting `rgba="0 0 0 0"`.

## Implementation Shape

Do not extend the current rigid-body `UsdToMjcf` articulation path for this feature.

Reason:

- The current converter in `lightwheel/srl/from_usd/to_mjcf.py` builds MJCF articulations from `UsdPhysics.Joint` data.
- `Biped_Setup.usd` is a `UsdSkel` character, not a rigid-body USD.
- The desired behavior is "clone an MJCF template and swap visuals", which is a different pipeline with different invariants.

Recommended implementation:

- Add a new module under `lightwheel/srl/from_usd/`, for example `skinned_visual_to_template_mjcf.py`.
- Keep responsibilities separated into small units:
  - `load_template_mjcf(...)`
  - `load_template_usd(...)`
  - `load_biped_skin(...)`
  - `partition_and_bake_mesh(...)`
  - `write_visual_mesh_assets(...)`
  - `inject_visuals_into_template(...)`
- Reuse current-repo mesh and material export ideas where useful, especially from:
  - `lightwheel/srl/from_usd/usd_to_obj.py`
  - `lightwheel/srl/from_usd/_from_usd_helper.py`
- Do not route this feature through `TransformGraph.init_from_stage(...)`.

## CLI Shape

Add a dedicated CLI entrypoint parallel to the existing converter entrypoint.

Proposed arguments:

- `--template-mjcf`
- `--template-usd`
- `--visual-usd`
- `--output-dir`
- `--keep-template-visuals` as an optional debug flag

Expected output:

- One converted MJCF that preserves the SMPL controller-facing structure exactly.
- A `visuals/` directory containing exported mesh files, material files, and copied textures.

## Failure Handling

Be conservative and fail only when the output would clearly violate the invariant.

- If a Biped joint cannot be mapped:
  - collapse it into the nearest approved parent body
  - emit a warning
- If a non-critical SMPL body receives no triangles:
  - keep going
  - emit a warning
- If a major visible body receives no triangles:
  - fail fast

Major visible bodies:

- `Pelvis`
- `Torso`
- `Spine`
- `Chest`
- `Head`
- `L_Hip`, `L_Knee`, `L_Ankle`, `L_Toe`
- `R_Hip`, `R_Knee`, `R_Ankle`, `R_Toe`
- `L_Shoulder`, `L_Elbow`, `L_Wrist`, `L_Hand`
- `R_Shoulder`, `R_Elbow`, `R_Wrist`, `R_Hand`

- If triangle ownership is ambiguous:
  - use the highest summed mapped weight
  - report how many faces required forced assignment
- If textures or materials fail:
  - keep geometry export working
  - fall back to plain materials

## Verification

Verification must prove that the output is the same SMPL physics model with new visuals.

### Structural verification

- Parse the template MJCF and output MJCF.
- Assert identical body name order.
- Assert identical joint name order.
- Assert identical motor name order.
- Assert identical actuator count and joint count.

### Physics-shape verification

- Assert that every original SMPL physics geom is still present on the same body.
- Assert that newly added Biped geoms have zero contact participation.
- Assert that new Biped geoms do not contribute density or inertia.

### Model-dimension verification

If MuJoCo Python bindings are available:

- Load both models.
- Assert identical `nq`.
- Assert identical `nv`.
- Assert identical `nu`.

### Partition-report verification

- Emit a report of triangle counts per SMPL body.
- Emit a report of exported mesh asset counts per SMPL body.
- Emit warnings for empty or near-empty major visible bodies.

## Non-Goals

- Converting the Biped skeleton itself into a new physics rig.
- Adding runtime retargeting or controller-side changes.
- Making fingers or twist bones articulate independently in physics.
- Preserving USD animation payload behavior from `Biped_Setup.usd`.
- Generalizing the existing rigid-body USD-to-MJCF converter to support arbitrary `UsdSkel` characters in one pass.

## Result

The final product is a new conversion path that produces an MJCF visually shaped like the Biped character while remaining controller-compatible with the original SMPL motion-tracking setup.
