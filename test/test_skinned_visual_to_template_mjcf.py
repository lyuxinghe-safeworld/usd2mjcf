from pathlib import Path

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
