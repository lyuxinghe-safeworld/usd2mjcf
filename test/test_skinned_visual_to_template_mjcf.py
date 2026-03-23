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
