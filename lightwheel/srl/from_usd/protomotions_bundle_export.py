from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import shutil
import subprocess
import sys


DEFAULT_USD_BODIES_ROOT_PRIM_PATH = "/World/envs/env_.*/Robot/bodies/"


@dataclass
class ProtoMotionsBundlePaths:
    bundle_dir: Path
    bundled_mjcf_path: Path
    visuals_dir: Path
    usd_dir: Path
    usd_path: Path
    usd_config_path: Path
    robot_config_path: Path
    factory_snippet_path: Path
    install_instructions_path: Path
    manifest_path: Path


def _robot_config_class_name(robot_name: str) -> str:
    parts = [part for part in robot_name.replace("-", "_").split("_") if part]
    return "".join(part[:1].upper() + part[1:] for part in parts) + "RobotConfig"


def _robot_config_python(
    robot_name: str,
    asset_file_name: str,
    usd_asset_file_name: str,
    usd_bodies_root_prim_path: str,
) -> str:
    class_name = _robot_config_class_name(robot_name)
    return f"""from dataclasses import dataclass, field

from protomotions.robot_configs.base import RobotAssetConfig
from protomotions.robot_configs.smpl import SmplRobotConfig


@dataclass
class {class_name}(SmplRobotConfig):
    asset: RobotAssetConfig = field(
        default_factory=lambda: RobotAssetConfig(
            asset_file_name="{asset_file_name}",
            usd_asset_file_name="{usd_asset_file_name}",
            usd_bodies_root_prim_path="{usd_bodies_root_prim_path}",
            self_collisions=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            angular_damping=0.0,
            linear_damping=0.0,
        )
    )
"""


def _factory_registration_snippet(robot_name: str) -> str:
    class_name = _robot_config_class_name(robot_name)
    return f"""elif robot_name == "{robot_name}":
    from protomotions.robot_configs.{robot_name} import {class_name}

    config = {class_name}()
"""


def _install_instructions_markdown(robot_name: str) -> str:
    class_name = _robot_config_class_name(robot_name)
    return f"""# ProtoMotions Registration for `{robot_name}`

Copy the generated bundle files into your ProtoMotions checkout:

```bash
cp protomotions_bundle/assets/mjcf/{robot_name}.xml \\
  /path/to/ProtoMotions/protomotions/data/assets/mjcf/

mkdir -p /path/to/ProtoMotions/protomotions/data/assets/usd/{robot_name}
cp -r protomotions_bundle/assets/usd/{robot_name}/* \\
  /path/to/ProtoMotions/protomotions/data/assets/usd/{robot_name}/

cp protomotions_bundle/robot_configs/{robot_name}.py \\
  /path/to/ProtoMotions/protomotions/robot_configs/
```

Then paste the contents of `protomotions_bundle/factory_registration_snippet.py`
into `protomotions/robot_configs/factory.py`.

The generated robot config class is `{class_name}`.

Smoke-test commands:

```bash
python examples/tutorial/2_load_robot.py --robot {robot_name} --simulator isaacgym
python examples/tutorial/2_load_robot.py --robot {robot_name} --simulator isaaclab
```
"""


def _usd_config_yaml(asset_path: Path, usd_path: Path) -> str:
    return "\n".join(
        [
            f"asset_path: {asset_path}",
            f"usd_dir: {usd_path.parent}",
            f"usd_file_name: {usd_path.name}",
            "force_usd_conversion: true",
            "make_instanceable: true",
            "import_inertia_tensor: true",
            "fix_base: false",
            "import_sites: false",
            "self_collision: false",
            "link_density: 0.0",
            "",
        ]
    )


def _manifest_payload(
    *,
    robot_name: str,
    visual_usd_path: Path,
    template_mjcf_path: Path,
    template_usd_path: Path | None,
    output_mjcf_path: Path,
    bundled_mjcf_path: Path,
    usd_path: Path,
    body_names: list[str],
    joint_names: list[str],
    motor_names: list[str],
    triangle_counts: dict[str, int],
    joint_to_body_mapping: dict[str, str],
) -> dict:
    return {
        "robot_name": robot_name,
        "visual_usd_path": visual_usd_path.as_posix(),
        "template_mjcf_path": template_mjcf_path.as_posix(),
        "template_usd_path": None if template_usd_path is None else template_usd_path.as_posix(),
        "output_mjcf_path": output_mjcf_path.as_posix(),
        "bundled_mjcf_path": bundled_mjcf_path.as_posix(),
        "bundled_usd_path": usd_path.as_posix(),
        "body_names": body_names,
        "joint_names": joint_names,
        "motor_names": motor_names,
        "exported_triangle_count_by_body": triangle_counts,
        "joint_to_body_mapping": joint_to_body_mapping,
    }


def _generate_usd_with_protomotions(
    protomotions_root: Path,
    mjcf_path: Path,
    usd_path: Path,
    protomotions_python: str | None = None,
) -> None:
    script_path = protomotions_root / "data" / "scripts" / "convert_mjcf_to_usd.py"
    if not script_path.is_file():
        raise ValueError(
            f"ProtoMotions MJCF-to-USD converter not found at '{script_path}'."
        )

    python_executable = sys.executable if protomotions_python is None else protomotions_python
    try:
        subprocess.run(
            [
                python_executable,
                script_path.as_posix(),
                mjcf_path.as_posix(),
                usd_path.as_posix(),
                "--make-instanceable",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
    except subprocess.CalledProcessError as exc:
        details = exc.stderr.strip() or exc.stdout.strip() or str(exc)
        raise RuntimeError(
            "ProtoMotions USD conversion failed. "
            f"Tried interpreter '{python_executable}'. "
            "Use an Isaac Lab / ProtoMotions Python via protomotions_python "
            "if the current environment cannot import Isaac Lab.\n"
            f"{details}"
        ) from exc

    if not usd_path.is_file():
        raise RuntimeError(f"ProtoMotions USD conversion did not create '{usd_path}'.")


def export_protomotions_bundle(
    *,
    output_dir: Path,
    output_mjcf_path: Path,
    visuals_dir: Path,
    visual_usd_path: Path,
    template_mjcf_path: Path,
    template_usd_path: Path | None,
    protomotions_root: Path,
    robot_name: str,
    body_names: list[str],
    joint_names: list[str],
    motor_names: list[str],
    triangle_counts: dict[str, int],
    joint_to_body_mapping: dict[str, str],
    usd_bodies_root_prim_path: str = DEFAULT_USD_BODIES_ROOT_PRIM_PATH,
    protomotions_python: str | None = None,
) -> ProtoMotionsBundlePaths:
    bundle_dir = output_dir / "protomotions_bundle"
    mjcf_dir = bundle_dir / "assets" / "mjcf"
    bundled_visuals_dir = mjcf_dir / "visuals"
    usd_dir = bundle_dir / "assets" / "usd" / robot_name
    robot_configs_dir = bundle_dir / "robot_configs"

    mjcf_dir.mkdir(parents=True, exist_ok=True)
    usd_dir.mkdir(parents=True, exist_ok=True)
    robot_configs_dir.mkdir(parents=True, exist_ok=True)

    bundled_mjcf_path = mjcf_dir / f"{robot_name}.xml"
    shutil.copy2(output_mjcf_path, bundled_mjcf_path)

    if visuals_dir.exists():
        shutil.copytree(visuals_dir, bundled_visuals_dir, dirs_exist_ok=True)

    usd_path = usd_dir / f"{robot_name}.usda"
    _generate_usd_with_protomotions(
        protomotions_root,
        bundled_mjcf_path,
        usd_path,
        protomotions_python=protomotions_python,
    )

    usd_config_path = usd_dir / "config.yaml"
    usd_config_path.write_text(
        _usd_config_yaml(bundled_mjcf_path, usd_path),
        encoding="utf-8",
    )

    robot_config_path = robot_configs_dir / f"{robot_name}.py"
    robot_config_path.write_text(
        _robot_config_python(
            robot_name=robot_name,
            asset_file_name=f"mjcf/{robot_name}.xml",
            usd_asset_file_name=f"usd/{robot_name}/{robot_name}.usda",
            usd_bodies_root_prim_path=usd_bodies_root_prim_path,
        ),
        encoding="utf-8",
    )

    factory_snippet_path = bundle_dir / "factory_registration_snippet.py"
    factory_snippet_path.write_text(
        _factory_registration_snippet(robot_name),
        encoding="utf-8",
    )

    install_instructions_path = bundle_dir / "install_instructions.md"
    install_instructions_path.write_text(
        _install_instructions_markdown(robot_name),
        encoding="utf-8",
    )

    manifest_path = bundle_dir / "manifest.json"
    manifest_path.write_text(
        json.dumps(
            _manifest_payload(
                robot_name=robot_name,
                visual_usd_path=visual_usd_path,
                template_mjcf_path=template_mjcf_path,
                template_usd_path=template_usd_path,
                output_mjcf_path=output_mjcf_path,
                bundled_mjcf_path=bundled_mjcf_path,
                usd_path=usd_path,
                body_names=body_names,
                joint_names=joint_names,
                motor_names=motor_names,
                triangle_counts=triangle_counts,
                joint_to_body_mapping=joint_to_body_mapping,
            ),
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    return ProtoMotionsBundlePaths(
        bundle_dir=bundle_dir,
        bundled_mjcf_path=bundled_mjcf_path,
        visuals_dir=bundled_visuals_dir,
        usd_dir=usd_dir,
        usd_path=usd_path,
        usd_config_path=usd_config_path,
        robot_config_path=robot_config_path,
        factory_snippet_path=factory_snippet_path,
        install_instructions_path=install_instructions_path,
        manifest_path=manifest_path,
    )
