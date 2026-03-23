import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from lightwheel.srl.from_usd.skinned_visual_to_template_mjcf import (
    convert_skinned_visual_to_template_mjcf,
)


DEFAULT_BIPED_TO_SMPL_MAPPING = {
    "Root": "Pelvis",
    "Pelvis": "Pelvis",
    "L_UpLeg": "L_Hip",
    "L_UpLegTwist1": "L_Hip",
    "L_LoLeg": "L_Knee",
    "L_LoLegTwist1": "L_Knee",
    "L_Ankle": "L_Ankle",
    "L_Ball": "L_Toe",
    "R_UpLeg": "R_Hip",
    "R_UpLegTwist1": "R_Hip",
    "R_LoLeg": "R_Knee",
    "R_LoLegTwist1": "R_Knee",
    "R_Ankle": "R_Ankle",
    "R_Ball": "R_Toe",
    "Spine1": "Torso",
    "Spine2": "Spine",
    "Spine3": "Chest",
    "Chest": "Chest",
    "Neck1": "Neck",
    "Neck2": "Neck",
    "Head": "Head",
    "L_Clavicle": "L_Thorax",
    "L_UpArm": "L_Shoulder",
    "L_UpArmTwist1": "L_Shoulder",
    "L_LoArm": "L_Elbow",
    "L_LoArmTwist1": "L_Elbow",
    "L_Wrist": "L_Wrist",
    "L_ThumbFinger1": "L_Hand",
    "L_IndexFinger1": "L_Hand",
    "L_MiddleFinger1": "L_Hand",
    "L_RingFinger1": "L_Hand",
    "L_PinkyFinger1": "L_Hand",
    "R_Clavicle": "R_Thorax",
    "R_UpArm": "R_Shoulder",
    "R_UpArmTwist1": "R_Shoulder",
    "R_LoArm": "R_Elbow",
    "R_LoArmTwist1": "R_Elbow",
    "R_Wrist": "R_Wrist",
    "R_ThumbFinger1": "R_Hand",
    "R_IndexFinger1": "R_Hand",
    "R_MiddleFinger1": "R_Hand",
    "R_RingFinger1": "R_Hand",
    "R_PinkyFinger1": "R_Hand",
}


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Swap skinned USD visuals onto an MJCF template",
    )
    parser.add_argument("--template-mjcf", required=True)
    parser.add_argument("--template-usd", default=None)
    parser.add_argument("--visual-usd", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--keep-template-visuals", action="store_true")
    args = parser.parse_args()

    output_xml, report = convert_skinned_visual_to_template_mjcf(
        template_mjcf_path=Path(args.template_mjcf),
        template_usd_path=None if args.template_usd is None else Path(args.template_usd),
        visual_usd_path=Path(args.visual_usd),
        output_dir=Path(args.output_dir),
        joint_to_body_mapping=DEFAULT_BIPED_TO_SMPL_MAPPING,
        keep_template_visuals=args.keep_template_visuals,
    )

    print(f"output_xml={output_xml}")
    print(f"body_names={report.body_names}")
    print(f"joint_names={report.joint_names}")
    print(f"motor_names={report.motor_names}")


if __name__ == "__main__":
    main()
