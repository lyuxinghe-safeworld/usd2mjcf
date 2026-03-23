import argparse
import os
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from lightwheel.srl.from_usd.to_mjcf import UsdToMjcf

def main():
    parser = argparse.ArgumentParser(description="Convert USD to MJCF")
    parser.add_argument("input_path", type=str, help="Path to input USD file")
    parser.add_argument("--output_path", type=str, default=None, help="Path to output MJCF file")
    parser.add_argument("--generate_collision", action='store_true', help="Generate collision meshes for the MJCF model")
    parser.add_argument("--preprocess_resolution", type=int, default=20, help="Preprocessing voxelization resolution for convex decomposition")
    parser.add_argument("--resolution", type=int, default=2000, help="Main voxelization resolution for convex decomposition")
    
    args = parser.parse_args()

    input_path = args.input_path
    output_path = args.output_path
    generate_collision = args.generate_collision
    preprocess_resolution = args.preprocess_resolution
    resolution = args.resolution
    
    if output_path is None:
        output_path = os.path.dirname(input_path)
    mjcf_dir = os.path.join(output_path, 'MJCF')
    
    mjcf_file = UsdToMjcf.init_from_file(input_path).save_to_file(mjcf_dir)
    
    # Only generate collision meshes if requested
    if generate_collision:
        from utils.add_collision import add_collision_to_only_visual_mjcf
        add_collision_to_only_visual_mjcf(mjcf_file, preprocess_resolution=preprocess_resolution, resolution=resolution)

if __name__ == "__main__":
    main()

