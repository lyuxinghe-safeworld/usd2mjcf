# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
"""NVIDIA SRL Usd To Urdf package."""


# Local Folder
from .._usd_to_urdf import _get_version

# Set `__version__` attribute
__version__ = _get_version()

# Remove `_get_version` so it is not added as an attribute
del _get_version

from .skinned_visual_to_template_mjcf import convert_skinned_visual_to_template_mjcf
