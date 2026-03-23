from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET


def _format_float_tuple(values: tuple[float, ...]) -> str:
    return " ".join(f"{value:.10g}" for value in values)


def _iter_bodies(parent: ET.Element):
    for body in parent.findall("body"):
        yield body
        yield from _iter_bodies(body)


def _ensure_child(root: ET.Element, tag: str, before_tag: str | None = None) -> ET.Element:
    child = root.find(tag)
    if child is not None:
        return child

    child = ET.Element(tag)
    if before_tag is None:
        root.append(child)
        return child

    for index, element in enumerate(list(root)):
        if element.tag == before_tag:
            root.insert(index, child)
            return child

    root.append(child)
    return child


@dataclass
class VisualGeomSpec:
    body_name: str
    geom_name: str
    mesh_name: str
    mesh_file: str
    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]
    material_name: str | None


@dataclass
class TemplateMjcfModel:
    tree: ET.ElementTree
    root: ET.Element
    asset: ET.Element
    worldbody: ET.Element
    actuator: ET.Element
    body_by_name: dict[str, ET.Element]
    body_names: list[str]
    joint_names: list[str]
    motor_names: list[str]
    body_to_geoms: dict[str, list[ET.Element]]


def load_template_mjcf(path: Path | str) -> TemplateMjcfModel:
    tree = ET.parse(path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("Template MJCF is missing <worldbody>.")

    asset = _ensure_child(root, "asset", before_tag="worldbody")
    actuator = _ensure_child(root, "actuator")

    bodies = list(_iter_bodies(worldbody))
    body_names = [body.get("name") for body in bodies if body.get("name")]
    body_by_name = {body.get("name"): body for body in bodies if body.get("name")}
    joint_names = [
        joint.get("name")
        for joint in worldbody.iterfind(".//joint")
        if joint.get("name") is not None
    ]
    motor_names = [
        motor.get("name")
        for motor in actuator.iterfind(".//motor")
        if motor.get("name") is not None
    ]
    body_to_geoms = {
        body_name: list(body.findall("geom"))
        for body_name, body in body_by_name.items()
    }

    return TemplateMjcfModel(
        tree=tree,
        root=root,
        asset=asset,
        worldbody=worldbody,
        actuator=actuator,
        body_by_name=body_by_name,
        body_names=body_names,
        joint_names=joint_names,
        motor_names=motor_names,
        body_to_geoms=body_to_geoms,
    )


def inject_visual_geoms(
    model: TemplateMjcfModel,
    visual_specs: list[VisualGeomSpec],
    keep_template_visuals: bool = False,
) -> None:
    if not keep_template_visuals:
        for geoms in model.body_to_geoms.values():
            for geom in geoms:
                geom.set("rgba", "0 0 0 0")

    existing_mesh_names = {
        mesh.get("name")
        for mesh in model.asset.findall("mesh")
        if mesh.get("name") is not None
    }

    for spec in visual_specs:
        body = model.body_by_name.get(spec.body_name)
        if body is None:
            raise KeyError(f"Body '{spec.body_name}' not found in template MJCF.")

        if spec.mesh_name not in existing_mesh_names:
            mesh = ET.SubElement(model.asset, "mesh")
            mesh.set("name", spec.mesh_name)
            mesh.set("file", spec.mesh_file)
            existing_mesh_names.add(spec.mesh_name)

        geom = ET.SubElement(body, "geom")
        geom.set("name", spec.geom_name)
        geom.set("type", "mesh")
        geom.set("mesh", spec.mesh_name)
        geom.set("pos", _format_float_tuple(spec.pos))
        geom.set("quat", _format_float_tuple(spec.quat))
        geom.set("contype", "0")
        geom.set("conaffinity", "0")
        if spec.material_name is not None:
            geom.set("material", spec.material_name)

        model.body_to_geoms[spec.body_name].append(geom)
