"""YUBIのCAD接触を指先の板・基部の箱へ置換し、既存の慣性を保持する。"""
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def text(values):
    return ' '.join(format(float(v), '.17g') for v in values)


def mesh_vertices(model, geom_id):
    mesh = model.geom_dataid[geom_id]
    start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, model.geom_quat[geom_id])
    vertices = model.mesh_vert[start:start + count].astype(float)
    return np.einsum('ij,kj->ik', vertices, rotation.reshape(3, 3)) + model.geom_pos[geom_id]


def plate(model, body_id, thickness, length):
    visual = [i for i in range(model.ngeom) if model.geom_bodyid[i] == body_id and
              model.geom_group[i] == 2 and model.geom_rgba[i, 0] > .9 and model.geom_rgba[i, 1] < .3]
    if len(visual) != 1:
        raise ValueError('指先パッドの赤い表示メッシュを特定できません')
    mesh_name = model.mesh(model.geom_dataid[visual[0]]).name.removesuffix('_visual')
    candidates = [i for i in range(model.ngeom) if model.geom_bodyid[i] == body_id and
                  model.geom_group[i] == 3 and model.mesh(model.geom_dataid[i]).name == mesh_name]
    if len(candidates) != 1:
        raise ValueError('指先パッドの元メッシュを特定できません')
    geom = candidates[0]
    vertices = mesh_vertices(model, geom)
    mesh = model.geom_dataid[geom]
    start, count = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
    triangles = vertices[model.mesh_face[start:start + count]]
    cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    area = np.linalg.norm(cross, axis=1)
    normals = cross / np.maximum(area[:, None], 1e-15)
    # 左側の指は-Y、右側の指は+Yを向く内側平面から選ぶ。
    sign = -1 if model.body(body_id).name.endswith('_left') else 1
    mask = (normals[:, 1] * sign > .8) & (area > 1e-12)
    planes = np.column_stack([normals[mask], np.sum(normals[mask] * triangles[mask, 0], axis=1)])
    _, labels = np.unique(np.round(planes, 4), axis=0, return_inverse=True)
    selected = labels == np.argmax(np.bincount(labels, weights=area[mask]))
    normal = np.average(normals[mask][selected], axis=0, weights=area[mask][selected])
    normal[0] = 0
    normal /= np.linalg.norm(normal)
    surface = float(np.average(triangles[mask, 0][selected] @ normal, weights=area[mask][selected]))
    width_axis = np.array([1., 0., 0.])
    rotation = np.column_stack([width_axis, normal, np.cross(width_axis, normal)])
    local = np.einsum('ij,jk->ik', vertices, rotation)
    lo, hi = local.min(axis=0), local.max(axis=0)
    length = min(length, hi[2] - lo[2])
    # 先端側だけを板にする。曲がった根元へ接触面を延長しない。
    if rotation[2, 2] < 0:
        zcenter = hi[2] - length/2
    else:
        zcenter = lo[2] + length/2
    center = rotation @ np.array([(lo[0] + hi[0])/2, surface - thickness/2, zcenter])
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, rotation.reshape(-1))
    return center, quat, np.array([(hi[0] - lo[0])/2, thickness/2, length/2])


def simplify_hands(xml, settings):
    thickness = float(settings.get('plate_thickness_m', .004))
    length = float(settings.get('plate_length_m', .045))
    if not (.001 <= thickness <= .02 and .01 <= length <= .1):
        raise ValueError('指先板の厚み・長さが不正です')
    reference = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding='unicode'))
    xml.find('compiler').set('inertiafromgeom', 'auto')
    for prefix in ('R', 'L'):
        for suffix in ('gripper_base', 'finger_left', 'finger_right'):
            name = f'{prefix}_{suffix}'
            body = xml.find(f".//body[@name='{name}']")
            body_id = reference.body(name).id
            old = body.find('inertial')
            if old is not None:
                body.remove(old)
            ET.SubElement(body, 'inertial', mass=str(reference.body_mass[body_id]),
                          pos=text(reference.body_ipos[body_id]), quat=text(reference.body_iquat[body_id]),
                          diaginertia=text(reference.body_inertia[body_id]))
            if suffix == 'gripper_base':
                ids = [i for i in range(reference.ngeom) if reference.geom_bodyid[i] == body_id and reference.geom_group[i] == 3]
                vertices = np.concatenate([mesh_vertices(reference, i) for i in ids])
                lo, hi = vertices.min(axis=0), vertices.max(axis=0)
                center, quat, size = (lo + hi)/2, np.array([1, 0, 0, 0]), (hi - lo)/2
            else:
                center, quat, size = plate(reference, body_id, thickness, length)
            for geom in list(body.findall('geom')):
                if geom.get('group') == '3':
                    body.remove(geom)
            attrs = dict(name=name + '_simple_collision', type='box', pos=text(center), quat=text(quat),
                         size=text(size), group='3', density='0', contype='1', conaffinity='1',
                         priority='1', rgba='.4 .6 .8 0')
            # 箱の既定solrefと平均せず、板に指定した接触特性を使う。
            for key in ('friction', 'solref', 'solimp', 'condim'):
                if key in settings:
                    attrs[key] = str(settings[key]) if key == 'condim' else text(settings[key])
            ET.SubElement(body, 'geom', **attrs)
    return reference
