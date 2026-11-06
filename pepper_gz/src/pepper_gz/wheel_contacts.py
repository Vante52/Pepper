"""Rodillos pasivos solo en SDF, conservando masa, centro e inercia de las ruedas."""

import math
import xml.etree.ElementTree as ET

import numpy as np


def rotation(roll, pitch, yaw):
    """Matriz Rz Ry Rx de una pose SDF."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def parallel_axis(mass, position):
    """Traslada el tensor desde el centro al origen del frame."""
    return mass * (np.dot(position, position) * np.eye(3) - np.outer(position, position))


def capsule_inertia(mass, radius, length):
    """Tensor central de una cápsula uniforme orientada según Z."""
    cylinder = mass * length / (length + 4. * radius / 3.)
    sphere = mass - cylinder
    axial = .5 * cylinder * radius**2 + .4 * sphere * radius**2
    transverse = cylinder * (3. * radius**2 + length**2) / 12.
    transverse += sphere * (83. / 320. * radius**2 + (length / 2. + 3.*radius/8.)**2)
    return np.diag([transverse, transverse, axial])


def read_inertia(element):
    """Lee el triángulo superior almacenado por SDF."""
    return np.array([[float(element.findtext('i' + ''.join(sorted(a+b)), '0'))
                      for b in 'xyz'] for a in 'xyz'])


def write_inertia(link, mass, center, matrix):
    """Escribe masa y tensor en un frame inercial paralelo al link."""
    existing = link.find('inertial')
    if existing is not None:
        link.remove(existing)
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass').text = str(mass)
    ET.SubElement(inertial, 'pose').text = ' '.join(str(v) for v in center) + ' 0 0 0'
    values = ET.SubElement(inertial, 'inertia')
    for i, a in enumerate('xyz'):
        for j, b in enumerate('xyz'):
            if j >= i:
                ET.SubElement(values, 'i' + a + b).text = str(matrix[i, j])


def configure_wheel_contacts(sdf, config):
    """Reemplaza las esferas por contactos que permiten rodar transversalmente."""
    names = config.get('wheel_links')
    if (not isinstance(names, list) or not names
            or not all(isinstance(name, str) and name for name in names)
            or len(set(names)) != len(names)):
        raise ValueError('wheel_links debe contener nombres distintos')
    count = config.get('roller_count')
    if type(count) is not int or not 6 <= count <= 32:
        raise ValueError('roller_count debe estar entre 6 y 32')
    for key in ('roller_radius', 'roller_length', 'roller_mass', 'friction', 'friction2',
                'rolling_friction', 'torsional_friction'):
        value = config.get(key)
        if (type(value) not in (int, float) or not math.isfinite(value) or value < 0
                or (key.startswith('roller_') and value == 0)):
            raise ValueError(f'{key} no es válido')
    model = sdf.find('model')
    radius, length, mass = (config[k] for k in ('roller_radius', 'roller_length', 'roller_mass'))
    tensor = capsule_inertia(mass, radius, length)
    for name in names:
        wheel = model.find(f"link[@name='{name}']")
        if wheel is None or len(wheel.findall('collision')) != 1:
            raise ValueError(f'Se esperaba una colisión esférica en {name}')
        collision = wheel.find('collision')
        wheel_radius = float(collision.findtext('geometry/sphere/radius', 'nan'))
        if not math.isfinite(wheel_radius) or not 0 < radius < wheel_radius:
            raise ValueError(f'{name}: radio de rodillo incompatible con la rueda')
        source = wheel.find('inertial')
        original_mass = float(source.findtext('mass'))
        pose = [float(v) for v in source.findtext('pose', '0 0 0 0 0 0').split()]
        center, r = np.array(pose[:3]), rotation(*pose[3:])
        original = r @ read_inertia(source.find('inertia')) @ r.T
        original += parallel_axis(original_mass, center)
        hub_mass = original_mass - count * mass
        if hub_mass <= 0:
            raise ValueError(f'{name}: la masa de los rodillos supera la original')
        hub_center = original_mass * center / hub_mass
        added = np.zeros((3, 3))
        for index in range(count):
            angle = 2. * math.pi * index / count
            position = np.array([0., (wheel_radius-radius)*math.sin(angle),
                                 (wheel_radius-radius)*math.cos(angle)])
            roll = -angle - math.pi / 2.
            r = rotation(roll, 0., 0.)
            added += r @ tensor @ r.T + parallel_axis(mass, position)
            roller_name = f'{name}_roller_{index:02d}'
            if model.find(f"link[@name='{roller_name}']") is not None:
                raise ValueError(f'Rodillo duplicado: {roller_name}')
            roller = ET.SubElement(model, 'link', name=roller_name)
            ET.SubElement(roller, 'pose', relative_to=name).text = (
                ' '.join(str(v) for v in position) + f' {roll} 0 0')
            ET.SubElement(roller, 'self_collide').text = 'false'
            write_inertia(roller, mass, np.zeros(3), tensor)
            contact = ET.SubElement(roller, 'collision', name='roller_contact')
            capsule = ET.SubElement(ET.SubElement(contact, 'geometry'), 'capsule')
            ET.SubElement(capsule, 'radius').text = str(radius)
            ET.SubElement(capsule, 'length').text = str(length)
            friction = ET.SubElement(ET.SubElement(contact, 'surface'), 'friction')
            bullet = ET.SubElement(friction, 'bullet')
            for key in ('friction', 'friction2', 'rolling_friction'):
                ET.SubElement(bullet, key).text = str(config[key])
            ET.SubElement(ET.SubElement(friction, 'torsional'), 'coefficient').text = str(
                config['torsional_friction'])
            joint = ET.SubElement(model, 'joint', name=roller_name+'_joint', type='revolute')
            ET.SubElement(joint, 'parent').text = name
            ET.SubElement(joint, 'child').text = roller_name
            ET.SubElement(joint, 'pose', relative_to=roller_name).text = '0 0 0 0 0 0'
            axis = ET.SubElement(joint, 'axis')
            ET.SubElement(axis, 'xyz').text = '0 0 1'
            limit = ET.SubElement(axis, 'limit')
            ET.SubElement(limit, 'lower').text = '-1e16'
            ET.SubElement(limit, 'upper').text = '1e16'
        hub_tensor = original - added - parallel_axis(hub_mass, hub_center)
        if min(np.linalg.eigvalsh(hub_tensor)) <= 0:
            raise ValueError(f'{name}: no se puede conservar una inercia positiva')
        write_inertia(wheel, hub_mass, hub_center, hub_tensor)
        wheel.remove(collision)
