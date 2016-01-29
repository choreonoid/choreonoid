#!/usr/bin/env python
#
#

import sys
import os

#
# Functions.
#

#
def add_element_to_material(material, element, limit, key):
    if (('name' in material) == True and len(element) > limit and element[0] == key):
        key = element.pop(0)

        for val in element:
            try:
                float(val)
            except ValueError:
                return False

        material[key] = element
        return True

    return False

#
def create_ambient(material, ambient):
    if (add_element_to_material(material, ambient, 3, 'ambient') == True):
        r = float(ambient[0])
        g = float(ambient[1])
        b = float(ambient[2])

        # XXX: really correct ?
        material['ambientIntensity'] = (r + g + b) / 3.0

#
def create_diffuse(material, diffuse):
    add_element_to_material(material, diffuse, 3, 'diffuse')

#
def create_specular(material, specular):
    if (add_element_to_material(material, specular, 4, 'specular') == True):
        if (len(specular) == 4):
            f = float(specular[3])
        elif (len(specular) == 5):
            f = float(specular[4])
        else:
            return

        if (f >= 1.0 and f <= 128.0):
            material['shininess'] = f / 128.0

#
def create_emissive(material, emissive):
    add_element_to_material(material, emissive, 3, 'emissive')

#
def create_key(material, name):
    if (name == ''):
        return

    tmp = name.split(':')

    if (len(tmp) > 1):
        material['name'] = tmp[0].split(' ')[1].rstrip().lstrip()
        material['inherit'] = tmp[1].rstrip().lstrip() 
    else:
        material['name'] = name.split(' ')[1].rstrip().lstrip()

#
def add_material_to_result(result, material):
    if (len(material) < 2 or ('name' in material) == False):
        return
 
    key = material.pop('name')
    result[key] = material

#
def print_color_material(result):
    indent = '    '
    prefix = 'SDFLOADER_SET_COLOR_MATERIAL('
    suffix = ');'

    # these default values, the Choreonoid's default values 
    # see choreonoid/src/Util/SceneDrawable.cpp SgMaterial()
    ai = 0.02  # ambient intensity
    tr = '0.0' # transparency

    # these default values, see http://www.ogre3d.org/docs/manual/manual_16.html
    dc = [ '1.0', '1.0', '1.0' ] # diffuse color
    ec = [ '0.0', '0.0', '0.0' ] # emmisive color
    sc = [ '0.0', '0.0', '0.0' ] # specular color
    sh = 0.0 # shininess (in specular)

    for key, val in sorted(result.items()):
        if (key.startswith('Gazebo/') == False):
            continue

        isSettingAmibient = False
        isSettingDiffuse  = False
        isSettingSpecular = False
        isSettingEmissive = False
        isInherit         = ('inherit' in val)

        if (isInherit == True):
            parent = val['inherit']
        else:
            parent = None

        tmp = []
        tmp.append(indent)
        tmp.append(prefix)
        tmp.append('"' + key + '", ')

        if (('diffuse' in val) == True):
            tmp.append(val['diffuse'][0] + ', ')
            tmp.append(val['diffuse'][1] + ', ')
            tmp.append(val['diffuse'][2] + ', ')
            isSettingDiffuse = True
        elif (isInherit == True and ('diffuse' in parent) == True):
            tmp.append(parent['diffuse'][0] + ', ')
            tmp.append(parent['diffuse'][1] + ', ')
            tmp.append(parent['diffuse'][2] + ', ')
            isSettingDiffuse = True
        else:
            tmp.append(dc[0] + ', ')
            tmp.append(dc[1] + ', ')
            tmp.append(dc[2] + ', ')

        if (('emissive' in val) == True):
            tmp.append(val['emissive'][0] + ', ')
            tmp.append(val['emissive'][1] + ', ')
            tmp.append(val['emissive'][2] + ', ')
            isSettingEmissive = True
        elif (isInherit == True and ('emissive' in parent) == True):
            tmp.append(parent['emissive'][0] + ', ')
            tmp.append(parent['emissive'][1] + ', ')
            tmp.append(parent['emissive'][2] + ', ')
            isSettingEmissive = True
        else:
            tmp.append(ec[0] + ', ')
            tmp.append(ec[1] + ', ')
            tmp.append(ec[2] + ', ')

        if (('specular' in val) == True):
            tmp.append(val['specular'][0] + ', ')
            tmp.append(val['specular'][1] + ', ')
            tmp.append(val['specular'][2] + ', ')
            isSettingSpecular = True
        elif (isInherit == True and ('specular' in parent) == True):
            tmp.append(parent['specular'][0] + ', ')
            tmp.append(parent['specular'][1] + ', ')
            tmp.append(parent['specular'][2] + ', ')
            isSettingSpecular = True
        else:
            tmp.append(sc[0] + ', ')
            tmp.append(sc[1] + ', ')
            tmp.append(sc[2] + ', ')

        if (('shininess' in val) == True):
            sh = val['shininess']
        elif (isInherit == True and ('shininess' in parent) == True):
            sh = parent['shininess']

        if (('ambientIntensity' in val) == True):
            ai = val['ambientIntensity']
            isSettingAmibient = True
        elif (isInherit == True and ('ambientIntensity' in parent) == True):
            ai = parent['ambientIntensity']
            isSettingAmibient = True

        tmp.append(str(ai) + ', ')
        tmp.append(str(sh) + ', ')
        tmp.append(tr + ', ')

        if (isSettingAmibient == True):
            tmp.append('true, ')
        else:
            tmp.append('false, ')

        if (isSettingDiffuse == True):
            tmp.append('true, ')
        else:
            tmp.append('false, ')

        if (isSettingSpecular == True):
            tmp.append('true, ')
        else:
            tmp.append('false, ')

        if (isSettingEmissive == True):
            tmp.append('true')
        else:
            tmp.append('false')

        tmp.append(suffix)

        line = ''.join(tmp)
        print line

#
def usage():
    name = os.path.basename(sys.argv[0])

    print 'Usage: ' + name + ' <path to gazebo.material>'
    print '     : e.g. ' + name + ' /usr/share/gazebo-6.5/media/materials/scripts/gazebo.material > gazebo_colors'

#
# Main.
#

if __name__ == '__main__':
    try:
        if (len(sys.argv) < 2):
            usage()
            sys.exit(1)

        result = {}
        material = {}

        with open(sys.argv[1]) as file:
            for line in file:
                tmp = line.lstrip().rstrip()

                if (tmp.startswith('//')):
                    continue

                if (tmp.startswith('material ')):
                    add_material_to_result(result, material)

                    material = {}
                    create_key(material, tmp)
                elif (tmp.startswith('ambient ')):
                    create_ambient(material, tmp.split(' '))
                elif (tmp.startswith('diffuse ')):
                    create_diffuse(material, tmp.split(' '))
                elif (tmp.startswith('specular ')):
                    create_specular(material, tmp.split(' '))
                elif (tmp.startswith('emissive ')):
                    create_emissive(material, tmp.split(' '))

        print_color_material(result)
    except IOError:
        usage()
        sys.exit(1)

    sys.exit(0)
