#!/usr/bin/env python
# Copyright (C) 2015 Swift Navigation Inc.
# Contact: Bhaskar Mookerji <mookerji@swiftnav.com>
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
# THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
# EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

"""Generator for Protocol Buffers target.

This module consumes the YAML spec and generates some message class
files.

"""

import os
import re
import copy
from sbpg.targets.templating import *
from sbpg.utils import markdown_links

MESSAGES_TEMPLATE_NAME = 'message_template.ros.j2'
CONVERSION_TEMPLATE_NAME = 'conversion_template.ros.j2'
CONVERSION_SRC_TEMPLATE_NAME = 'conversion_src_template.ros.j2'
CALLBACK_TEMPLATE_NAME = 'callback_template.ros.j2'

TYPE_MAP = {
  'u8': 'uint8',
  'u16': 'uint16',
  'u32': 'uint32',
  'u64': 'uint64',
  's8': 'int8',
  's16': 'int16',
  's32': 'int32',
  's64': 'int64',
  'float': 'float32',
  'double': 'float64',
  'string': 'string',
}

def to_comment(value):
  """
  Builds a comment.
  """
  if value is None:
    return ''
  if len(value.split('\n')) == 1:
    return "# " + value
  else:
    return '\n'.join(['# ' + l for l in value.split('\n')[:-1]])

def to_unit(value):
    return to_comment('[' + str(value) + ']')

def to_identifier(s):
  """
  Convert snake_case to camel_case.
  """
  if s.startswith('GPS'):
      s = 'Gps' + s[3:]
  if s.startswith('STEC'):
      s = 'Stec' + s[4:]
  if s.startswith('UART'):
      s = 'Uart' + s[4:]
  return ''.join([i.capitalize() for i in s.split('_')]) if '_' in s else s

def to_msg_type(s):
    return "SBP_" + s

def to_type(f, type_map=TYPE_MAP):
    name = f.type_id
    if name.startswith('GPS'):
        name = 'Gps' + name[3:]
    if name.startswith('STEC'):
        name = 'Stec' + name[4:]
    if name.startswith('UART'):
        name = 'Uart' + name[4:]
    if type_map.get(name, None):
        return type_map.get(name, None)
    elif name == 'array':
        fill = f.options['fill'].value
        f_ = copy.copy(f)
        f_.type_id = fill
        return "%s[]" % to_type(f_)
    return name

def to_title(s):
    return s.title()

def is_deprecated(definition):
    if 'DEP' in definition.identifier.upper():
        return True
    if not definition.desc:
        return False
    return 'legacy' in definition.desc

def is_empty(definition):
    return len(definition.fields) == 0

def to_sbp_file_name(identifier):
    prefix = 'swiftnav.sbp.'
    if identifier.startswith(prefix):
        return identifier[len(prefix):]
    else:
        return identifier

def to_sbp_msg_type_name(s):
    if s.isupper():
        s = s.lower() + '_t'
    else:
        # https://stackoverflow.com/questions/1175208/elegant-python-function-to-convert-camelcase-to-snake-case
        s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', s)
        s = re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower() + '_t'
    if s == 'gnss_signal_t' or s == 'gps_time_t':
        s = 'sbp_' + s
    return s

def to_topic(s):
    s = to_sbp_msg_type_name(s)
    suffix = '_t'
    if s.endswith(suffix):
        s = s[:-len(suffix)]
    prefix = 'sbp_'
    if s.startswith(prefix):
        s = s[len(prefix):]
    prefix = 'msg_'
    if s.startswith(prefix):
        s = s[len(prefix):]
    return s

def is_default_type(f, type_map=TYPE_MAP):
    name = f.type_id
    if type_map.get(name, None):
        return True

def is_default_array_type(f, type_map=TYPE_MAP):
    return f.type_id == 'array' and type_map.get(f.options['fill'].value, None)

JENV.filters['ros_to_identifier'] = to_identifier
JENV.filters['ros_to_type'] = to_type
JENV.filters['ros_to_comment'] = to_comment
JENV.filters['ros_to_unit'] = to_unit
JENV.filters['ros_to_title'] = to_title
JENV.filters['ros_to_sbp_file_name'] = to_sbp_file_name
JENV.filters['ros_is_deprecated'] = is_deprecated
JENV.filters['ros_is_empty'] = is_empty
JENV.filters['ros_is_default_type'] = is_default_type
JENV.filters['ros_is_default_array_type'] = is_default_array_type
JENV.filters['ros_to_sbp_msg_type_name'] = to_sbp_msg_type_name
JENV.filters['ros_to_topic'] = to_topic
JENV.filters['ros_to_msg_type'] = to_msg_type

def render_source(output_dir, package_spec):
    """
    Render and output to a directory given a package specification.
    """
    path, name = package_spec.filepath
    # Create a new ROS message for each SBP message.
    msg_output_dir = output_dir + '/msg'
    if not os.path.exists(msg_output_dir):
        os.mkdir(msg_output_dir)
    ros_template = JENV.get_template(MESSAGES_TEMPLATE_NAME)
    for definition in package_spec.definitions:
        if is_deprecated(definition):
            continue
        destination_filename = '%s/%s.msg' % (msg_output_dir, to_identifier(definition.identifier))
        with open(destination_filename, 'w') as f:
            f.write(ros_template.render(
                name=name,
                message=definition
            ))

def render_conversions(output_dir, all_specs):
    # Create conversion files.
    # Header
    include_dir = output_dir + '/include'
    if not os.path.exists(include_dir):
        os.mkdir(include_dir)
    include_dir = include_dir + '/piksi_multi_msgs'
    if not os.path.exists(include_dir):
        os.mkdir(include_dir)

    ros_template = JENV.get_template(CONVERSION_TEMPLATE_NAME)
    destination_filename = '%s/conversion.h' % (include_dir)
    with open(destination_filename, 'w') as f:
        f.write(ros_template.render(
            all_specs=all_specs
        ))

    # Source
    src_dir = output_dir + '/src'
    if not os.path.exists(src_dir):
        os.mkdir(src_dir)

    ros_template = JENV.get_template(CONVERSION_SRC_TEMPLATE_NAME)
    destination_filename = '%s/conversion.cc' % (src_dir)
    with open(destination_filename, 'w') as f:
        f.write(ros_template.render(
            all_specs=all_specs
        ))

    # ROS callbacks
    export_dir = output_dir + '/../piksi_multi_cpp/include/piksi_multi_cpp/sbp_callback_handler/sbp_callback_handler_relay'
    ros_template = JENV.get_template(CALLBACK_TEMPLATE_NAME)
    destination_filename = '%s/sbp_callback_handler_relay_sbp.h' % (export_dir)
    with open(destination_filename, 'w') as f:
        f.write(ros_template.render(
            all_specs=all_specs
        ))
