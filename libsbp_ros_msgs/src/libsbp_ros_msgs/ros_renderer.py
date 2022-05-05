#!/usr/bin/env python3

import jinja2
import copy
import re

CALLBACK_TEMPLATE_NAME = 'callback_template.j2'
CONVERSION_TEMPLATE_HEADER_NAME = 'conversion_template_header.j2'
CONVERSION_TEMPLATE_SRC_NAME = 'conversion_template_src.j2'
ROS_MESSAGES_TEMPLATE_NAME = 'ros_message_template.j2'

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

def to_title(s):
    """
    Returns the title field from a spec name.
    """
    return s.title()

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
    """
    Creates a commented unit from a unit value.
    """
    return to_comment('[' + str(value) + ']')

def to_type(f, type_map=TYPE_MAP):
    """
    Builds a type name from a fields type_id.
    """
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

def to_identifier(s):
  """
  Convert snake_case to CamelCase.
  """
  if s.startswith('GPS'):
      s = 'Gps' + s[3:]
  if s.startswith('STEC'):
      s = 'Stec' + s[4:]
  if s.startswith('UART'):
      s = 'Uart' + s[4:]
  return ''.join([i.capitalize() for i in s.split('_')]) if '_' in s else s

def to_hex(value):
    """
    Convert decimal value to hex string.
    """
    return " 0x%0.4X" % value

def to_sbp_file_name(identifier):
    """
    Creates file name from sbp message identifier.
    """
    prefix = 'swiftnav.sbp.'
    if identifier.startswith(prefix):
        return identifier[len(prefix):]
    else:
        return identifier

def to_sbp_struct(identifier):
    """
    Convert identifier to c struct name.
    """
    if identifier.isupper():
        identifier = identifier.lower() + '_t'
    else:
        # https://stackoverflow.com/questions/1175208/elegant-python-function-to-convert-camelcase-to-snake-case
        identifier = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', identifier)
        identifier = re.sub('([a-z0-9])([A-Z])', r'\1_\2', identifier).lower() + '_t'
    # Handle these to cases individually.
    if identifier == 'gnss_signal_t' or identifier == 'gps_time_t':
        identifier = 'sbp_' + identifier
    return identifier

def to_topic(identifier):
    """
    Convert identifier to a uniform ROS topic name.
    """
    identifier = to_sbp_struct(identifier)
    suffix = '_t'
    if identifier.endswith(suffix):
        identifier = identifier[:-len(suffix)]
    prefix = 'sbp_'
    if identifier.startswith(prefix):
        identifier = identifier[len(prefix):]
    prefix = 'msg_'
    if identifier.startswith(prefix):
        identifier = identifier[len(prefix):]
    return identifier

def is_default_type(f, type_map=TYPE_MAP):
    """
    Check whether the field has a default type, e.g. u8 or s16.
    """
    name = f.type_id
    if type_map.get(name, None):
        return True

def is_default_array_type(f, type_map=TYPE_MAP):
    """
    Check whether the field is an array and is made up of default types, e.g. u8 or s16.
    """
    return f.type_id == 'array' and type_map.get(f.options['fill'].value, None)

def to_msg_definition(identifier):
    """
    Create message definition.
    """
    return "SBP_" + identifier

def render_msgs(output_dir, all_specs, verbose):
    """
    Renders a ROS message for each spec definition.
    """
    if verbose:
        print("Rendering messages.")
    template = JENV.get_template(ROS_MESSAGES_TEMPLATE_NAME)
    for spec in all_specs:
        path, name = spec.filepath
        for definition in spec.definitions:
            msg_file = '%s/%s.msg' % (output_dir, to_identifier(definition.identifier))
            with open(msg_file, 'w') as f:
                f.write(template.render(
                    name=name,
                    message=definition
                ))

def render_conversion_header(output_dir, all_specs, verbose):
    """
    Renders conversion definitions.
    """
    if verbose:
        print("Rendering conversion header.")
    ros_template = JENV.get_template(CONVERSION_TEMPLATE_HEADER_NAME)
    destination_filename = '%s/conversion.h' % (output_dir)
    with open(destination_filename, 'w') as f:
        f.write(ros_template.render(
            all_specs=all_specs
        ))


def render_conversion_src(output_dir, all_specs, verbose):
    """
    Renders conversion implementation.
    """
    if verbose:
        print("Rendering conversion source.")
    ros_template = JENV.get_template(CONVERSION_TEMPLATE_SRC_NAME)
    destination_filename = '%s/conversion.cc' % (output_dir)
    with open(destination_filename, 'w') as f:
        f.write(ros_template.render(
            all_specs=all_specs
        ))

def render_cb_export(output_dir, all_specs, verbose):
    """
    Renders callback handler.
    """
    if verbose:
        print("Rendering callback export file.")
    ros_template = JENV.get_template(CALLBACK_TEMPLATE_NAME)
    destination_filename = '%s/sbp_relays.h' % (output_dir)
    with open(destination_filename, 'w') as f:
        f.write(ros_template.render(
            all_specs=all_specs
        ))



def setup_environment(source_dir):
    print("Setting up JENV with source_dir: %s" % (source_dir))
    global JENV
    JENV = jinja2.Environment(block_start_string = '((*',
                              block_end_string = '*))',
                              variable_start_string = '(((',
                              variable_end_string = ')))',
                              comment_start_string = '((=',
                              comment_end_string = '=))',
                              loader=jinja2.FileSystemLoader(searchpath=source_dir + '/resources'),
                              )

    JENV.filters['to_title'] = to_title
    JENV.filters['to_comment'] = to_comment
    JENV.filters['to_unit'] = to_unit
    JENV.filters['to_type'] = to_type
    JENV.filters['to_identifier'] = to_identifier
    JENV.filters['to_hex'] = to_hex
    JENV.filters['to_sbp_file_name'] = to_sbp_file_name
    JENV.filters['to_sbp_struct'] = to_sbp_struct
    JENV.filters['to_topic'] = to_topic
    JENV.filters['is_default_type'] = is_default_type
    JENV.filters['is_default_array_type'] = is_default_array_type
    JENV.filters['to_msg_definition'] = to_msg_definition
