#!/usr/bin/env python

from libsbp_ros_msgs.resources import *

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

def render_msgs(output_dir, all_specs, verbose):
    if verbose:
        print("Rendering messages.")

def render_conversion_header(output_dir, all_specs, verbose):
    if verbose:
        print("Rendering conversion header.")

def render_conversion_src(output_dir, all_specs, verbose):
    if verbose:
        print("Rendering conversion source.")

def render_cb_export(output_dir, all_specs, verbose):
    if verbose:
        print("Rendering callback export file.")
