# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/planning/proto/fem_pos_deviation_smoother_config.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/planning/proto/fem_pos_deviation_smoother_config.proto',
  package='apollo.planning',
  syntax='proto2',
  serialized_pb=_b('\n>modules/planning/proto/fem_pos_deviation_smoother_config.proto\x12\x0f\x61pollo.planning\"\xb6\x05\n\x1d\x46\x65mPosDeviationSmootherConfig\x12-\n\x18weight_fem_pos_deviation\x18\x02 \x01(\x01:\x0b\x31\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x12\x1f\n\x14weight_ref_deviation\x18\x03 \x01(\x01:\x01\x31\x12\x1d\n\x12weight_path_length\x18\x04 \x01(\x01:\x01\x31\x12)\n\x1a\x61pply_curvature_constraint\x18\x05 \x01(\x08:\x05\x66\x61lse\x12\x32\n%weight_curvature_constraint_slack_var\x18\x06 \x01(\x01:\x03\x31\x30\x30\x12!\n\x14\x63urvature_constraint\x18\x07 \x01(\x01:\x03\x30.2\x12\x16\n\x07use_sqp\x18\x08 \x01(\x08:\x05\x66\x61lse\x12\x18\n\x08sqp_ftol\x18\t \x01(\x01:\x06\x30.0001\x12\x17\n\x08sqp_ctol\x18\n \x01(\x01:\x05\x30.001\x12\x1c\n\x10sqp_pen_max_iter\x18\x0b \x01(\x05:\x02\x31\x30\x12\x1d\n\x10sqp_sub_max_iter\x18\x0c \x01(\x05:\x03\x31\x30\x30\x12\x15\n\x08max_iter\x18\x64 \x01(\x05:\x03\x35\x30\x30\x12\x15\n\ntime_limit\x18\x65 \x01(\x01:\x01\x30\x12\x16\n\x07verbose\x18\x66 \x01(\x08:\x05\x66\x61lse\x12 \n\x12scaled_termination\x18g \x01(\x08:\x04true\x12\x18\n\nwarm_start\x18h \x01(\x08:\x04true\x12\x17\n\x0bprint_level\x18\xc8\x01 \x01(\x05:\x01\x30\x12#\n\x15max_num_of_iterations\x18\xc9\x01 \x01(\x05:\x03\x35\x30\x30\x12)\n\x1c\x61\x63\x63\x65ptable_num_of_iterations\x18\xca\x01 \x01(\x05:\x02\x31\x35\x12\x13\n\x03tol\x18\xcb\x01 \x01(\x01:\x05\x31\x65-08\x12\x1c\n\x0e\x61\x63\x63\x65ptable_tol\x18\xcc\x01 \x01(\x01:\x03\x30.1')
)




_FEMPOSDEVIATIONSMOOTHERCONFIG = _descriptor.Descriptor(
  name='FemPosDeviationSmootherConfig',
  full_name='apollo.planning.FemPosDeviationSmootherConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='weight_fem_pos_deviation', full_name='apollo.planning.FemPosDeviationSmootherConfig.weight_fem_pos_deviation', index=0,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(10000000000),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='weight_ref_deviation', full_name='apollo.planning.FemPosDeviationSmootherConfig.weight_ref_deviation', index=1,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(1),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='weight_path_length', full_name='apollo.planning.FemPosDeviationSmootherConfig.weight_path_length', index=2,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(1),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='apply_curvature_constraint', full_name='apollo.planning.FemPosDeviationSmootherConfig.apply_curvature_constraint', index=3,
      number=5, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='weight_curvature_constraint_slack_var', full_name='apollo.planning.FemPosDeviationSmootherConfig.weight_curvature_constraint_slack_var', index=4,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(100),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='curvature_constraint', full_name='apollo.planning.FemPosDeviationSmootherConfig.curvature_constraint', index=5,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0.2),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='use_sqp', full_name='apollo.planning.FemPosDeviationSmootherConfig.use_sqp', index=6,
      number=8, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='sqp_ftol', full_name='apollo.planning.FemPosDeviationSmootherConfig.sqp_ftol', index=7,
      number=9, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0.0001),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='sqp_ctol', full_name='apollo.planning.FemPosDeviationSmootherConfig.sqp_ctol', index=8,
      number=10, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0.001),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='sqp_pen_max_iter', full_name='apollo.planning.FemPosDeviationSmootherConfig.sqp_pen_max_iter', index=9,
      number=11, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=10,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='sqp_sub_max_iter', full_name='apollo.planning.FemPosDeviationSmootherConfig.sqp_sub_max_iter', index=10,
      number=12, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=100,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='max_iter', full_name='apollo.planning.FemPosDeviationSmootherConfig.max_iter', index=11,
      number=100, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=500,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='time_limit', full_name='apollo.planning.FemPosDeviationSmootherConfig.time_limit', index=12,
      number=101, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='verbose', full_name='apollo.planning.FemPosDeviationSmootherConfig.verbose', index=13,
      number=102, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='scaled_termination', full_name='apollo.planning.FemPosDeviationSmootherConfig.scaled_termination', index=14,
      number=103, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=True,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='warm_start', full_name='apollo.planning.FemPosDeviationSmootherConfig.warm_start', index=15,
      number=104, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=True,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='print_level', full_name='apollo.planning.FemPosDeviationSmootherConfig.print_level', index=16,
      number=200, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='max_num_of_iterations', full_name='apollo.planning.FemPosDeviationSmootherConfig.max_num_of_iterations', index=17,
      number=201, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=500,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='acceptable_num_of_iterations', full_name='apollo.planning.FemPosDeviationSmootherConfig.acceptable_num_of_iterations', index=18,
      number=202, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=15,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='tol', full_name='apollo.planning.FemPosDeviationSmootherConfig.tol', index=19,
      number=203, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(1e-08),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='acceptable_tol', full_name='apollo.planning.FemPosDeviationSmootherConfig.acceptable_tol', index=20,
      number=204, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0.1),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=84,
  serialized_end=778,
)

DESCRIPTOR.message_types_by_name['FemPosDeviationSmootherConfig'] = _FEMPOSDEVIATIONSMOOTHERCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

FemPosDeviationSmootherConfig = _reflection.GeneratedProtocolMessageType('FemPosDeviationSmootherConfig', (_message.Message,), dict(
  DESCRIPTOR = _FEMPOSDEVIATIONSMOOTHERCONFIG,
  __module__ = 'modules.planning.proto.fem_pos_deviation_smoother_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.FemPosDeviationSmootherConfig)
  ))
_sym_db.RegisterMessage(FemPosDeviationSmootherConfig)


# @@protoc_insertion_point(module_scope)
