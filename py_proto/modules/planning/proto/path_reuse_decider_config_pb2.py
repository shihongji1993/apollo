# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/planning/proto/path_reuse_decider_config.proto

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
  name='modules/planning/proto/path_reuse_decider_config.proto',
  package='apollo.planning',
  syntax='proto2',
  serialized_pb=_b('\n6modules/planning/proto/path_reuse_decider_config.proto\x12\x0f\x61pollo.planning\",\n\x16PathReuseDeciderConfig\x12\x12\n\nreuse_path\x18\x01 \x01(\x08')
)




_PATHREUSEDECIDERCONFIG = _descriptor.Descriptor(
  name='PathReuseDeciderConfig',
  full_name='apollo.planning.PathReuseDeciderConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='reuse_path', full_name='apollo.planning.PathReuseDeciderConfig.reuse_path', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
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
  serialized_start=75,
  serialized_end=119,
)

DESCRIPTOR.message_types_by_name['PathReuseDeciderConfig'] = _PATHREUSEDECIDERCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

PathReuseDeciderConfig = _reflection.GeneratedProtocolMessageType('PathReuseDeciderConfig', (_message.Message,), dict(
  DESCRIPTOR = _PATHREUSEDECIDERCONFIG,
  __module__ = 'modules.planning.proto.path_reuse_decider_config_pb2'
  # @@protoc_insertion_point(class_scope:apollo.planning.PathReuseDeciderConfig)
  ))
_sym_db.RegisterMessage(PathReuseDeciderConfig)


# @@protoc_insertion_point(module_scope)