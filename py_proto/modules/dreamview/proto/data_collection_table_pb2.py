# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/dreamview/proto/data_collection_table.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/dreamview/proto/data_collection_table.proto',
  package='apollo.dreamview',
  syntax='proto2',
  serialized_pb=_b('\n3modules/dreamview/proto/data_collection_table.proto\x12\x10\x61pollo.dreamview\"\x84\x01\n\tCriterion\x12\r\n\x05\x66ield\x18\x01 \x01(\t\x12\x41\n\x13\x63omparison_operator\x18\x02 \x01(\x0e\x32$.apollo.dreamview.ComparisonOperator\x12\r\n\x05value\x18\x03 \x01(\x02\x12\x16\n\x0evehicle_config\x18\x04 \x01(\t\"E\n\x05Range\x12\x0c\n\x04name\x18\x01 \x01(\t\x12.\n\tcriterion\x18\x02 \x03(\x0b\x32\x1b.apollo.dreamview.Criterion\"?\n\x07\x46\x65\x61ture\x12\x0c\n\x04name\x18\x01 \x01(\t\x12&\n\x05range\x18\x02 \x03(\x0b\x32\x17.apollo.dreamview.Range\"6\n\x08Scenario\x12*\n\x07\x66\x65\x61ture\x18\x01 \x03(\x0b\x32\x19.apollo.dreamview.Feature\"\xd8\x01\n\x13\x44\x61taCollectionTable\x12\x45\n\x08scenario\x18\x01 \x03(\x0b\x32\x33.apollo.dreamview.DataCollectionTable.ScenarioEntry\x12\x17\n\x0f\x66rame_threshold\x18\x02 \x02(\r\x12\x14\n\x0ctotal_frames\x18\x03 \x02(\r\x1aK\n\rScenarioEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12)\n\x05value\x18\x02 \x01(\x0b\x32\x1a.apollo.dreamview.Scenario:\x02\x38\x01*\x82\x01\n\x12\x43omparisonOperator\x12\t\n\x05\x45QUAL\x10\x00\x12\r\n\tNOT_EQUAL\x10\x01\x12\x10\n\x0cGREATER_THAN\x10\x02\x12\x19\n\x15GREATER_THAN_OR_EQUAL\x10\x03\x12\r\n\tLESS_THAN\x10\x04\x12\x16\n\x12LESS_THAN_OR_EQUAL\x10\x05')
)

_COMPARISONOPERATOR = _descriptor.EnumDescriptor(
  name='ComparisonOperator',
  full_name='apollo.dreamview.ComparisonOperator',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='EQUAL', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NOT_EQUAL', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GREATER_THAN', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GREATER_THAN_OR_EQUAL', index=3, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LESS_THAN', index=4, number=4,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LESS_THAN_OR_EQUAL', index=5, number=5,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=620,
  serialized_end=750,
)
_sym_db.RegisterEnumDescriptor(_COMPARISONOPERATOR)

ComparisonOperator = enum_type_wrapper.EnumTypeWrapper(_COMPARISONOPERATOR)
EQUAL = 0
NOT_EQUAL = 1
GREATER_THAN = 2
GREATER_THAN_OR_EQUAL = 3
LESS_THAN = 4
LESS_THAN_OR_EQUAL = 5



_CRITERION = _descriptor.Descriptor(
  name='Criterion',
  full_name='apollo.dreamview.Criterion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='field', full_name='apollo.dreamview.Criterion.field', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='comparison_operator', full_name='apollo.dreamview.Criterion.comparison_operator', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.dreamview.Criterion.value', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='vehicle_config', full_name='apollo.dreamview.Criterion.vehicle_config', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
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
  serialized_start=74,
  serialized_end=206,
)


_RANGE = _descriptor.Descriptor(
  name='Range',
  full_name='apollo.dreamview.Range',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.dreamview.Range.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='criterion', full_name='apollo.dreamview.Range.criterion', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=208,
  serialized_end=277,
)


_FEATURE = _descriptor.Descriptor(
  name='Feature',
  full_name='apollo.dreamview.Feature',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.dreamview.Feature.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='range', full_name='apollo.dreamview.Feature.range', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=279,
  serialized_end=342,
)


_SCENARIO = _descriptor.Descriptor(
  name='Scenario',
  full_name='apollo.dreamview.Scenario',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='feature', full_name='apollo.dreamview.Scenario.feature', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=344,
  serialized_end=398,
)


_DATACOLLECTIONTABLE_SCENARIOENTRY = _descriptor.Descriptor(
  name='ScenarioEntry',
  full_name='apollo.dreamview.DataCollectionTable.ScenarioEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='apollo.dreamview.DataCollectionTable.ScenarioEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='apollo.dreamview.DataCollectionTable.ScenarioEntry.value', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=_descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001')),
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=542,
  serialized_end=617,
)

_DATACOLLECTIONTABLE = _descriptor.Descriptor(
  name='DataCollectionTable',
  full_name='apollo.dreamview.DataCollectionTable',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='scenario', full_name='apollo.dreamview.DataCollectionTable.scenario', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='frame_threshold', full_name='apollo.dreamview.DataCollectionTable.frame_threshold', index=1,
      number=2, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='total_frames', full_name='apollo.dreamview.DataCollectionTable.total_frames', index=2,
      number=3, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_DATACOLLECTIONTABLE_SCENARIOENTRY, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=401,
  serialized_end=617,
)

_CRITERION.fields_by_name['comparison_operator'].enum_type = _COMPARISONOPERATOR
_RANGE.fields_by_name['criterion'].message_type = _CRITERION
_FEATURE.fields_by_name['range'].message_type = _RANGE
_SCENARIO.fields_by_name['feature'].message_type = _FEATURE
_DATACOLLECTIONTABLE_SCENARIOENTRY.fields_by_name['value'].message_type = _SCENARIO
_DATACOLLECTIONTABLE_SCENARIOENTRY.containing_type = _DATACOLLECTIONTABLE
_DATACOLLECTIONTABLE.fields_by_name['scenario'].message_type = _DATACOLLECTIONTABLE_SCENARIOENTRY
DESCRIPTOR.message_types_by_name['Criterion'] = _CRITERION
DESCRIPTOR.message_types_by_name['Range'] = _RANGE
DESCRIPTOR.message_types_by_name['Feature'] = _FEATURE
DESCRIPTOR.message_types_by_name['Scenario'] = _SCENARIO
DESCRIPTOR.message_types_by_name['DataCollectionTable'] = _DATACOLLECTIONTABLE
DESCRIPTOR.enum_types_by_name['ComparisonOperator'] = _COMPARISONOPERATOR
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Criterion = _reflection.GeneratedProtocolMessageType('Criterion', (_message.Message,), dict(
  DESCRIPTOR = _CRITERION,
  __module__ = 'modules.dreamview.proto.data_collection_table_pb2'
  # @@protoc_insertion_point(class_scope:apollo.dreamview.Criterion)
  ))
_sym_db.RegisterMessage(Criterion)

Range = _reflection.GeneratedProtocolMessageType('Range', (_message.Message,), dict(
  DESCRIPTOR = _RANGE,
  __module__ = 'modules.dreamview.proto.data_collection_table_pb2'
  # @@protoc_insertion_point(class_scope:apollo.dreamview.Range)
  ))
_sym_db.RegisterMessage(Range)

Feature = _reflection.GeneratedProtocolMessageType('Feature', (_message.Message,), dict(
  DESCRIPTOR = _FEATURE,
  __module__ = 'modules.dreamview.proto.data_collection_table_pb2'
  # @@protoc_insertion_point(class_scope:apollo.dreamview.Feature)
  ))
_sym_db.RegisterMessage(Feature)

Scenario = _reflection.GeneratedProtocolMessageType('Scenario', (_message.Message,), dict(
  DESCRIPTOR = _SCENARIO,
  __module__ = 'modules.dreamview.proto.data_collection_table_pb2'
  # @@protoc_insertion_point(class_scope:apollo.dreamview.Scenario)
  ))
_sym_db.RegisterMessage(Scenario)

DataCollectionTable = _reflection.GeneratedProtocolMessageType('DataCollectionTable', (_message.Message,), dict(

  ScenarioEntry = _reflection.GeneratedProtocolMessageType('ScenarioEntry', (_message.Message,), dict(
    DESCRIPTOR = _DATACOLLECTIONTABLE_SCENARIOENTRY,
    __module__ = 'modules.dreamview.proto.data_collection_table_pb2'
    # @@protoc_insertion_point(class_scope:apollo.dreamview.DataCollectionTable.ScenarioEntry)
    ))
  ,
  DESCRIPTOR = _DATACOLLECTIONTABLE,
  __module__ = 'modules.dreamview.proto.data_collection_table_pb2'
  # @@protoc_insertion_point(class_scope:apollo.dreamview.DataCollectionTable)
  ))
_sym_db.RegisterMessage(DataCollectionTable)
_sym_db.RegisterMessage(DataCollectionTable.ScenarioEntry)


_DATACOLLECTIONTABLE_SCENARIOENTRY.has_options = True
_DATACOLLECTIONTABLE_SCENARIOENTRY._options = _descriptor._ParseOptions(descriptor_pb2.MessageOptions(), _b('8\001'))
# @@protoc_insertion_point(module_scope)
