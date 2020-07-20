# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/drivers/gnss/proto/gnss_best_pose.proto

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


from modules.common.proto import header_pb2 as modules_dot_common_dot_proto_dot_header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/drivers/gnss/proto/gnss_best_pose.proto',
  package='apollo.drivers.gnss',
  syntax='proto2',
  serialized_pb=_b('\n/modules/drivers/gnss/proto/gnss_best_pose.proto\x12\x13\x61pollo.drivers.gnss\x1a!modules/common/proto/header.proto\"\xaa\x05\n\x0cGnssBestPose\x12%\n\x06header\x18\x01 \x01(\x0b\x32\x15.apollo.common.Header\x12\x18\n\x10measurement_time\x18\x02 \x01(\x01\x12\x37\n\nsol_status\x18\x03 \x01(\x0e\x32#.apollo.drivers.gnss.SolutionStatus\x12\x33\n\x08sol_type\x18\x04 \x01(\x0e\x32!.apollo.drivers.gnss.SolutionType\x12\x10\n\x08latitude\x18\x05 \x01(\x01\x12\x11\n\tlongitude\x18\x06 \x01(\x01\x12\x12\n\nheight_msl\x18\x07 \x01(\x01\x12\x12\n\nundulation\x18\x08 \x01(\x02\x12.\n\x08\x64\x61tum_id\x18\t \x01(\x0e\x32\x1c.apollo.drivers.gnss.DatumId\x12\x18\n\x10latitude_std_dev\x18\n \x01(\x02\x12\x19\n\x11longitude_std_dev\x18\x0b \x01(\x02\x12\x16\n\x0eheight_std_dev\x18\x0c \x01(\x02\x12\x17\n\x0f\x62\x61se_station_id\x18\r \x01(\x0c\x12\x18\n\x10\x64ifferential_age\x18\x0e \x01(\x02\x12\x14\n\x0csolution_age\x18\x0f \x01(\x02\x12\x18\n\x10num_sats_tracked\x18\x10 \x01(\r\x12\x1c\n\x14num_sats_in_solution\x18\x11 \x01(\r\x12\x13\n\x0bnum_sats_l1\x18\x12 \x01(\r\x12\x16\n\x0enum_sats_multi\x18\x13 \x01(\r\x12\x10\n\x08reserved\x18\x14 \x01(\r\x12 \n\x18\x65xtended_solution_status\x18\x15 \x01(\r\x12 \n\x18galileo_beidou_used_mask\x18\x16 \x01(\r\x12\x1d\n\x15gps_glonass_used_mask\x18\x17 \x01(\r*\x90\x02\n\x0eSolutionStatus\x12\x10\n\x0cSOL_COMPUTED\x10\x00\x12\x14\n\x10INSUFFICIENT_OBS\x10\x01\x12\x12\n\x0eNO_CONVERGENCE\x10\x02\x12\x0f\n\x0bSINGULARITY\x10\x03\x12\r\n\tCOV_TRACE\x10\x04\x12\r\n\tTEST_DIST\x10\x05\x12\x0e\n\nCOLD_START\x10\x06\x12\r\n\tV_H_LIMIT\x10\x07\x12\x0c\n\x08VARIANCE\x10\x08\x12\r\n\tRESIDUALS\x10\t\x12\x15\n\x11INTEGRITY_WARNING\x10\r\x12\x0b\n\x07PENDING\x10\x12\x12\x0f\n\x0bINVALID_FIX\x10\x13\x12\x10\n\x0cUNAUTHORIZED\x10\x14\x12\x10\n\x0cINVALID_RATE\x10\x16*\x9f\x04\n\x0cSolutionType\x12\x08\n\x04NONE\x10\x00\x12\x0c\n\x08\x46IXEDPOS\x10\x01\x12\x0f\n\x0b\x46IXEDHEIGHT\x10\x02\x12\r\n\tFLOATCONV\x10\x04\x12\x0c\n\x08WIDELANE\x10\x05\x12\x0e\n\nNARROWLANE\x10\x06\x12\x14\n\x10\x44OPPLER_VELOCITY\x10\x08\x12\n\n\x06SINGLE\x10\x10\x12\x0b\n\x07PSRDIFF\x10\x11\x12\x08\n\x04WAAS\x10\x12\x12\x0e\n\nPROPOGATED\x10\x13\x12\x0c\n\x08OMNISTAR\x10\x14\x12\x0c\n\x08L1_FLOAT\x10 \x12\x12\n\x0eIONOFREE_FLOAT\x10!\x12\x10\n\x0cNARROW_FLOAT\x10\"\x12\n\n\x06L1_INT\x10\x30\x12\x0c\n\x08WIDE_INT\x10\x31\x12\x0e\n\nNARROW_INT\x10\x32\x12\x12\n\x0eRTK_DIRECT_INS\x10\x33\x12\x0c\n\x08INS_SBAS\x10\x34\x12\r\n\tINS_PSRSP\x10\x35\x12\x0f\n\x0bINS_PSRDIFF\x10\x36\x12\x10\n\x0cINS_RTKFLOAT\x10\x37\x12\x10\n\x0cINS_RTKFIXED\x10\x38\x12\x10\n\x0cINS_OMNISTAR\x10\x39\x12\x13\n\x0fINS_OMNISTAR_HP\x10:\x12\x13\n\x0fINS_OMNISTAR_XP\x10;\x12\x0f\n\x0bOMNISTAR_HP\x10@\x12\x0f\n\x0bOMNISTAR_XP\x10\x41\x12\x12\n\x0ePPP_CONVERGING\x10\x44\x12\x07\n\x03PPP\x10\x45\x12\x16\n\x12INS_PPP_CONVERGING\x10I\x12\x0b\n\x07INS_PPP\x10J*\x14\n\x07\x44\x61tumId\x12\t\n\x05WGS84\x10=')
  ,
  dependencies=[modules_dot_common_dot_proto_dot_header__pb2.DESCRIPTOR,])

_SOLUTIONSTATUS = _descriptor.EnumDescriptor(
  name='SolutionStatus',
  full_name='apollo.drivers.gnss.SolutionStatus',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='SOL_COMPUTED', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INSUFFICIENT_OBS', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NO_CONVERGENCE', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SINGULARITY', index=3, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COV_TRACE', index=4, number=4,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TEST_DIST', index=5, number=5,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COLD_START', index=6, number=6,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='V_H_LIMIT', index=7, number=7,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='VARIANCE', index=8, number=8,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RESIDUALS', index=9, number=9,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INTEGRITY_WARNING', index=10, number=13,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PENDING', index=11, number=18,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INVALID_FIX', index=12, number=19,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UNAUTHORIZED', index=13, number=20,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INVALID_RATE', index=14, number=22,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=793,
  serialized_end=1065,
)
_sym_db.RegisterEnumDescriptor(_SOLUTIONSTATUS)

SolutionStatus = enum_type_wrapper.EnumTypeWrapper(_SOLUTIONSTATUS)
_SOLUTIONTYPE = _descriptor.EnumDescriptor(
  name='SolutionType',
  full_name='apollo.drivers.gnss.SolutionType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NONE', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FIXEDPOS', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FIXEDHEIGHT', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FLOATCONV', index=3, number=4,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='WIDELANE', index=4, number=5,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NARROWLANE', index=5, number=6,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DOPPLER_VELOCITY', index=6, number=8,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SINGLE', index=7, number=16,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PSRDIFF', index=8, number=17,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='WAAS', index=9, number=18,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PROPOGATED', index=10, number=19,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='OMNISTAR', index=11, number=20,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='L1_FLOAT', index=12, number=32,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='IONOFREE_FLOAT', index=13, number=33,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NARROW_FLOAT', index=14, number=34,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='L1_INT', index=15, number=48,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='WIDE_INT', index=16, number=49,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NARROW_INT', index=17, number=50,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RTK_DIRECT_INS', index=18, number=51,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_SBAS', index=19, number=52,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_PSRSP', index=20, number=53,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_PSRDIFF', index=21, number=54,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_RTKFLOAT', index=22, number=55,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_RTKFIXED', index=23, number=56,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_OMNISTAR', index=24, number=57,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_OMNISTAR_HP', index=25, number=58,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_OMNISTAR_XP', index=26, number=59,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='OMNISTAR_HP', index=27, number=64,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='OMNISTAR_XP', index=28, number=65,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PPP_CONVERGING', index=29, number=68,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PPP', index=30, number=69,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_PPP_CONVERGING', index=31, number=73,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_PPP', index=32, number=74,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=1068,
  serialized_end=1611,
)
_sym_db.RegisterEnumDescriptor(_SOLUTIONTYPE)

SolutionType = enum_type_wrapper.EnumTypeWrapper(_SOLUTIONTYPE)
_DATUMID = _descriptor.EnumDescriptor(
  name='DatumId',
  full_name='apollo.drivers.gnss.DatumId',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='WGS84', index=0, number=61,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=1613,
  serialized_end=1633,
)
_sym_db.RegisterEnumDescriptor(_DATUMID)

DatumId = enum_type_wrapper.EnumTypeWrapper(_DATUMID)
SOL_COMPUTED = 0
INSUFFICIENT_OBS = 1
NO_CONVERGENCE = 2
SINGULARITY = 3
COV_TRACE = 4
TEST_DIST = 5
COLD_START = 6
V_H_LIMIT = 7
VARIANCE = 8
RESIDUALS = 9
INTEGRITY_WARNING = 13
PENDING = 18
INVALID_FIX = 19
UNAUTHORIZED = 20
INVALID_RATE = 22
NONE = 0
FIXEDPOS = 1
FIXEDHEIGHT = 2
FLOATCONV = 4
WIDELANE = 5
NARROWLANE = 6
DOPPLER_VELOCITY = 8
SINGLE = 16
PSRDIFF = 17
WAAS = 18
PROPOGATED = 19
OMNISTAR = 20
L1_FLOAT = 32
IONOFREE_FLOAT = 33
NARROW_FLOAT = 34
L1_INT = 48
WIDE_INT = 49
NARROW_INT = 50
RTK_DIRECT_INS = 51
INS_SBAS = 52
INS_PSRSP = 53
INS_PSRDIFF = 54
INS_RTKFLOAT = 55
INS_RTKFIXED = 56
INS_OMNISTAR = 57
INS_OMNISTAR_HP = 58
INS_OMNISTAR_XP = 59
OMNISTAR_HP = 64
OMNISTAR_XP = 65
PPP_CONVERGING = 68
PPP = 69
INS_PPP_CONVERGING = 73
INS_PPP = 74
WGS84 = 61



_GNSSBESTPOSE = _descriptor.Descriptor(
  name='GnssBestPose',
  full_name='apollo.drivers.gnss.GnssBestPose',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='apollo.drivers.gnss.GnssBestPose.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='measurement_time', full_name='apollo.drivers.gnss.GnssBestPose.measurement_time', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='sol_status', full_name='apollo.drivers.gnss.GnssBestPose.sol_status', index=2,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='sol_type', full_name='apollo.drivers.gnss.GnssBestPose.sol_type', index=3,
      number=4, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='latitude', full_name='apollo.drivers.gnss.GnssBestPose.latitude', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='longitude', full_name='apollo.drivers.gnss.GnssBestPose.longitude', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='height_msl', full_name='apollo.drivers.gnss.GnssBestPose.height_msl', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='undulation', full_name='apollo.drivers.gnss.GnssBestPose.undulation', index=7,
      number=8, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='datum_id', full_name='apollo.drivers.gnss.GnssBestPose.datum_id', index=8,
      number=9, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=61,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='latitude_std_dev', full_name='apollo.drivers.gnss.GnssBestPose.latitude_std_dev', index=9,
      number=10, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='longitude_std_dev', full_name='apollo.drivers.gnss.GnssBestPose.longitude_std_dev', index=10,
      number=11, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='height_std_dev', full_name='apollo.drivers.gnss.GnssBestPose.height_std_dev', index=11,
      number=12, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='base_station_id', full_name='apollo.drivers.gnss.GnssBestPose.base_station_id', index=12,
      number=13, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='differential_age', full_name='apollo.drivers.gnss.GnssBestPose.differential_age', index=13,
      number=14, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='solution_age', full_name='apollo.drivers.gnss.GnssBestPose.solution_age', index=14,
      number=15, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='num_sats_tracked', full_name='apollo.drivers.gnss.GnssBestPose.num_sats_tracked', index=15,
      number=16, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='num_sats_in_solution', full_name='apollo.drivers.gnss.GnssBestPose.num_sats_in_solution', index=16,
      number=17, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='num_sats_l1', full_name='apollo.drivers.gnss.GnssBestPose.num_sats_l1', index=17,
      number=18, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='num_sats_multi', full_name='apollo.drivers.gnss.GnssBestPose.num_sats_multi', index=18,
      number=19, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='reserved', full_name='apollo.drivers.gnss.GnssBestPose.reserved', index=19,
      number=20, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='extended_solution_status', full_name='apollo.drivers.gnss.GnssBestPose.extended_solution_status', index=20,
      number=21, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='galileo_beidou_used_mask', full_name='apollo.drivers.gnss.GnssBestPose.galileo_beidou_used_mask', index=21,
      number=22, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='gps_glonass_used_mask', full_name='apollo.drivers.gnss.GnssBestPose.gps_glonass_used_mask', index=22,
      number=23, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
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
  serialized_start=108,
  serialized_end=790,
)

_GNSSBESTPOSE.fields_by_name['header'].message_type = modules_dot_common_dot_proto_dot_header__pb2._HEADER
_GNSSBESTPOSE.fields_by_name['sol_status'].enum_type = _SOLUTIONSTATUS
_GNSSBESTPOSE.fields_by_name['sol_type'].enum_type = _SOLUTIONTYPE
_GNSSBESTPOSE.fields_by_name['datum_id'].enum_type = _DATUMID
DESCRIPTOR.message_types_by_name['GnssBestPose'] = _GNSSBESTPOSE
DESCRIPTOR.enum_types_by_name['SolutionStatus'] = _SOLUTIONSTATUS
DESCRIPTOR.enum_types_by_name['SolutionType'] = _SOLUTIONTYPE
DESCRIPTOR.enum_types_by_name['DatumId'] = _DATUMID
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

GnssBestPose = _reflection.GeneratedProtocolMessageType('GnssBestPose', (_message.Message,), dict(
  DESCRIPTOR = _GNSSBESTPOSE,
  __module__ = 'modules.drivers.gnss.proto.gnss_best_pose_pb2'
  # @@protoc_insertion_point(class_scope:apollo.drivers.gnss.GnssBestPose)
  ))
_sym_db.RegisterMessage(GnssBestPose)


# @@protoc_insertion_point(module_scope)
