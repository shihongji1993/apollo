load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cub_radar_message_manager",
    srcs = ["cub_radar_message_manager.cc"],
    hdrs = ["cub_radar_message_manager.h"],
    deps = [
        "//modules/common/util:message_util",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/radar/cub_radar/protocol:drivers_cub_radar_protocol",
    ],
)

cc_library(
    name = "cub_radar_canbus_lib",
    srcs = ["cub_radar_canbus_component.cc"],
    hdrs = ["cub_radar_canbus_component.h"],
    copts = ['-DMODULE_NAME=\\"cub_radar\\"'],
    deps = [
        ":cub_radar_message_manager",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/radar/cub_radar/protocol:drivers_cub_radar_protocol",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:pose_proto",
        "@eigen",
    ],
)

cc_binary(
    name = "libcub_radar.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":cub_radar_canbus_lib"],
)

cpplint()

