package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "cub_radar_proto",
    deps = [
        ":cub_radar_proto_lib",
    ],
)

proto_library(
    name = "cub_radar_proto_lib",
    srcs = [
        "cub_radar_conf.proto",
    ],
    deps = [
        "//modules/drivers/canbus/proto:canbus_proto_lib",
    ],
)
