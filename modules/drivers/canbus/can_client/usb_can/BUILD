load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "usb_can_client",
    srcs = ["usb_can_client.cc" ],
    hdrs = ["usb_can_client.h",
        "controlcan.h"
    ],
    deps = [
        "//modules/drivers/canbus:sensor_gflags",
        "//modules/drivers/canbus/can_client",
        "//third_party/can_card_library/usb_can",
    ],
)

cc_test(
    name = "usb_can_client_test",
    size = "small",
    srcs = [
        "usb_can_client_test.cc",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/canbus/can_client/usb_can:usb_can_client",
        "//modules/drivers/canbus/common:canbus_common",
        "@gtest//:main",
    ],
)

cpplint()
