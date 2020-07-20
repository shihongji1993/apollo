package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "usb_can",
    srcs = select(
        {
            ":x86_mode": [
                "include/libcontrolcan.so",
            ],
            ":arm_mode": [
                "include/libcontrolcan.so",
            ],
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ),
    hdrs = [
        "include/controlcan.h",
    ],
    include_prefix = "usb_can",
)

config_setting(
    name = "x86_mode",
    values = {"cpu": "k8"},
)

config_setting(
    name = "arm_mode",
    values = {"cpu": "arm"},
)
