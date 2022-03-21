load("@pip_requirements//:requirements.bzl", "requirement")
load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "curling",
    hdrs = ["curling.h",'core.h'],
    srcs =['core.cc','generator.cc','json.hpp','objects.hpp'],
    deps = [
        "@eigen",
        "//envpool/core:async_envpool",
         "@pybind11",
    ]
)

pybind_extension(
    name = "classic_control_envpool",
    srcs = [
        "classic_control.cc"
    ],
    deps = [
        "//envpool/core:py_envpool",
        ":curling",
    ]
)

py_library(
    name = "classic_control",
    srcs = ["__init__.py"],
    data = [":classic_control_envpool.so"],
    deps = ["//envpool/python:api"],
)

py_test(
    name = "classic_control_test",
    srcs = ["classic_control_test.py"],
    deps = [
        ":classic_control",
        requirement("numpy"),
        requirement("absl-py"),
        requirement("pygame"),
        requirement("matplotlib"),
        requirement("opencv-python-headless"),
        requirement("ptvsd"),
    ],
    imports=[":classic_control_envpool.so"],
    data = [":classic_control_envpool.so"],
)

py_binary(
    name = "train_emit",
    srcs = ["train/train_emit.py"],
    deps = [
        "//envpool",
        requirement("numpy"),
        requirement("absl-py"),
        requirement("pygame"),
        requirement("matplotlib"),
        requirement("opencv-python-headless"),
        requirement("ptvsd"),
        requirement("torch"),
        requirement("tianshou"),
    ]
)


py_library(
    name = "classic_control_registration",
    srcs = ["registration.py"],
    deps = [
        "//envpool:registration",
    ],
)

cc_test(
    name = "core_test_test",
    srcs = ["core_test.cc"],
    deps = [
        ":curling",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)
