load("@pip_requirements//:requirements.bzl", "requirement")

filegroup(
    name = "clang_tidy_config",
    data = [".clang-tidy"],
)

py_binary(
    name = "setup",
    srcs = [
        "setup.py",
    ],
    data = [
        "README.md",
        "setup.cfg",
        "//src",
    ],
    main = "setup.py",
    python_version = "PY3",
    deps = [
        requirement("setuptools"),
        requirement("wheel"),
    ],
)