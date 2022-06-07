# Copyright 2021 Garena Online Private Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""EnvPool workspace initialization, this is loaded in WORKSPACE."""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def workspace():
    """Load requested packages."""
    maybe(
        http_archive,
        name = "rules_python",
        sha256 = "954aa89b491be4a083304a2cb838019c8b8c3720a7abb9c4cb81ac7a24230cea",
        urls = [
            "https://mirror.bazel.build/github.com/bazelbuild/rules_python/releases/download/0.4.0/rules_python-0.4.0.tar.gz",
            "https://github.com/bazelbuild/rules_python/releases/download/0.4.0/rules_python-0.4.0.tar.gz",
        ],
    )

    maybe(
        http_archive,
        name = "rules_foreign_cc",
        sha256 = "69023642d5781c68911beda769f91fcbc8ca48711db935a75da7f6536b65047f",
        strip_prefix = "rules_foreign_cc-0.6.0",
        url = "https://github.com/bazelbuild/rules_foreign_cc/archive/0.6.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "pybind11_bazel",
        sha256 = "a5666d950c3344a8b0d3892a88dc6b55c8e0c78764f9294e806d69213c03f19d",
        strip_prefix = "pybind11_bazel-26973c0ff320cb4b39e45bc3e4297b82bc3a6c09",
        urls = [
            "https://github.com/pybind/pybind11_bazel/archive/26973c0ff320cb4b39e45bc3e4297b82bc3a6c09.zip",
        ],
    )

    maybe(
        http_archive,
        name = "pybind11",
        build_file = "@pybind11_bazel//:pybind11.BUILD",
        sha256 = "8ff2fff22df038f5cd02cea8af56622bc67f5b64534f1b83b9f133b8366acff2",
        strip_prefix = "pybind11-2.6.2",
        urls = [
            "https://github.com/pybind/pybind11/archive/refs/tags/v2.6.2.tar.gz",
        ],
    )

    maybe(
        http_archive,
        name = "com_google_absl",
        sha256 = "59b862f50e710277f8ede96f083a5bb8d7c9595376146838b9580be90374ee1f",
        strip_prefix = "abseil-cpp-20210324.2",
        urls = [
            "https://github.com/abseil/abseil-cpp/archive/refs/tags/20210324.2.tar.gz",
        ],
    )

    maybe(
        http_archive,
        name = "com_github_gflags_gflags",
        sha256 = "34af2f15cf7367513b352bdcd2493ab14ce43692d2dcd9dfc499492966c64dcf",
        strip_prefix = "gflags-2.2.2",
        urls = ["https://github.com/gflags/gflags/archive/v2.2.2.tar.gz"],
    )

    maybe(
        http_archive,
        name = "com_github_google_glog",
        sha256 = "21bc744fb7f2fa701ee8db339ded7dce4f975d0d55837a97be7d46e8382dea5a",
        strip_prefix = "glog-0.5.0",
        urls = ["https://github.com/google/glog/archive/v0.5.0.zip"],
    )

    maybe(
        http_archive,
        name = "com_google_googletest",
        sha256 = "b4870bf121ff7795ba20d20bcdd8627b8e088f2d1dab299a031c1034eddc93d5",
        strip_prefix = "googletest-release-1.11.0",
        urls = [
            "https://github.com/google/googletest/archive/refs/tags/release-1.11.0.tar.gz",
        ],
    )

    maybe(
        http_archive,
        name = "glibc_version_header",
        sha256 = "57db74f933b7a9ea5c653498640431ce0e52aaef190d6bb586711ec4f8aa2b9e",
        strip_prefix = "glibc_version_header-0.1/version_headers/",
        urls = [
            "https://github.com/wheybags/glibc_version_header/archive/refs/tags/0.1.tar.gz",
        ],
        build_file = "//third_party/glibc_version_header:glibc_version_header.BUILD",
    )

    maybe(
        http_archive,
        name = "opencv",
        urls = [
            "https://github.com/opencv/opencv/archive/refs/tags/4.5.4.tar.gz",
        ],
        sha256 = "c20bb83dd790fc69df9f105477e24267706715a9d3c705ca1e7f613c7b3bad3d",
        strip_prefix = "opencv-4.5.4",
        build_file = "//third_party/opencv:opencv.BUILD",
    )

    maybe(
        http_archive,
        name = "eigen",
        urls = [
            "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz",
        ],
        sha256 = "8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72",
        strip_prefix = "eigen-3.4.0",
        build_file = "//third_party/eigen:eigen.BUILD",
    )
    maybe(
        http_archive,
        name = "com_google_benchmark",
        sha256 = "367e963b8620080aff8c831e24751852cffd1f74ea40f25d9cc1b667a9dd5e45",
        strip_prefix = "benchmark-1.6.1",
        urls = ["https://github.com/google/benchmark/archive/v1.6.1.zip"],
        # build_file = "//third_party/benchmark:benchmark.BUILD"
    )
    maybe(
        http_archive,
        name = "com_github_nelhage_rules_boost",
        sha256 = "b3cbdceaa95b8cfe3a69ff37f8ad0e53a77937433234f6b9a6add2eff5bde333",
        strip_prefix = "rules_boost-1e3a69bf2d5cd10c34b74f066054cd335d033d71",
        urls = [
            "https://github.com/nelhage/rules_boost/archive/1e3a69bf2d5cd10c34b74f066054cd335d033d71.tar.gz",
        ],
    )

    maybe(
        http_archive,
        name = "boost",
        build_file = "@com_github_nelhage_rules_boost//:BUILD.boost",
        patch_cmds = ["rm -f doc/pdf/BUILD"],
        sha256 = "d73a8da01e8bf8c7eda40b4c84915071a8c8a0df4a6734537ddde4a8580524ee",
        strip_prefix = "boost_1_71_0",
        urls = [
            "https://cdn.sail.sea.com/sail/bazel/boost_1_71_0.tar.bz2",
            "https://mirror.bazel.build/dl.bintray.com/boostorg/release/1.71.0/source/boost_1_71_0.tar.bz2",
            "https://dl.bintray.com/boostorg/release/1.71.0/source/boost_1_71_0.tar.bz2",
        ],
    )

    mypy_integration_version = "0.2.0"  # Latest @ 26th June 2021

    maybe(
        http_archive,
        name = "mypy_integration",
        sha256 = "621df076709dc72809add1f5fe187b213fee5f9b92e39eb33851ab13487bd67d",
        strip_prefix = "bazel-mypy-integration-{version}".format(version = mypy_integration_version),
        urls = [
            "https://github.com/thundergolfer/bazel-mypy-integration/archive/refs/tags/{version}.tar.gz".format(version = mypy_integration_version),
        ],
    )

workspace0 = workspace