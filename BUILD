# Copyright 2020 Aaron Webster
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

package(default_visibility = ["//visibility:public"])

licenses(["notice"])  # Apache 2.0

load("@com_google_emboss//:build_defs.bzl", "emboss_cc_library")

cc_library(
    name = "font5x7",
    hdrs = ["font5x7.h"],
    deps = [],
)

cc_library(
    name = "colormap",
    hdrs = ["colormap.h"],
    deps = [],
)

cc_binary(
    name = "flirone",
    srcs = ["flirone.cc"],
    copts = [
        "-I/usr/include/libusb-1.0",
    ],
    linkopts = [
        "-lusb-1.0",
        "-lm",
    ],
    deps = [
        ":colormap",
        ":font5x7",
        "@com_google_absl//absl/base",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
        "@com_google_absl//absl/strings",
    ],
)

# emboss_cc_library(
#     name = "packet",
#     srcs = ["packet.emb"],
# )
