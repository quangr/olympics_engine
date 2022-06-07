workspace(name = "olympics_engine")

load("workspace0.bzl", workspace0 = "workspace")

workspace0()

load("workspace1.bzl", workspace1 = "workspace")

workspace1()

load("pip.bzl", pip_workspace = "workspace")

pip_workspace()

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")


# Hedron's Compile Commands Extractor for Bazel
# https://github.com/hedronvision/bazel-compile-commands-extractor
http_archive(
    name = "hedron_compile_commands",

    # Replace the commit hash in both places (below) with the latest, rather than using the stale one here.
    # Even better, set up Renovate and let it do the work for you (see "Suggestion: Updates" in the README).
    url = "https://github.com/hedronvision/bazel-compile-commands-extractor/archive/2e8b7654fa10c44b9937453fa4974ed2229d5366.tar.gz",
    strip_prefix = "bazel-compile-commands-extractor-2e8b7654fa10c44b9937453fa4974ed2229d5366",
    # When you first run this tool, it'll recommend a sha256 hash to put here with a message like: "DEBUG: Rule 'hedron_compile_commands' indicated that a canonical reproducible form can be obtained by modifying arguments sha256 = ..."
)
load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")
hedron_compile_commands_setup()