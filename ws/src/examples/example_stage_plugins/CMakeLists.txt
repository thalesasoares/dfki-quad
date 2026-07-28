cmake_minimum_required(VERSION 3.8)
project(example_stage_plugins)

# ##############################################################################
# Example out-of-package stage plugins (issue #17, M4.2).
#
# This file is the reference a third party copies to add a control stage without
# touching the host, so it is written to be read: everything here is either
# ordinary ament boilerplate or a rule that doc/modularity spells out, and the
# non-obvious lines carry the reason inline.
# ##############################################################################

# Mirror the `controllers` build type and per-architecture Release flags exactly
# (controllers/CMakeLists.txt:14-21). This is a hard rule for any package that
# ships a stage plugin, not a style preference: a stage runs inside the 100 Hz /
# 500 Hz control loops, so a plugin .so built at a lower optimisation level than
# the node is an invisible on-robot regression that no test would catch
# (doc/modularity/stock_plugins.md §4, "Same optimisation as the node").
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
else ()
    set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")
endif ()

find_package(ament_cmake REQUIRED)
# `controllers` owns the stage interfaces, the StagePlugin base and the loader,
# and re-exports common / Eigen3 / rclcpp / interfaces / pluginlib
# (doc/modularity/plugin_discovery.md §1), so this is the only pipeline package a
# stage plugin has to name. pluginlib is found explicitly because this file calls
# its macros by name.
find_package(controllers REQUIRED)
find_package(pluginlib REQUIRED)

# ##############################################################################
# --- The plugin library --- #
# ##############################################################################
#
# SHARED because pluginlib dlopens it. Note what is *absent*: no algorithm
# sources and no private header of `controllers`. The plugin compiles against the
# installed export surface only, which is the property that makes it a genuine
# third-party example rather than an in-tree one.
add_library(example_slc_plugins SHARED src/passthrough_slc_plugin.cpp)

# CONSUME `controllers` AS AN INCLUDE PATH, NEVER AS A LINK DEPENDENCY.
#
# This is the single least obvious rule in this file, and getting it wrong is a
# hard build failure rather than a subtle one, so it is worth stating plainly:
#
#   ament_target_dependencies(example_slc_plugins controllers ...)
#
# does NOT work. `ament_target_dependencies` adds ${controllers_LIBRARIES} to the
# link line, and that variable carries the transitive closure `controllers`
# re-exports — common -> quad_model -> drake::drake — none of which exists in a
# consumer that has not found drake, and none of which a stage plugin needs. The
# failure is `links to target "drake::drake" but the target was not found`.
#
# The reason it is unnecessary is the important half: a stage plugin references
# no compiled symbol of `controllers` or `common`. Everything it touches is a
# pure-virtual interface (ModelInterface, StateInterface,
# SwingLegControllerInterface), a header-only helper (StageInit, StageInitError)
# or a plain data struct (FeetTargets, GaitSequence). The *host* provides the
# implementations at runtime through the vtable. So headers are all that is
# needed, and taking only headers is what keeps this .so free of the non-PIC
# static-libfmt closure that `controllers` documents at
# doc/modularity/stage_loading.md §6 and stock_plugins.md §4.
#
# ${controllers_INCLUDE_DIRS} already carries `common`'s and `interfaces`'
# include directories, because `controllers` re-exports them
# (ament_export_dependencies, doc/modularity/plugin_discovery.md §1) — that
# re-export is exactly what spares a third party from finding them by hand.
target_include_directories(example_slc_plugins SYSTEM PRIVATE ${controllers_INCLUDE_DIRS})

# The real link dependencies: pluginlib (the export macro and its registry) and
# rclcpp (parameter values and logging). Both are PIC shared libraries, so both
# link cleanly into a plugin .so.
ament_target_dependencies(example_slc_plugins pluginlib rclcpp)
# Eigen is named through its imported target rather than via
# ament_target_dependencies: `controllers` re-exports Eigen3 as a *package*, but
# the exported ${controllers_INCLUDE_DIRS} does not carry Eigen's include
# directory (Eigen ships an INTERFACE target, not ament include vars), so the
# stage interface headers' `#include <Eigen/Geometry>` would not resolve. The
# target is header-only, so this adds an include path and no link-time closure.
target_link_libraries(example_slc_plugins Eigen3::Eigen)

# The same warning set every target in `controllers` uses, including the one
# demotion: a SHARED library forces -fPIC, and GCC + -Ofast + -fPIC makes Eigen's
# TriangularMatrixVector emit a -Wmaybe-uninitialized FALSE POSITIVE that a
# non-PIC executable never hits. Demoted so it stays visible without failing the
# build; every other warning remains a hard error (controllers/CMakeLists.txt:478).
target_compile_options(example_slc_plugins PRIVATE
        -Wall -Wextra -Wpedantic -Werror -Wno-error=maybe-uninitialized)

# pluginlib resolves `<library path="...">` relative to the install prefix of the
# package that owns the description XML — this package — so the .so must land in
# this package's lib/, next to the exported XML's prefix.
install(TARGETS example_slc_plugins DESTINATION lib)

# THE ONE LINE THAT MAKES IT DISCOVERABLE.
#
# The first argument is `controllers`, not this project: it names the ament
# resource (`controllers__pluginlib__plugin`) that StageLoader searches, because
# `controllers` owns the base classes. pluginlib resolves that resource across
# *every* installed package, so registering here is enough — no allowlist, no
# host edit, nothing in `controllers` to touch
# (mit_controller/stage_loader.hpp, kStagePluginPackage).
pluginlib_export_plugin_description_file(controllers plugins/example_slc_plugins.xml)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
