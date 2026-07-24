cmake_minimum_required(VERSION 3.8)
project(controllers)

option(ON_PI "On Raspberry Pi" FALSE)
if (ON_PI)
    message(WARNING "Skipping package on Raspberry Pi.")
    return()
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)
IF (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
else ()
    set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")
    message(WARNING "detected non ARM architecture, applying -Ofast - this might be harmfull!")
endif ()


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP) # for speeding up eigens matrix multiplications
if (NOT WITHOUT_DRAKE)
    find_package(drake CONFIG REQUIRED)
    find_package(simulator REQUIRED)
endif ()
find_package(acados REQUIRED)
find_package(Matplot++ REQUIRED)
find_package(unitree_go REQUIRED)
find_package(fmt REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OsqpEigen REQUIRED)


# ARCOPT
pkg_search_module(wbc-core REQUIRED IMPORTED_TARGET wbc-core)
pkg_search_module(wbc-robot_models-pinocchio REQUIRED IMPORTED_TARGET wbc-robot_models-pinocchio)
pkg_search_module(wbc-solvers-eiquadprog REQUIRED IMPORTED_TARGET wbc-solvers-eiquadprog)
pkg_search_module(wbc-solvers-qpswift REQUIRED IMPORTED_TARGET wbc-solvers-qpswift)
pkg_search_module(wbc-solvers-proxqp REQUIRED IMPORTED_TARGET wbc-solvers-proxqp)
pkg_search_module(wbc-solvers-qpoases REQUIRED IMPORTED_TARGET wbc-solvers-qpoases)
pkg_search_module(wbc-scenes-acceleration_reduced_tsid REQUIRED IMPORTED_TARGET wbc-scenes-acceleration_reduced_tsid)
pkg_search_module(wbc-scenes-acceleration_tsid REQUIRED IMPORTED_TARGET wbc-scenes-acceleration_tsid)
pkg_search_module(wbc-controllers REQUIRED IMPORTED_TARGET wbc-controllers)
pkg_search_module(wbc-solvers-osqp REQUIRED IMPORTED_TARGET wbc-solvers-osqp)
pkg_search_module(wbc-types REQUIRED IMPORTED_TARGET wbc-types)
pkg_search_module(wbc-solvers-hpipm REQUIRED IMPORTED_TARGET wbc-solvers-hpipm)
# pkg_search_module(base-types REQUIRED IMPORTED_TARGET base-types)


include_directories(include include/mit_controller src)

# ##############################################################################
# --- Stage plugin description schema (issue #6, M2.1) --- #
# ##############################################################################
# One plugin description file per stage base class. They install the XML and
# register the `controllers__pluginlib__plugin` ament-index resource that
# pluginlib::ClassLoader reads for discovery. The files are schema-only in M2.1
# (no <class> entries yet); M2.3 (#8) fills the wrappers in, M3.1 (#12) adds the
# contact class, and #13 reworks only wbc_plugins.xml. See
# doc/modularity/plugin_discovery.md.
pluginlib_export_plugin_description_file(controllers plugins/gait_sequencer_plugins.xml)
pluginlib_export_plugin_description_file(controllers plugins/mpc_plugins.xml)
pluginlib_export_plugin_description_file(controllers plugins/slc_plugins.xml)
pluginlib_export_plugin_description_file(controllers plugins/wbc_plugins.xml)
pluginlib_export_plugin_description_file(controllers plugins/model_adaptation_plugins.xml)
pluginlib_export_plugin_description_file(controllers plugins/contact_logic_plugins.xml)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights comment the
    # line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo) comment the line
    # when this package is in a git repo and when a copyright and license is added
    # to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    # ##########################################################################
    # --- Stage interface contract test (issue #5, M1.5) --- #
    # ##########################################################################
    # Fake stubs (test/fake_stages.hpp) implement all six stage interfaces plus
    # the gait seam; test/test_stage_contracts.cpp static_asserts the contract
    # shape (§4 of doc/modularity/stage_contracts.md) and drives each fake once
    # through a unique_ptr<Interface>, the way the host owns its stages. Because
    # the API-break detection is compile-time, the ordinary `colcon build`
    # (CI's `cbg`) is already the compile smoke — `colcon test` only adds the
    # runtime pass. Deps are common/interfaces/rclcpp/Eigen3 only: no drake,
    # acados, ARC-OPT or hardware, so it builds on both CI lanes.
    #
    # This supersedes src/tools/contact_logic_interface_check.cpp (M1.4), whose
    # N_LEGS sizing asserts are migrated into the test. The M1.3 surface check
    # (src/tools/pipeline_types_surface_check.cpp) stays: it guards a different
    # axis (export-only include path). See doc/modularity/contract_tests.md.
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_stage_contracts
            test/test_stage_contracts.cpp
            src/potato_sim/potato_model.cpp)
    target_include_directories(test_stage_contracts PRIVATE
            include include/mit_controller test)
    ament_target_dependencies(test_stage_contracts common interfaces rclcpp Eigen3)
    target_compile_options(test_stage_contracts PRIVATE -Wall -Wextra -Wpedantic -Werror)

    # ##########################################################################
    # --- Plugin discovery smoke test (issue #6, M2.1) --- #
    # ##########################################################################
    # Verifies the six plugin description files are exported for pluginlib
    # discovery (the controllers__pluginlib__plugin ament resource) and that a
    # ClassLoader constructs for each stage base from the exported header + XML,
    # reporting zero declared classes (M2.1 is schema-only; M2.3 flips the count).
    #
    # The ament resource is in the install space, so AMENT_PREFIX_PATH is extended
    # with CMAKE_INSTALL_PREFIX for the test process. Under `colcon test` the
    # package is already installed, so the resource is present. See
    # doc/modularity/plugin_discovery.md.
    find_package(ament_index_cpp REQUIRED)
    ament_add_gtest(test_plugin_discovery
            test/test_plugin_discovery.cpp
            APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX})
    target_include_directories(test_plugin_discovery PRIVATE
            include include/mit_controller)
    ament_target_dependencies(test_plugin_discovery
            pluginlib ament_index_cpp common interfaces rclcpp Eigen3)
    target_compile_options(test_plugin_discovery PRIVATE -Wall -Wextra -Wpedantic -Werror)

    # ##########################################################################
    # --- Stage loader test (issue #7, M2.2) --- #
    # ##########################################################################
    # test/test_stage_loader.cpp drives StageLoader through the real pluginlib
    # chain — description XML, class lookup, dlopen, default construction, Init —
    # and pins the fail-fast behaviour that is the issue's actual acceptance
    # criterion: a missing or invalid `type:` throws with the offending key/value
    # named, and never substitutes another stage.
    #
    # That needs a loadable plugin, and M2.2 ships none in production (the stock
    # wrappers are M2.3, issue #8). Hence this test-only library. Two rules keep
    # it from leaking into the product:
    #
    #  1. test/stage_loader_test_plugins.xml is NOT passed to
    #     pluginlib_export_plugin_description_file(), so it never joins the
    #     `controllers__pluginlib__plugin` resource and test_plugin_discovery
    #     keeps (correctly) seeing zero declared classes for every stage base.
    #     The test hands the path to StageLoader's explicit plugin_xml_paths
    #     argument instead.
    #  2. Both the library and the test exist only under BUILD_TESTING.
    #
    # The library *is* installed, into the same lib/ the package's other targets
    # use: pluginlib resolves a plugin library only relative to the install prefix
    # of the package that owns the description XML (getAllLibraryPathsToTry —
    # absolute paths in `path=` are not honoured), and the XML lives in this
    # package's source tree, so `controllers`' prefix is where it will look.
    #
    # `common` and `interfaces` are pulled in as include paths only, not through
    # ament_target_dependencies. This plugin needs just their *headers* — the
    # abstract ModelInterface/StateInterface and the default-constructible
    # GaitState message — and linking `common` would drag in its whole library
    # closure (quad_model_pino → pinocchio → drake → a non-PIC static libfmt.a),
    # which cannot go into a shared object at all: "relocation R_X86_64_PC32 …
    # can not be used when making a shared object". Worth knowing for M2.3
    # (issue #8), whose stock plugin libraries link acados and fmt for real — see
    # doc/modularity/stage_loading.md §6.
    add_library(stage_loader_test_plugins SHARED test/stage_loader_test_plugins.cpp)
    # `common` exports no ament include directories (it carries them on its
    # imported targets instead — the transitive-dependency wart noted in
    # doc/modularity/pipeline_types.md §7), so its include path is read off the
    # target rather than linked.
    target_include_directories(stage_loader_test_plugins PRIVATE
            include include/mit_controller
            $<TARGET_PROPERTY:common::quad_model_symbolic,INTERFACE_INCLUDE_DIRECTORIES>
            ${interfaces_INCLUDE_DIRS})
    ament_target_dependencies(stage_loader_test_plugins pluginlib rclcpp Eigen3)
    target_compile_options(stage_loader_test_plugins PRIVATE -Wall -Wextra -Wpedantic -Werror)
    install(TARGETS stage_loader_test_plugins DESTINATION lib)

    ament_add_gtest(test_stage_loader
            test/test_stage_loader.cpp
            src/potato_sim/potato_model.cpp
            APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX})
    target_include_directories(test_stage_loader PRIVATE
            include include/mit_controller test)
    ament_target_dependencies(test_stage_loader
            pluginlib common interfaces rclcpp Eigen3)
    # The test-only description file, by absolute source path: it is deliberately
    # not exported, so there is no ament resource to find it through.
    target_compile_definitions(test_stage_loader PRIVATE
            STAGE_LOADER_TEST_PLUGIN_XML="${CMAKE_CURRENT_SOURCE_DIR}/test/stage_loader_test_plugins.xml")
    target_compile_options(test_stage_loader PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_dependencies(test_stage_loader stage_loader_test_plugins)
endif ()


# ##############################################################################
# --- Trajectory Controller --- #
# ##############################################################################

if (NOT WITHOUT_DRAKE)
    add_executable(trajectory_replay src/trajectory_replay.cpp)

    ament_target_dependencies(trajectory_replay interfaces rclcpp
            sensor_msgs geometry_msgs common)

    target_include_directories(
            trajectory_replay
            PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
    install(TARGETS trajectory_replay DESTINATION lib/${PROJECT_NAME})

endif ()
# ##############################################################################
# --- Gait Plotting --- #
# ##############################################################################

add_executable(gait_plotting src/tools/plotting.cpp)

ament_target_dependencies(gait_plotting Eigen3 interfaces rclcpp common
)
target_link_libraries(gait_plotting matplot)

target_include_directories(
        gait_plotting
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# ##############################################################################
# --- Gait Sequence Plotting --- #
# ##############################################################################

add_executable(gait_sequence_plotting src/tools/plot_gs.cpp)

ament_target_dependencies(gait_sequence_plotting Eigen3 interfaces rclcpp common
)
target_link_libraries(gait_sequence_plotting matplot)

target_include_directories(
        gait_sequence_plotting
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# ##############################################################################
# --- MIT CONTROLLER--- #
# ##############################################################################
if (NOT WITHOUT_DRAKE)
    add_executable(mitcontroller
            src/mit_controller/mit_controller.cpp
            src/mit_controller/mpc.cpp
            src/mit_controller/gait.cpp
            src/mit_controller/simple_gait_sequencer.cpp
            src/mit_controller/bio_gait_sequencer.cpp
            src/mit_controller/adaptive_gait_sequencer.cpp
            src/mit_controller/mpc_trajectory_planner.cpp
            src/mit_controller/raibert_foot_step_planner.cpp
            src/mit_controller/swing_leg_controller.cpp
            src/potato_sim/potato_sim.cpp
            src/potato_sim/potato_model.cpp
            src/SwingTrajectory.cpp
            src/model_adaptation/least_squares_model_adaptation.cpp
            src/model_adaptation/kf_model_adaptation.cpp
    )

    ament_target_dependencies(mitcontroller Eigen3 interfaces common drake simulator)
    target_link_libraries(mitcontroller acados matplot)
    target_compile_options(mitcontroller PRIVATE -Wall -Wextra -Wpedantic -Werror)
    # `mitcontroller` is an *executable*, so its generated export file declares
    # `add_executable(controllers::mitcontroller IMPORTED)`. Exporting it put that
    # imported executable into `controllers_TARGETS`, so every downstream
    # `ament_target_dependencies(<tgt> controllers)` tried to link an executable
    # and failed at CMake generate time with "links to target
    # controllers::mitcontroller but the target was not found" — i.e. this package
    # could not be consumed by any other package at all. The export is dropped
    # here so that the shared pipeline type surface is actually usable (issue #3,
    # M1.3); the binary itself is still installed to `bin` exactly as before.
    install(
            TARGETS mitcontroller
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            RUNTIME DESTINATION bin
            INCLUDES
            DESTINATION include)
endif ()


# ##############################################################################
# --- MIT CONTROLLER NODE --- #
# ##############################################################################
add_executable(mitcontrollernode
        src/mit_controller_node.cpp
        src/mit_controller/mpc.cpp
        src/mit_controller/gait.cpp
        src/mit_controller/simple_gait_sequencer.cpp
        src/mit_controller/adaptive_gait_sequencer.cpp
        src/mit_controller/bio_gait_sequencer.cpp
        src/mit_controller/mpc_trajectory_planner.cpp
        src/mit_controller/raibert_foot_step_planner.cpp
        src/mit_controller/swing_leg_controller.cpp
        src/mit_controller/inverse_dynamics.cpp
        src/mit_controller/wbc_arc_opt.cpp
        src/model_adaptation/least_squares_model_adaptation.cpp
        src/model_adaptation/kf_model_adaptation.cpp
        src/SwingTrajectory.cpp
)

set_source_files_properties(
        src/mit_controller/wbc_arc_opt.cpp
        PROPERTIES
        COMPILE_FLAGS -fpermissive
)

target_link_libraries(mitcontrollernode
        PkgConfig::wbc-core
        PkgConfig::wbc-robot_models-pinocchio
        PkgConfig::wbc-solvers-eiquadprog
        PkgConfig::wbc-solvers-qpswift
        PkgConfig::wbc-solvers-proxqp
        PkgConfig::wbc-solvers-qpoases
        PkgConfig::wbc-solvers-osqp
        PkgConfig::wbc-scenes-acceleration_reduced_tsid
        PkgConfig::wbc-scenes-acceleration_tsid
        PkgConfig::wbc-controllers
        PkgConfig::wbc-types
        OsqpEigen::OsqpEigen
        PkgConfig::wbc-solvers-hpipm
)

if (ROBOT_NAME STREQUAL "ulab")
    target_compile_definitions(mitcontrollernode PUBLIC ROBOT_MODEL=ULAB)
    MESSAGE(WARNING "Building for ulab quad")
elseif (ROBOT_NAME STREQUAL "go2")
    target_compile_definitions(mitcontrollernode PUBLIC ROBOT_MODEL=GO2)
    MESSAGE(WARNING "Building for go2 quad")
else ()
    MESSAGE(FATAL_ERROR "Please specify robot model during build")
endif ()

target_compile_options(mitcontrollernode PRIVATE -Wall -Wextra -Wpedantic -Werror)
ament_target_dependencies(mitcontrollernode Eigen3 common interfaces rclcpp)
target_link_libraries(mitcontrollernode acados fmt::fmt)
target_include_directories(
        mitcontrollernode
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


# ##############################################################################
# --- test Swing leg Controller --- #
# ##############################################################################

add_executable(testSwingTrajectoryControllerNode src/testSwingTrajectoryControllerNode.cpp
        src/SwingTrajectory.cpp)

ament_target_dependencies(testSwingTrajectoryControllerNode Eigen3 interfaces rclcpp
        sensor_msgs geometry_msgs common)

target_include_directories(
        testSwingTrajectoryControllerNode
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# ##############################################################################
# --- Sys ID Leg Controller --- #
# ##############################################################################

add_executable(SysIDLegControllerNode src/system_identification/SysIDLegControllerNode.cpp src/system_identification/SysIDTrajectory.cpp)

ament_target_dependencies(SysIDLegControllerNode Eigen3 interfaces rclcpp common)

target_include_directories(
        SysIDLegControllerNode
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


# ##############################################################################
# --- Shared pipeline type surface compile check --- #
# ##############################################################################
# Builds the exported pipeline type + stage interface headers with a deliberately
# restricted include path — `include/` plus the declared dependencies only, i.e.
# what an out-of-package stage plugin sees. This fails the build if a surface
# header stops being self-contained or starts depending on node internals.
# See doc/modularity/pipeline_types.md (issue #3, M1.3) and plugin_lifecycle.md
# (issue #6, M2.1). rclcpp / interfaces joined the dependency list when the stage
# interface headers (which include rclcpp/parameter_value.hpp, rclcpp/time.hpp and
# interfaces/msg/gait_state.hpp) joined the surface.

add_library(pipeline_types_surface_check OBJECT src/tools/pipeline_types_surface_check.cpp)

# Drop the directory-level include_directories() above: no include/mit_controller,
# no src/. Must be set before ament_target_dependencies, which appends to this
# same property.
set_property(TARGET pipeline_types_surface_check
        PROPERTY INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include")

# pluginlib joins the list in M2.2 (issue #7): stage_loader.hpp is on the surface
# and includes pluginlib/class_loader.hpp. Listing it here is also the check that
# `ament_export_dependencies(... pluginlib)` is what an out-of-package plugin
# actually needs — this target consumes the headers the way that consumer does.
ament_target_dependencies(pipeline_types_surface_check common Eigen3 rclcpp interfaces pluginlib)
target_compile_options(pipeline_types_surface_check PRIVATE -Wall -Wextra -Wpedantic -Werror)

# The M1.4 ContactLogicInterface compile check
# (src/tools/contact_logic_interface_check.cpp) lived here. It is superseded by
# the M1.5 stage contract test (test/test_stage_contracts.cpp, in the
# BUILD_TESTING block above), which includes the same header through a fake stub
# and migrates its N_LEGS sizing asserts. See doc/modularity/contract_tests.md.

# ##############################################################################
# --- CPU power consumption logging--- #
# ##############################################################################
add_executable(log_cpu_power src/tools/log_cpu_power.cpp)

ament_target_dependencies(log_cpu_power
        rclcpp
        std_msgs
)

target_include_directories(
        log_cpu_power
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


# ##############################################################################
# Install nodes #
# ##############################################################################

##############################
# Python package and scripts #
##############################

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/joy_to_target.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/quad_stand_up.py DESTINATION lib/${PROJECT_NAME})

#########
# Share #
#########

if (NOT WITHOUT_DRAKE)
    install(TARGETS trajectory_replay gait_plotting gait_sequence_plotting log_cpu_power testSwingTrajectoryControllerNode SysIDLegControllerNode mitcontrollernode DESTINATION lib/${PROJECT_NAME})
else ()
    install(TARGETS gait_plotting gait_sequence_plotting log_cpu_power testSwingTrajectoryControllerNode SysIDLegControllerNode mitcontrollernode DESTINATION lib/${PROJECT_NAME})
endif ()

install(DIRECTORY launch config trajectories DESTINATION share/${PROJECT_NAME}/)

###################################
# Shared pipeline type surface    #
###################################
# The in-process data types the control pipeline stages exchange (issue #3,
# M1.3). This is an explicit allowlist, not `install(DIRECTORY include/)`: the
# rest of `include/` is node and algorithm internals and must stay private so
# that "stable surface" keeps meaning something. The stage interface headers were
# added to the surface in M2.1 (issue #6) — see the block just below.
#
# Keep in sync with src/tools/pipeline_types_surface_check.cpp and
# doc/modularity/pipeline_types.md.
install(FILES
        include/mit_controller/target.hpp
        include/mit_controller/gait_sequence.hpp
        include/mit_controller/wrench_sequence.hpp
        include/mit_controller/feet_targets.hpp
        include/mit_controller/mpc_prediction.hpp
        include/mit_controller/pipeline_constants.hpp
        DESTINATION include/mit_controller)

###################################
# Stage interface + plugin surface (issue #6, M2.1)
###################################
# The frozen stage APIs (stage_contracts.md) plus the pluginlib lifecycle base
# (plugin_lifecycle.md). These join the exported surface now that the pluginlib
# base has landed — pipeline_types.md §6 planned this for M2.1. Still an explicit
# allowlist: everything not listed here stays node/algorithm-internal.
#
# gait_sequencer_types.hpp travels with the GS interface (it includes it);
# joint_commands.hpp travels with the WBC interface (SolverInformation / WBCReturn
# live inside the interface headers themselves). stage_plugin.hpp is already
# self-contained (plugin_lifecycle.md §7), and so is stage_loader.hpp (issue #7,
# M2.2) — it includes only stage_plugin.hpp and pluginlib, never a stage
# interface, so a consumer pays for exactly the bases it instantiates.
#
# Keep in sync with src/tools/pipeline_types_surface_check.cpp and
# doc/modularity/{pipeline_types,plugin_lifecycle,stage_loading}.md.
install(FILES
        include/mit_controller/gait_sequencer_interface.hpp
        include/mit_controller/gait_sequencer_types.hpp
        include/mit_controller/mpc_interface.hpp
        include/mit_controller/swing_leg_controller_interface.hpp
        include/mit_controller/wbc_interface.hpp
        include/mit_controller/joint_commands.hpp
        include/mit_controller/contact_logic_interface.hpp
        include/mit_controller/stage_plugin.hpp
        include/mit_controller/stage_loader.hpp
        DESTINATION include/mit_controller)
install(FILES
        include/model_adaptation/model_adaptation_interface.hpp
        DESTINATION include/model_adaptation)

ament_export_include_directories(include)
# The exported stage interface surface (below) includes rclcpp/parameter_value.hpp,
# rclcpp/time.hpp and interfaces/msg/gait_state.hpp, and StagePlugin<Interface> is a
# pluginlib base. Out-of-package stage plugins (the M4 goal) must pull these in
# transitively rather than re-`find_package`-ing them by hand — the same
# consumer-boilerplate trap that `common` fell into (pipeline_types.md §7, item 1).
ament_export_dependencies(common Eigen3 rclcpp interfaces pluginlib)

ament_package()
