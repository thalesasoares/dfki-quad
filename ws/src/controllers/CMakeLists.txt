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
# pluginlib::ClassLoader reads for discovery. M2.3 (#8) filled in the stock
# wrapper <class> entries (built by the stock stage plugin section below);
# contact_logic_plugins.xml stays empty until M3.1 (#12) adds the contact class,
# and #13 reworks only wbc_plugins.xml. See doc/modularity/plugin_discovery.md.
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

    # ##########################################################################
    # --- Stock stage plugin test (issue #8, M2.3) --- #
    # ##########################################################################
    # Drives the real stock wrappers through production ament discovery: each
    # stock plugin dlopens, constructs and Init's, and the fail-fast error paths
    # hold. Unlike test_stage_loader (synthetic plugin), this proves the shipped
    # libraries. The test executable links only common/interfaces/pluginlib — the
    # algorithm code (acados, ARC-OPT, fmt) lives in the plugin .so files that
    # pluginlib dlopens, so a successful load of libwbc_plugins is the runtime
    # check that the non-PIC-libfmt link story (stage_loading.md §6) resolves.
    # potato_model.cpp supplies the BrickModel/BrickState clones for StageInit.
    ament_add_gtest(test_stock_plugins
            test/test_stock_plugins.cpp
            src/potato_sim/potato_model.cpp
            APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX})
    target_include_directories(test_stock_plugins PRIVATE
            include include/mit_controller test)
    ament_target_dependencies(test_stock_plugins
            pluginlib common interfaces rclcpp Eigen3)
    target_compile_options(test_stock_plugins PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_dependencies(test_stock_plugins
            gait_sequencer_plugins mpc_plugins slc_plugins wbc_plugins model_adaptation_plugins)

    # ##########################################################################
    # --- Stage selection test (issue #9, M2.4) --- #
    # ##########################################################################
    # Pins the host's stage selection: the legacy `gait_sequencer` / `ma_mode` /
    # USE_WBC derivation of the `*.type` defaults (include/stage_selection.hpp),
    # and the fact that every default so derived names a plugin actually declared
    # for that stage base. The latter needs production ament discovery, hence the
    # same AMENT_PREFIX_PATH append and plugin-library dependency as
    # test_stock_plugins; it loads no stage, so no algorithm library is needed.
    ament_add_gtest(test_stage_selection
            test/test_stage_selection.cpp
            APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX})
    target_include_directories(test_stage_selection PRIVATE
            include include/mit_controller test)
    ament_target_dependencies(test_stage_selection
            pluginlib common interfaces rclcpp Eigen3)
    target_compile_options(test_stage_selection PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_dependencies(test_stage_selection
            gait_sequencer_plugins mpc_plugins slc_plugins wbc_plugins model_adaptation_plugins)
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
# The pipeline host (issue #9, M2.4): ROS I/O, loop timers, callback groups and
# plugin lifetime — and no algorithm code at all. Every algorithm source that used
# to be compiled in here now lives only in the stock plugin libraries above, which
# pluginlib dlopens at bring-up. That is what makes "the host cannot construct a
# concrete stage" a build-time fact rather than a convention: MPC, the gait
# sequencers, the swing leg controller, the WBCs and the model adaptation
# estimators are not linked into this executable any more.
#
# Consequently the solver/optimiser link line (ARC-OPT + its QP solvers, acados,
# OsqpEigen, fmt) and the -fpermissive source property for wbc_arc_opt.cpp moved
# down to the plugin section with the sources they belong to: the frozen stage
# *interfaces* the host talks to are plain virtual classes over Eigen types.
#
# bio_gait_sequencer.cpp is intentionally not here: it has no plugin wrapper until
# #16 (M4.1) and is compiled by the `mitcontroller` harness and — for the
# WITHOUT_DRAKE lane, which skips that harness — by gait_sequencer_plugins.
add_executable(mitcontrollernode
        src/mit_controller_node.cpp
)

# The stages are loaded, not linked, so the plugin libraries must exist before the
# node is worth running. Nothing links them; this only orders the build so that a
# plain `colcon build` never leaves a host without the stock plugins it defaults to.
add_dependencies(mitcontrollernode
        gait_sequencer_plugins
        mpc_plugins
        slc_plugins
        wbc_plugins
        model_adaptation_plugins)

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
ament_target_dependencies(mitcontrollernode Eigen3 common interfaces rclcpp pluginlib)
target_include_directories(
        mitcontrollernode
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


# ##############################################################################
# --- Stock stage plugins (issue #8, M2.3) --- #
# ##############################################################################
# One SHARED library per stage base, matching the per-stage description XML in
# plugins/. Each holds a thin adapter (src/plugins/<family>_plugins.cpp) plus the
# untouched algorithm sources it wraps; the wrapper's Init is the host factory
# body relocated behind the plugin boundary (plugin_lifecycle.md §4). The
# algorithm sources are compiled here as well as into mitcontrollernode until M2.4
# turns the host into a thin loader and drops them from the node.
#
# `common` and `interfaces` headers are needed but `common` is consumed as an
# INCLUDE PATH ONLY, never linked: linking it drags in its whole closure
# (quad_model_pino -> pinocchio -> drake -> a non-PIC static libfmt.a), which
# cannot go into a shared object — "relocation R_X86_64_PC32 ... can not be used
# when making a shared object" (stage_loading.md §6). The algorithm code only
# uses common's *abstract* ModelInterface/StateInterface and header-only helpers,
# so it never references a compiled `common` symbol. `interfaces` is a normal ROS
# message package (shared, PIC) and is linked normally for the GaitState message.
# The directory-level include_directories(include include/mit_controller src)
# above already puts this package's own headers and src/plugins on the path.
#
# The libraries inherit the directory-scope Release flags (-Ofast on x86 / -O3 on
# aarch64) so the control-loop code is optimised identically to the node — a
# lower-optimised plugin .so would be an invisible on-robot regression.
set(stock_plugin_common_includes
        $<TARGET_PROPERTY:common::quad_model_symbolic,INTERFACE_INCLUDE_DIRECTORIES>)

# Same warning set as every other target here, but with one demotion: a SHARED
# library forces -fPIC on its objects, and GCC 11 + -Ofast + -fPIC makes Eigen's
# TriangularMatrixVector emit a -Wmaybe-uninitialized FALSE POSITIVE (on `result`,
# deep in Eigen, not in our code) that the non-PIC node executable never hits.
# Demote only that one warning to non-fatal so it stays visible without failing
# the build; every other warning remains a hard error.
set(stock_plugin_compile_options -Wall -Wextra -Wpedantic -Werror -Wno-error=maybe-uninitialized)

# bio_gait_sequencer.cpp has no wrapper class yet — #16 (M4.1) adds one — but it is
# built here so that it still gets compiled at all: M2.4 dropped it from
# mitcontrollernode along with the other algorithms, and the only other target that
# compiled it (`mitcontroller`) is skipped in the WITHOUT_DRAKE / onboard lane. It
# shares every dependency of the two sequencers below, so this costs one TU and
# keeps the file from silently rotting until #16 wraps it.
add_library(gait_sequencer_plugins SHARED
        src/plugins/gait_sequencer_plugins.cpp
        src/mit_controller/simple_gait_sequencer.cpp
        src/mit_controller/adaptive_gait_sequencer.cpp
        src/mit_controller/bio_gait_sequencer.cpp
        src/mit_controller/gait.cpp
        src/mit_controller/mpc_trajectory_planner.cpp
        src/mit_controller/raibert_foot_step_planner.cpp)
target_include_directories(gait_sequencer_plugins PRIVATE ${stock_plugin_common_includes})
ament_target_dependencies(gait_sequencer_plugins pluginlib rclcpp Eigen3 interfaces)
target_compile_options(gait_sequencer_plugins PRIVATE ${stock_plugin_compile_options})

add_library(mpc_plugins SHARED
        src/plugins/mpc_plugins.cpp
        src/mit_controller/mpc.cpp)
target_include_directories(mpc_plugins PRIVATE ${stock_plugin_common_includes})
ament_target_dependencies(mpc_plugins pluginlib rclcpp Eigen3 interfaces)
target_link_libraries(mpc_plugins acados)
target_compile_options(mpc_plugins PRIVATE ${stock_plugin_compile_options})

add_library(slc_plugins SHARED
        src/plugins/slc_plugins.cpp
        src/mit_controller/swing_leg_controller.cpp
        src/SwingTrajectory.cpp)
target_include_directories(slc_plugins PRIVATE ${stock_plugin_common_includes})
ament_target_dependencies(slc_plugins pluginlib rclcpp Eigen3 interfaces)
target_compile_options(slc_plugins PRIVATE ${stock_plugin_compile_options})

# wbc_arc_opt.cpp needs -fpermissive: qpSWIFT's Auxilary.h declares members that
# shadow the type names `kkt` and `stats`, which is an error without it. The
# property is directory-scoped and used to sit next to mitcontrollernode's source
# list; M2.4 dropped that list, so it lives here, with the only target that still
# compiles the file.
set_source_files_properties(
        src/mit_controller/wbc_arc_opt.cpp
        PROPERTIES
        COMPILE_FLAGS -fpermissive
)

# wbc_arc_opt.cpp links fmt as header-only so the non-PIC libfmt.a stays off this
# shared object's link line (stage_loading.md §6). Both WBC classes live here
# because they are two instantiations of the same template interface
# (kWBC + kWBCCartesian), registered in the one libwbc_plugins.
add_library(wbc_plugins SHARED
        src/plugins/wbc_plugins.cpp
        src/mit_controller/wbc_arc_opt.cpp
        src/mit_controller/inverse_dynamics.cpp)
target_include_directories(wbc_plugins PRIVATE ${stock_plugin_common_includes})
ament_target_dependencies(wbc_plugins pluginlib rclcpp Eigen3 interfaces)
# acados is linked for its include path: ARC-OPT's hpipm solver header
# (wbc/solvers/hpipm/HPIPMSolver.hpp) includes acados_c/dense_qp_interface.h, the
# same reason mitcontrollernode links acados.
target_link_libraries(wbc_plugins
        acados
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
        fmt::fmt-header-only)
target_compile_options(wbc_plugins PRIVATE ${stock_plugin_compile_options})

add_library(model_adaptation_plugins SHARED
        src/plugins/model_adaptation_plugins.cpp
        src/model_adaptation/kf_model_adaptation.cpp
        src/model_adaptation/least_squares_model_adaptation.cpp)
target_include_directories(model_adaptation_plugins PRIVATE ${stock_plugin_common_includes})
ament_target_dependencies(model_adaptation_plugins pluginlib rclcpp Eigen3 interfaces)
target_compile_options(model_adaptation_plugins PRIVATE ${stock_plugin_compile_options})

# pluginlib resolves a plugin library relative to the install prefix of the
# package that owns the description XML (this package), so the .so must land in
# the same lib/ the exported XML points at.
install(TARGETS
        gait_sequencer_plugins
        mpc_plugins
        slc_plugins
        wbc_plugins
        model_adaptation_plugins
        DESTINATION lib)


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
