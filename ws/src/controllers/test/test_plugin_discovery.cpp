// Plugin discovery smoke test (issue #6, M2.1).
//
// M2.1 exports the stage plugin surface and defines the plugin description schema
// but ships no plugin classes yet (those are M2.3, issue #8). This test pins the
// two things M2.1 *does* promise and that would otherwise only be asserted in the
// PR text:
//
//   1. The six plugin description files register the `controllers__pluginlib__plugin`
//      ament-index resource and are discoverable through it. This is the real proof
//      of acceptance criterion "Package exports plugin XML correctly for pluginlib
//      discovery" — it fails if a pluginlib_export_plugin_description_file() call is
//      dropped or the file fails to install.
//   2. A pluginlib::ClassLoader can be constructed for each active stage base class
//      from the exported header + XML, and declares the stock plugin IDs M2.3
//      (issue #8) added. contact_logic stays empty until M3.1 (#12). This flip
//      from "zero declared" to "the stock IDs" is the intended signal that M2.3
//      landed.
//
//      Point 2 asserted *exact* per-base class sets until M4.2 (#17). It cannot
//      any more, and the reason is the milestone succeeding rather than a test
//      being weakened: pluginlib resolves the `controllers__pluginlib__plugin`
//      resource across every installed package, so from #17 onwards any package
//      in the workspace may legitimately add a class to one of these bases
//      without `controllers` knowing (that is precisely M4's exit criterion).
//      An exact-set assertion here would therefore fail on a correct workspace,
//      and — worse — would pass or fail depending on whether the *other* package
//      happened to be built, which is environment, not correctness. What survives
//      is the assertion that actually belongs to this package: every stock ID is
//      present. Exactness is kept where it is still well defined — point 1 is
//      scoped to package `controllers`, so it still pins an exact file list, and
//      each out-of-package plugin asserts its own IDs in its own suite (see
//      examples/example_stage_plugins/test/).
//
// Runtime note: the ament resource lives in the install space, so the CMake target
// appends CMAKE_INSTALL_PREFIX to AMENT_PREFIX_PATH for this test (see
// CMakeLists.txt). Under `colcon test` the package is already installed, so the
// resource is present. See doc/modularity/plugin_discovery.md and contract_tests.md.

#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_resource.hpp"
#include "mit_controller/contact_logic_interface.hpp"
#include "mit_controller/gait_sequencer_interface.hpp"
#include "mit_controller/joint_commands.hpp"
#include "mit_controller/mpc_interface.hpp"
#include "mit_controller/stage_plugin.hpp"
#include "mit_controller/swing_leg_controller_interface.hpp"
#include "mit_controller/wbc_interface.hpp"
#include "model_adaptation/model_adaptation_interface.hpp"

namespace {

// The plugin description files registered in CMakeLists.txt, as the relative paths
// that pluginlib records in the resource content. Keep in sync with the
// pluginlib_export_plugin_description_file() calls.
const std::vector<std::string> kExpectedDescriptionFiles = {
    "share/controllers/plugins/gait_sequencer_plugins.xml",
    "share/controllers/plugins/mpc_plugins.xml",
    "share/controllers/plugins/slc_plugins.xml",
    "share/controllers/plugins/wbc_plugins.xml",
    "share/controllers/plugins/model_adaptation_plugins.xml",
    "share/controllers/plugins/contact_logic_plugins.xml",
};

std::vector<std::string> SplitNonEmptyLines(const std::string& content) {
  std::vector<std::string> lines;
  std::istringstream stream(content);
  std::string line;
  while (std::getline(stream, line)) {
    if (!line.empty()) {
      lines.push_back(line);
    }
  }
  return lines;
}

// The resource marker exists for package `controllers` and lists exactly the six
// description files. This is what proves the XML is exported for discovery.
TEST(PluginDiscovery, ResourceListsAllDescriptionFiles) {
  std::string content;
  std::string prefix;
  const bool found =
      ament_index_cpp::get_resource("controllers__pluginlib__plugin", "controllers", content, &prefix);
  ASSERT_TRUE(found) << "controllers__pluginlib__plugin resource not registered — is the package "
                        "installed and on AMENT_PREFIX_PATH?";

  const std::vector<std::string> listed = SplitNonEmptyLines(content);
  EXPECT_EQ(listed.size(), kExpectedDescriptionFiles.size());
  for (const auto& expected : kExpectedDescriptionFiles) {
    EXPECT_NE(std::find(listed.begin(), listed.end(), expected), listed.end())
        << "missing plugin description file in resource: " << expected;
  }
}

// A ClassLoader constructs for each active stage base from the exported header +
// XML and declares (at least) `expected_classes`. The constructor exercising the
// full chain — header compiles for a consumer, XML parses, resource resolves — is
// half the point; the declared-class set is the M2.3 signal.
//
// "At least" rather than "exactly" since M4.2 (#17) — see the header comment.
// This still fails on everything it is here to catch: a stock plugin dropped from
// its XML, a library that did not install, a base-class-type string that drifted
// out of sync between the XML and the header. The one thing it no longer objects
// to is another package adding a stage of its own, which is a feature.
template <class Base>
void ExpectDeclaresClasses(const std::string& base_class_type, const std::vector<std::string>& expected_classes) {
  std::unique_ptr<pluginlib::ClassLoader<Base>> loader;
  ASSERT_NO_THROW(
      loader = std::make_unique<pluginlib::ClassLoader<Base>>("controllers", base_class_type))
      << "ClassLoader failed to construct for base " << base_class_type;
  const std::vector<std::string> declared = loader->getDeclaredClasses();
  for (const auto& expected : expected_classes) {
    EXPECT_NE(std::find(declared.begin(), declared.end(), expected), declared.end())
        << "stock plugin '" << expected << "' is not declared for base " << base_class_type;
  }
}

TEST(PluginDiscovery, ClassLoaderDeclaresStockPluginsForEachStageBase) {
  ExpectDeclaresClasses<StagePlugin<GaitSequencerInterface>>(
      "StagePlugin<GaitSequencerInterface>", {"simple_gait", "adaptive_gait", "bio_gait"});
  ExpectDeclaresClasses<StagePlugin<MPCInterface>>("StagePlugin<MPCInterface>", {"acados_mpc"});
  ExpectDeclaresClasses<StagePlugin<SwingLegControllerInterface>>(
      "StagePlugin<SwingLegControllerInterface>", {"bezier_swing"});
  // Both WBCs under one base since #13 (M3.2) — the discovery-level statement of
  // "the command family is a runtime choice, not a build flavour".
  ExpectDeclaresClasses<StagePlugin<WBCInterface>>(
      "StagePlugin<WBCInterface>", {"wbc_arc_opt", "inverse_dynamics"});
  ExpectDeclaresClasses<StagePlugin<ModelAdaptationInterface>>(
      "StagePlugin<ModelAdaptationInterface>", {"kf_adaptation", "rls_adaptation"});
  ExpectDeclaresClasses<StagePlugin<ContactLogicInterface>>(
      "StagePlugin<ContactLogicInterface>", {"default_contact_logic"});
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
