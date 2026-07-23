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
//      from the exported header + XML, and (correctly, for M2.1) reports zero
//      declared classes. In M2.3 these same assertions flip to expecting the stock
//      IDs, so the test grows with the milestone instead of being rewritten.
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
// XML, and reports zero declared classes (M2.1 is schema-only). The constructor
// exercising the full chain — header compiles for a consumer, XML parses, resource
// resolves — is the point; M2.3 changes the expected count.
template <class Base>
void ExpectLoadableWithNoClassesYet(const std::string& base_class_type) {
  std::unique_ptr<pluginlib::ClassLoader<Base>> loader;
  ASSERT_NO_THROW(
      loader = std::make_unique<pluginlib::ClassLoader<Base>>("controllers", base_class_type))
      << "ClassLoader failed to construct for base " << base_class_type;
  EXPECT_TRUE(loader->getDeclaredClasses().empty())
      << "expected no stock plugins in M2.1 for base " << base_class_type;
}

TEST(PluginDiscovery, ClassLoaderConstructsForEachStageBase) {
  ExpectLoadableWithNoClassesYet<StagePlugin<GaitSequencerInterface>>(
      "StagePlugin<GaitSequencerInterface>");
  ExpectLoadableWithNoClassesYet<StagePlugin<MPCInterface>>("StagePlugin<MPCInterface>");
  ExpectLoadableWithNoClassesYet<StagePlugin<SwingLegControllerInterface>>(
      "StagePlugin<SwingLegControllerInterface>");
  ExpectLoadableWithNoClassesYet<StagePlugin<WBCInterface<JointTorqueVelocityPositionCommands>>>(
      "StagePlugin<WBCInterface<JointTorqueVelocityPositionCommands>>");
  ExpectLoadableWithNoClassesYet<StagePlugin<ModelAdaptationInterface>>(
      "StagePlugin<ModelAdaptationInterface>");
  ExpectLoadableWithNoClassesYet<StagePlugin<ContactLogicInterface>>(
      "StagePlugin<ContactLogicInterface>");
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
