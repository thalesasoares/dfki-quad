// Shipped stage overlay tests (issue #19, M4.4).
//
// M4.4 adds `stage_overlay:=<file>` to mit_controller.launch.py and ships two
// overlays in `config/overlays/`. An overlay is a params file layered over the
// robot config, so — exactly like the Go2 configs pinned by
// test_go2_yaml_stage_selection.cpp — everything it promises is a property of a
// *file*, and a broken one fails at robot bring-up rather than at build time.
// This suite is where that failure is moved to `colcon test`:
//
//   1. The directory reaches the install space at all. `config/overlays/` rides
//      on the existing `install(DIRECTORY config ...)` rule, so nothing in
//      CMakeLists names it; an empty result here is how a change to that rule
//      surfaces, instead of as `stage_overlay:=go2_bio_gait.yaml` aborting the
//      launch with "is not a file".
//   2. Every shipped overlay is well-formed: it parses, it addresses
//      `mit_controller_node`, and it selects at least one stage with a non-empty
//      string. The last part is the one that earns its keep — `gs.typ: bio_gait`
//      is valid YAML that sets a parameter nothing reads, and an overlay whose
//      whole purpose is one key silently doing nothing is precisely the failure
//      the fail-fast loader cannot catch (the loader only sees the key that *is*
//      set, which is still the stock default).
//   3. Every shipped overlay is smaller than the config it overlays. This is
//      issue #19's acceptance criterion — "can swap one stage without rewriting
//      full Go2 YAML" — stated as something a test can hold: an overlay that
//      grew into a copy of the stock file would still work, and would have lost
//      the point.
//   4. Each overlay selects the plugin it advertises, and that plugin is
//      declared for that stage's base. Same check as the Go2 YAML suite, and it
//      is what makes a stock plugin rename surface here rather than in a demo.
//
// (4) is unconditional for `go2_bio_gait.yaml` (a stock class, always built with
// this package) but *conditional* for `go2_example_passthrough_slc.yaml`, whose
// class ships from `example_stage_plugins`. The reasoning is the one
// test_plugin_discovery.cpp already records for its containment assertions:
// whether another package happens to be built is environment, not correctness,
// so requiring it here would fail a correct workspace. What can still be
// asserted precisely is the implication — if that package is installed, it must
// declare the class this overlay names — and that is what runs. When it is
// absent the test says so and skips, rather than passing quietly.
//
// Like the sibling suites this reads from the install prefix (the copy the
// launch file actually passes to the node) and never loads or initialises a
// stage.

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "mit_controller/gait_sequencer_interface.hpp"
#include "mit_controller/stage_loader.hpp"
#include "mit_controller/swing_leg_controller_interface.hpp"
#include "rclcpp/parameter_map.hpp"
#include "stage_selection.hpp"

namespace {

constexpr char kNodeName[] = "mit_controller_node";
constexpr char kOverlayDir[] = "overlays";

// The overlay `mit_controller.launch.py` resolves by bare filename, and the
// config it is layered over in (3).
constexpr char kBioGaitOverlay[] = "go2_bio_gait.yaml";
constexpr char kPassthroughSlcOverlay[] = "go2_example_passthrough_slc.yaml";
constexpr char kSimConfig[] = "mit_controller_sim_go2.yaml";

// The package that declares `example_passthrough_slc`, and the plugin id it
// declares. Named here rather than depended on: this suite must build and run
// whether or not that package is in the workspace.
constexpr char kExamplePackage[] = "example_stage_plugins";
constexpr char kPassthroughSlcPlugin[] = "example_passthrough_slc";

/** The six selection keys, as one list — the vocabulary of stage_loading.md §4. */
const std::vector<std::string>& SelectionKeys() {
  static const std::vector<std::string> keys = {stage_selection::kGaitSequencerTypeKey,
                                                stage_selection::kMPCTypeKey,
                                                stage_selection::kSwingLegControllerTypeKey,
                                                stage_selection::kWBCTypeKey,
                                                stage_selection::kModelAdaptationTypeKey,
                                                stage_selection::kContactLogicTypeKey};
  return keys;
}

std::string ConfigDir() { return ament_index_cpp::get_package_share_directory("controllers") + "/config"; }

std::string OverlayDir() { return ConfigDir() + "/" + kOverlayDir; }

/** Every `*.yaml` shipped in `config/overlays/`, by filename, sorted for stable output. */
std::vector<std::string> ShippedOverlays() {
  std::vector<std::string> files;
  const std::filesystem::path dir(OverlayDir());
  std::error_code ec;
  for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
    if (entry.is_regular_file() && entry.path().extension() == ".yaml") {
      files.push_back(entry.path().filename().string());
    }
  }
  // A missing or unreadable directory is a real defect (the install rule
  // changed), not a broken environment — report it rather than returning empty
  // and letting the count assertion blame the wrong thing.
  EXPECT_FALSE(ec) << "cannot read " << dir << ": " << ec.message();
  std::sort(files.begin(), files.end());
  return files;
}

/** The parameters `path` sets for `mit_controller_node`. */
std::vector<rclcpp::Parameter> LoadParams(const std::string& path) {
  const rclcpp::ParameterMap map = rclcpp::parameter_map_from_yaml_file(path);
  for (const auto& [node_fqn, params] : map) {
    if (node_fqn == kNodeName || node_fqn == std::string("/") + kNodeName) {
      return params;
    }
  }
  ADD_FAILURE() << "no '" << kNodeName << "' parameters in " << path
                << "; an overlay addressed to another node is layered on and silently does nothing";
  return {};
}

std::vector<rclcpp::Parameter> LoadOverlay(const std::string& file) { return LoadParams(OverlayDir() + "/" + file); }

/** The value of `key`, or an empty optional if the file does not set it. */
std::optional<rclcpp::ParameterValue> Find(const std::vector<rclcpp::Parameter>& params, const std::string& key) {
  const auto it = std::find_if(params.begin(), params.end(), [&key](const rclcpp::Parameter& param) {
    return param.get_name() == key;
  });
  if (it == params.end()) {
    return std::nullopt;
  }
  return it->get_parameter_value();
}

/** `expected` is declared for `base_class_type`, i.e. the loader could resolve it. */
template <class StageInterface>
void ExpectDeclared(const std::string& file,
                    const std::string& key,
                    const char* base_class_type,
                    const std::string& expected) {
  StageLoader<StageInterface> loader(base_class_type);
  const std::vector<std::string> declared = loader.DeclaredClasses();
  EXPECT_NE(std::find(declared.begin(), declared.end(), expected), declared.end())
      << file << ": '" << key << "' selects '" << expected << "', which is not declared for base '" << base_class_type
      << "'";
}

/** `key` is set to `expected` as a non-empty string. */
void ExpectSelects(const std::vector<rclcpp::Parameter>& params,
                   const std::string& file,
                   const std::string& key,
                   const std::string& expected) {
  const std::optional<rclcpp::ParameterValue> value = Find(params, key);
  ASSERT_TRUE(value.has_value()) << file << " does not set '" << key << "'";
  ASSERT_EQ(value->get_type(), rclcpp::ParameterType::PARAMETER_STRING)
      << file << ": '" << key << "' must be a string naming a plugin class";
  EXPECT_EQ(value->get<std::string>(), expected) << file << ": '" << key << "' no longer selects what it documents";
}

}  // namespace

/** (1) The overlays reached the install space. */
TEST(StageOverlays, ShippedOverlaysAreInstalled) {
  const std::vector<std::string> overlays = ShippedOverlays();
  EXPECT_FALSE(overlays.empty()) << "no overlay reached " << OverlayDir()
                                 << "; config/overlays/ rides on install(DIRECTORY config ...) and stage_overlay:= "
                                    "resolves bare filenames from there";
  for (const char* expected : {kBioGaitOverlay, kPassthroughSlcOverlay}) {
    EXPECT_NE(std::find(overlays.begin(), overlays.end(), expected), overlays.end())
        << expected << " is documented in doc/modularity/stage_overlays.md but is not installed";
  }
}

/** (2) Every shipped overlay parses, addresses the host, and selects a stage. */
TEST(StageOverlays, EveryShippedOverlaySelectsAStage) {
  for (const std::string& file : ShippedOverlays()) {
    const std::vector<rclcpp::Parameter> params = LoadOverlay(file);
    ASSERT_FALSE(params.empty()) << file << " sets no parameters at all";

    std::vector<std::string> selected;
    for (const std::string& key : SelectionKeys()) {
      const std::optional<rclcpp::ParameterValue> value = Find(params, key);
      if (!value.has_value()) {
        continue;
      }
      EXPECT_EQ(value->get_type(), rclcpp::ParameterType::PARAMETER_STRING)
          << file << ": '" << key << "' must be a string naming a plugin class";
      if (value->get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        EXPECT_FALSE(value->get<std::string>().empty()) << file << ": '" << key << "' is empty";
      }
      selected.push_back(key);
    }

    EXPECT_FALSE(selected.empty()) << file << " sets none of the six stage selection keys, so layering it changes no "
                                   << "stage. A misspelled key (gs.typ) reads exactly like this and is the likely "
                                   << "cause: the host would keep the stock default and report nothing.";
  }
}

/** (3) An overlay overlays; it does not restate the config. */
TEST(StageOverlays, EveryShippedOverlayIsSmallerThanTheConfigItOverlays) {
  const size_t stock = LoadParams(ConfigDir() + "/" + kSimConfig).size();
  ASSERT_GT(stock, 0u) << kSimConfig << " yielded no parameters";

  for (const std::string& file : ShippedOverlays()) {
    EXPECT_LT(LoadOverlay(file).size(), stock)
        << file << " sets as many parameters as " << kSimConfig
        << "; issue #19 is about swapping one stage *without* rewriting the full Go2 YAML";
  }
}

/** (4) The in-package overlay selects a declared stock class. */
TEST(StageOverlays, BioGaitOverlaySelectsTheBioSequencer) {
  const std::vector<rclcpp::Parameter> params = LoadOverlay(kBioGaitOverlay);
  const std::string key = stage_selection::kGaitSequencerTypeKey;
  ExpectSelects(params, kBioGaitOverlay, key, stage_selection::kBioGaitPlugin);
  ExpectDeclared<GaitSequencerInterface>(kBioGaitOverlay, key, stage_plugin_bases::kGaitSequencer,
                                         stage_selection::kBioGaitPlugin);

  // It must swap the gait and nothing else: the overlay's documented claim is
  // that every other stage keeps coming from the robot config.
  for (const std::string& other : SelectionKeys()) {
    if (other != key) {
      EXPECT_FALSE(Find(params, other).has_value()) << kBioGaitOverlay << " also sets '" << other << "'";
    }
  }
}

/**
 * (4) The out-of-package overlay selects the class `example_stage_plugins`
 * declares — asserted only when that package is actually in the workspace.
 */
TEST(StageOverlays, PassthroughSlcOverlaySelectsTheExamplePlugin) {
  const std::vector<rclcpp::Parameter> params = LoadOverlay(kPassthroughSlcOverlay);
  const std::string key = stage_selection::kSwingLegControllerTypeKey;
  ExpectSelects(params, kPassthroughSlcOverlay, key, kPassthroughSlcPlugin);

  try {
    ament_index_cpp::get_package_share_directory(kExamplePackage);
  } catch (const std::exception&) {
    GTEST_SKIP() << kExamplePackage << " is not in this workspace, so whether '" << kPassthroughSlcPlugin
                 << "' is declared says nothing about this package. Build it "
                    "(colcon build --packages-up-to example_stage_plugins) to check the selection resolves.";
  }
  ExpectDeclared<SwingLegControllerInterface>(kPassthroughSlcOverlay, key, stage_plugin_bases::kSwingLegController,
                                              kPassthroughSlcPlugin);
}
