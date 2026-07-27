// Go2 YAML stage selection tests (issue #10, M2.5).
//
// M2.5 wrote an explicit `*.type` key for every pipeline stage into the two
// shipped Go2 configs, replacing the implicit selection they previously
// inherited from the host's legacy bridge (`stage_selection.hpp`). The issue's
// acceptance criterion — "defaults match current stock Go2 behavior" — is a
// property of those *files*, so this test reads the installed YAMLs and pins it
// instead of leaving it to a reviewer's eye:
//
//   1. All five keys are present, string-typed and non-empty, spelled with the
//      stage_selection::k*TypeKey constants — so renaming a key breaks a test.
//      That is deliberate: M2.5 ratified the vocabulary (stage_loading.md §4),
//      and a later rename should be a conscious edit here, not a silent drift
//      between the host and the configs.
//   2. Every value names a plugin actually declared for that stage's base. The
//      same check test_stage_selection.cpp applies to the host's *defaults*, now
//      applied to what the configs actually ship — a stock plugin renamed in
//      plugins/*.xml would otherwise surface as a robot refusing to start.
//   3. Every value equals the default the legacy bridge derived before M2.5,
//      which is what makes the schema change behaviour-neutral. Changing a stock
//      default (#16's bio_gait, say) must edit this test consciously.
//   4. The legacy selectors `gait_sequencer` and `ma_mode` are gone from both
//      files. Two spellings of one choice must not coexist: with an explicit
//      `*.type` present the legacy key is a decoy that silently does nothing.
//
// (2) uses production ament discovery, so the CMake target appends
// CMAKE_INSTALL_PREFIX to AMENT_PREFIX_PATH, exactly as test_stage_selection
// does. Nothing here loads or initialises a stage.

#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "mit_controller/gait_sequencer_interface.hpp"
#include "mit_controller/mpc_interface.hpp"
#include "mit_controller/stage_loader.hpp"
#include "mit_controller/swing_leg_controller_interface.hpp"
#include "mit_controller/wbc_interface.hpp"
#include "model_adaptation/model_adaptation_interface.hpp"
#include "rclcpp/parameter_map.hpp"
#include "stage_selection.hpp"

namespace {

constexpr char kNodeName[] = "mit_controller_node";
constexpr char kSimConfig[] = "mit_controller_sim_go2.yaml";
constexpr char kRealConfig[] = "mit_controller_real_go2.yaml";

/**
 * The parameters the shipped config sets for `mit_controller_node`, read from
 * the *installed* share directory — the same file `mit_controller.launch.py`
 * passes to the node, not the source tree copy.
 */
std::vector<rclcpp::Parameter> LoadShippedConfig(const std::string& config_file) {
  const std::string path = ament_index_cpp::get_package_share_directory("controllers") + "/config/" + config_file;
  // A missing file here is a real defect (config/ dropped from install(), or a
  // renamed config), not a broken environment: fail rather than skip.
  const rclcpp::ParameterMap map = rclcpp::parameter_map_from_yaml_file(path);
  for (const auto& [node_fqn, params] : map) {
    if (node_fqn == kNodeName || node_fqn == std::string("/") + kNodeName) {
      return params;
    }
  }
  ADD_FAILURE() << "no '" << kNodeName << "' parameters in " << path;
  return {};
}

/** The value of `key`, or an empty optional if the config does not set it. */
std::optional<rclcpp::ParameterValue> Find(const std::vector<rclcpp::Parameter>& params, const std::string& key) {
  const auto it = std::find_if(params.begin(), params.end(), [&key](const rclcpp::Parameter& param) {
    return param.get_name() == key;
  });
  if (it == params.end()) {
    return std::nullopt;
  }
  return it->get_parameter_value();
}

/**
 * `key` is set to `expected`, as a non-empty string, and `expected` names a
 * plugin declared for `base_class_type`.
 */
template <class StageInterface>
void ExpectSelects(const std::vector<rclcpp::Parameter>& params,
                   const std::string& config_file,
                   const std::string& key,
                   const char* base_class_type,
                   const std::string& expected) {
  const std::optional<rclcpp::ParameterValue> value = Find(params, key);
  ASSERT_TRUE(value.has_value()) << config_file << " does not set '" << key << "'";
  ASSERT_EQ(value->get_type(), rclcpp::ParameterType::PARAMETER_STRING)
      << config_file << ": '" << key << "' must be a string naming a plugin class";
  const std::string selected = value->get<std::string>();
  ASSERT_FALSE(selected.empty()) << config_file << ": '" << key << "' is empty";

  // The behaviour-match criterion: the config selects what the pre-M2.5 legacy
  // derivation selected.
  EXPECT_EQ(selected, expected) << config_file << ": '" << key << "' no longer matches the stock Go2 default";

  StageLoader<StageInterface> loader(base_class_type);
  const std::vector<std::string> declared = loader.DeclaredClasses();
  EXPECT_NE(std::find(declared.begin(), declared.end(), selected), declared.end())
      << config_file << ": '" << key << "' selects '" << selected << "', which is not declared for base '"
      << base_class_type << "'";
}

/** Both Go2 configs must select the same stack; only tuning differs between them. */
void ExpectStockGo2Selection(const std::string& config_file) {
  const std::vector<rclcpp::Parameter> params = LoadShippedConfig(config_file);
  ASSERT_FALSE(params.empty()) << config_file << " yielded no parameters";

  ExpectSelects<GaitSequencerInterface>(params,
                                        config_file,
                                        stage_selection::kGaitSequencerTypeKey,
                                        stage_plugin_bases::kGaitSequencer,
                                        stage_selection::GaitSequencerTypeFromLegacy("Simple"));
  ExpectSelects<MPCInterface>(params,
                              config_file,
                              stage_selection::kMPCTypeKey,
                              stage_plugin_bases::kMPC,
                              stage_selection::kAcadosMPCPlugin);
  ExpectSelects<SwingLegControllerInterface>(params,
                                             config_file,
                                             stage_selection::kSwingLegControllerTypeKey,
                                             stage_plugin_bases::kSwingLegController,
                                             stage_selection::kBezierSwingPlugin);
  ExpectSelects<ModelAdaptationInterface>(params,
                                          config_file,
                                          stage_selection::kModelAdaptationTypeKey,
                                          stage_plugin_bases::kModelAdaptation,
                                          stage_selection::ModelAdaptationTypeFromLegacy(0));
  // The Go2 configs are only ever loaded by a Go2 build, whose WBC takes joint
  // commands (USE_WBC == true); the ULab flavour has its own configs and is
  // covered by test_stage_selection.
  ExpectSelects<WBCInterface<JointTorqueVelocityPositionCommands>>(params,
                                                                   config_file,
                                                                   stage_selection::kWBCTypeKey,
                                                                   stage_plugin_bases::kWBC,
                                                                   stage_selection::WBCTypeForBuild(true));
}

/** No legacy selector survives next to the explicit keys. */
void ExpectNoLegacySelectors(const std::string& config_file) {
  const std::vector<rclcpp::Parameter> params = LoadShippedConfig(config_file);
  ASSERT_FALSE(params.empty()) << config_file << " yielded no parameters";

  EXPECT_FALSE(Find(params, stage_selection::kLegacyGaitSequencerKey).has_value())
      << config_file << " still sets '" << stage_selection::kLegacyGaitSequencerKey
      << "'; with an explicit gs.type present it selects nothing and only misleads";
  EXPECT_FALSE(Find(params, stage_selection::kLegacyModelAdaptationModeKey).has_value())
      << config_file << " still sets '" << stage_selection::kLegacyModelAdaptationModeKey
      << "'; with an explicit model_adaptation.type present it selects nothing and only misleads";
}

}  // namespace

TEST(Go2YamlStageSelection, SimConfigSelectsTheStockStack) { ExpectStockGo2Selection(kSimConfig); }

TEST(Go2YamlStageSelection, RealConfigSelectsTheStockStack) { ExpectStockGo2Selection(kRealConfig); }

TEST(Go2YamlStageSelection, SimConfigCarriesNoLegacySelector) { ExpectNoLegacySelectors(kSimConfig); }

TEST(Go2YamlStageSelection, RealConfigCarriesNoLegacySelector) { ExpectNoLegacySelectors(kRealConfig); }
