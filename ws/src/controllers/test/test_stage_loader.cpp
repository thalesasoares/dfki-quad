// Stage loader test (issue #7, M2.2).
//
// M2.1 proved the plugin *schema* is discoverable (test_plugin_discovery.cpp);
// M1.5 proved the plugin *lifecycle* is implementable (test_stage_contracts.cpp,
// which drives a fake through create → Init → run → destroy by hand). This test
// covers the piece between them: StageLoader, the one place that turns a `type:`
// string into an initialised stage.
//
// Its main subject is the failure behaviour, because that is the acceptance
// criterion issue #7 actually cares about: a missing or invalid `type:` must
// produce a clear, named error and must never fall back to some other stage. A
// silently mis-loaded pipeline is the bug this layer exists to prevent, so every
// way of getting the selection wrong gets a case here, and each asserts that the
// message names the offending key or value.
//
// The plugins it loads are real, dlopen-ed shared objects built from
// test/stage_loader_test_plugins.cpp: the whole chain (XML → lookup → library
// load → default construction → Init) runs, not a mock of it. Their description
// file is passed explicitly rather than exported, so production discovery keeps
// seeing zero declared classes until M2.3 — see the file's own comment.
//
// Deps are pluginlib/common/interfaces/rclcpp/Eigen3 only — no drake, acados,
// ARC-OPT or hardware — so it runs headless on both CI lanes, like the other two
// modularity tests. See doc/modularity/stage_loading.md and contract_tests.md.

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "mit_controller/contact_logic_interface.hpp"
#include "mit_controller/gait_sequence.hpp"
#include "mit_controller/gait_sequencer_interface.hpp"
#include "mit_controller/gait_sequencer_types.hpp"
#include "mit_controller/joint_commands.hpp"
#include "mit_controller/mpc_interface.hpp"
#include "mit_controller/stage_loader.hpp"
#include "mit_controller/swing_leg_controller_interface.hpp"
#include "mit_controller/target.hpp"
#include "mit_controller/wbc_interface.hpp"
#include "model_adaptation/model_adaptation_interface.hpp"
#include "potato_sim/potato_model.hpp"

namespace {

// Mirrors the class names in test/stage_loader_test_plugins.xml.
constexpr const char* kOkPlugin = "stage_loader_test/ok";
constexpr const char* kFailingPlugin = "stage_loader_test/init_fails";

// The stage selection key. Spelled as the convention proposed in
// doc/modularity/stage_loading.md §4 — but note the loader never hard-codes it:
// it is an argument, so M2.5 (#10) can rename the vocabulary without touching
// the loader or this file's expectations about *behaviour*.
constexpr const char* kTypeKey = "gs.type";

// The required/optional keys of the test plugin, from
// test/stage_loader_test_plugins.cpp.
constexpr const char* kRequiredHeightKey = "loader_test.required_height";
constexpr const char* kOptionalTypeKey = "loader_test.optional_type";

constexpr double kRequiredHeight = 0.42;

StageLoader<GaitSequencerInterface> MakeTestLoader() {
  return StageLoader<GaitSequencerInterface>(stage_plugin_bases::kGaitSequencer,
                                             {STAGE_LOADER_TEST_PLUGIN_XML});
}

// The context the host hands a stage: model/state clones plus its whole
// parameter map (plugin_lifecycle.md §3). `type_value` is omitted from the map
// when empty, which is how the "no stage selected" case is built.
StageInit MakeInit(const std::string& type_value, bool with_required_key = true,
                   bool with_optional_key = false) {
  StageInit init;
  init.model = std::make_unique<BrickModel>(Eigen::Matrix3d::Identity(), 1.0);
  init.state = std::make_unique<BrickState>();
  if (!type_value.empty()) {
    init.params.emplace(kTypeKey, rclcpp::ParameterValue(type_value));
  }
  if (with_required_key) {
    init.params.emplace(kRequiredHeightKey, rclcpp::ParameterValue(kRequiredHeight));
  }
  if (with_optional_key) {
    init.params.emplace(kOptionalTypeKey, rclcpp::ParameterValue(std::string("adaptive")));
  }
  return init;
}

bool Mentions(const std::string& message, const std::string& needle) {
  return message.find(needle) != std::string::npos;
}

// -----------------------------------------------------------------------------
// The success path: resolve → create → Init → drive through the frozen interface.
// -----------------------------------------------------------------------------

TEST(StageLoader, LoadResolvesCreatesAndInitialises) {
  auto loader = MakeTestLoader();
  auto plugin = loader.Load(kTypeKey, MakeInit(kOkPlugin, true, true));
  ASSERT_NE(plugin, nullptr);

  // Everything below is read through the frozen stage interface — the way the
  // host will use it — not through the plugin's concrete type.
  GaitSequencerInterface& stage = *plugin;
  stage.UpdateTarget(Target{});
  GaitSequence sequence{};
  stage.GetGaitSequence(sequence);

  // MOVE proves Init ran and both clones were handed over; the echoed height
  // proves the required parameter arrived with its value intact.
  EXPECT_EQ(sequence.sequence_mode, GaitSequence::MOVE);
  EXPECT_DOUBLE_EQ(sequence.target_height, kRequiredHeight);
  // The optional key reached the stage too: the host passes its *whole* map, not
  // just the selection key.
  EXPECT_EQ(stage.GetType(), GS_Type::ADAPTIVE);
}

TEST(StageLoader, OptionalStageParameterDefaultsInsideTheStage) {
  auto loader = MakeTestLoader();
  auto plugin = loader.Load(kTypeKey, MakeInit(kOkPlugin, true, false));
  ASSERT_NE(plugin, nullptr);

  // The host supplied no value for the optional key and the load still succeeds:
  // defaults live in the stage, not in the host (plugin_lifecycle.md §3).
  EXPECT_EQ(plugin->GetType(), GS_Type::SIMPLE);
}

TEST(StageLoader, DeclaredClassesListsTheTestPlugins) {
  auto loader = MakeTestLoader();
  const std::vector<std::string> declared = loader.DeclaredClasses();
  EXPECT_EQ(declared.size(), 2u);
  EXPECT_NE(std::find(declared.begin(), declared.end(), kOkPlugin), declared.end());
  EXPECT_NE(std::find(declared.begin(), declared.end(), kFailingPlugin), declared.end());
  EXPECT_EQ(loader.BaseClassType(), stage_plugin_bases::kGaitSequencer);
}

// -----------------------------------------------------------------------------
// The fail-fast paths. Each asserts both the exception type and that the message
// names what went wrong — an error that does not say which key or which value is
// only marginally better than the silent fallback we are ruling out.
// -----------------------------------------------------------------------------

TEST(StageLoader, MissingTypeKeyIsFatalAndNamesTheKey) {
  auto loader = MakeTestLoader();
  try {
    loader.Load(kTypeKey, MakeInit(""));
    FAIL() << "a missing stage selection parameter must not load anything";
  } catch (const StageLoadError& error) {
    EXPECT_TRUE(Mentions(error.what(), kTypeKey)) << error.what();
    // and it tells the reader what could have been chosen instead
    EXPECT_TRUE(Mentions(error.what(), kOkPlugin)) << error.what();
  }
}

TEST(StageLoader, NonStringTypeKeyIsFatalAndNamesTheKey) {
  auto loader = MakeTestLoader();
  StageInit init = MakeInit("");
  init.params.emplace(kTypeKey, rclcpp::ParameterValue(true));
  try {
    loader.Load(kTypeKey, std::move(init));
    FAIL() << "a non-string stage selection parameter must not load anything";
  } catch (const StageLoadError& error) {
    EXPECT_TRUE(Mentions(error.what(), kTypeKey)) << error.what();
    EXPECT_TRUE(Mentions(error.what(), "string")) << error.what();
  }
}

TEST(StageLoader, EmptyTypeKeyIsFatalAndNamesTheKey) {
  auto loader = MakeTestLoader();
  StageInit init = MakeInit("");
  init.params.emplace(kTypeKey, rclcpp::ParameterValue(std::string("")));
  try {
    loader.Load(kTypeKey, std::move(init));
    FAIL() << "an empty stage selection parameter must not load anything";
  } catch (const StageLoadError& error) {
    EXPECT_TRUE(Mentions(error.what(), kTypeKey)) << error.what();
  }
}

TEST(StageLoader, UnknownTypeIsFatalAndListsTheDeclaredClasses) {
  auto loader = MakeTestLoader();
  try {
    loader.Load(kTypeKey, MakeInit("stage_loader_test/does_not_exist"));
    FAIL() << "an unknown stage type must not fall back to a declared plugin";
  } catch (const StageLoadError& error) {
    // The bad value, so the reader sees their typo...
    EXPECT_TRUE(Mentions(error.what(), "stage_loader_test/does_not_exist")) << error.what();
    // ...and the alternatives, so fixing it does not need a hunt through the XML.
    EXPECT_TRUE(Mentions(error.what(), kOkPlugin)) << error.what();
    EXPECT_TRUE(Mentions(error.what(), kFailingPlugin)) << error.what();
  }
}

// A stage that exists but refuses to start is a *different* failure from a stage
// that does not exist, and the exception type says which — the host must be able
// to tell "your YAML names nothing" from "your YAML names something broken".
TEST(StageLoader, StageInitFailureSurfacesAsStageInitErrorNamingTheStage) {
  auto loader = MakeTestLoader();
  try {
    loader.Load(kTypeKey, MakeInit(kFailingPlugin));
    FAIL() << "a stage whose Init throws must not be handed to the host";
  } catch (const StageInitError& error) {
    EXPECT_TRUE(Mentions(error.what(), kFailingPlugin)) << error.what();
    EXPECT_TRUE(Mentions(error.what(), "loader_test.deliberate_init_failure")) << error.what();
  }
}

TEST(StageLoader, MissingRequiredStageParameterFailsTheLoad) {
  auto loader = MakeTestLoader();
  try {
    loader.Load(kTypeKey, MakeInit(kOkPlugin, /*with_required_key=*/false));
    FAIL() << "a stage missing a required parameter must not be handed to the host";
  } catch (const StageInitError& error) {
    // StageInit::Require named the key; the loader added which stage asked for it.
    EXPECT_TRUE(Mentions(error.what(), kRequiredHeightKey)) << error.what();
    EXPECT_TRUE(Mentions(error.what(), kOkPlugin)) << error.what();
  }
}

TEST(StageLoader, StageLoadErrorAndStageInitErrorAreDistinguishable) {
  auto loader = MakeTestLoader();
  // Neither failure is catchable as the other: the taxonomy is usable at the
  // catch site, not just in the message text.
  EXPECT_THROW(loader.Load(kTypeKey, MakeInit("stage_loader_test/does_not_exist")), StageLoadError);
  EXPECT_THROW(loader.Load(kTypeKey, MakeInit(kFailingPlugin)), StageInitError);
}

// -----------------------------------------------------------------------------
// The reconfiguration seam (plugin_lifecycle.md §5): the host needs to build the
// replacement instance *before* it takes the stage lock, so creation and Init
// must be separable.
// -----------------------------------------------------------------------------

TEST(StageLoader, CreateDoesNotInitialiseSoTheHostCanInitOffLock) {
  auto loader = MakeTestLoader();
  auto plugin = loader.Create(kOkPlugin);
  ASSERT_NE(plugin, nullptr);

  // Seeded to MOVE so that reading KEEP back proves the stage actively reported
  // "not initialised" rather than simply leaving the caller's value alone.
  GaitSequence before{};
  before.sequence_mode = GaitSequence::MOVE;
  plugin->GetGaitSequence(before);
  EXPECT_EQ(before.sequence_mode, GaitSequence::KEEP) << "Create must not run Init";

  plugin->Init(MakeInit(kOkPlugin));

  GaitSequence after{};
  plugin->GetGaitSequence(after);
  EXPECT_EQ(after.sequence_mode, GaitSequence::MOVE);
  EXPECT_DOUBLE_EQ(after.target_height, kRequiredHeight);
}

// The whole reconfiguration sequence M2.4 will run: build and initialise the
// replacement off-loop, swap it in, then let the old instance go. Written
// against the member type the host will declare (PluginPtr, default-constructed
// then move-assigned) so that pattern is proven here and not discovered in M2.4.
TEST(StageLoader, ReplacementInstanceCanBeSwappedInAfterInit) {
  auto loader = MakeTestLoader();
  StageLoader<GaitSequencerInterface>::PluginPtr stage;  // the host's member
  ASSERT_EQ(stage, nullptr);

  stage = loader.Load(kTypeKey, MakeInit(kOkPlugin));

  auto replacement = loader.Create(kOkPlugin);
  replacement->Init(MakeInit(kOkPlugin, true, true));  // off-loop, before the swap
  stage = std::move(replacement);                      // the swap itself

  GaitSequence sequence{};
  stage->GetGaitSequence(sequence);
  EXPECT_EQ(sequence.sequence_mode, GaitSequence::MOVE);
  EXPECT_EQ(stage->GetType(), GS_Type::ADAPTIVE) << "the swapped-in instance is the new one";
}

TEST(StageLoader, CreateRejectsUnknownClasses) {
  auto loader = MakeTestLoader();
  EXPECT_THROW(loader.Create("stage_loader_test/does_not_exist"), StageLoadError);
}

// -----------------------------------------------------------------------------
// The schema the loader shares with the XML and with M2.3's wrappers.
// -----------------------------------------------------------------------------

// These literals are the ones written out in doc/modularity/plugin_discovery.md
// §3 and in the plugins/ description files. Duplicating them here on purpose:
// this is the assertion that the constants and the schema have not drifted, so
// it must not be written in terms of the constants it is checking.
TEST(StageLoader, BaseClassStringsMatchThePluginSchema) {
  EXPECT_EQ(std::string(stage_plugin_bases::kGaitSequencer), "StagePlugin<GaitSequencerInterface>");
  EXPECT_EQ(std::string(stage_plugin_bases::kMPC), "StagePlugin<MPCInterface>");
  EXPECT_EQ(std::string(stage_plugin_bases::kSwingLegController), "StagePlugin<SwingLegControllerInterface>");
  EXPECT_EQ(std::string(stage_plugin_bases::kWBC), "StagePlugin<WBCInterface>");
  EXPECT_EQ(std::string(stage_plugin_bases::kModelAdaptation), "StagePlugin<ModelAdaptationInterface>");
  EXPECT_EQ(std::string(stage_plugin_bases::kContactLogic), "StagePlugin<ContactLogicInterface>");
  EXPECT_EQ(std::string(kStagePluginPackage), "controllers");
}

// The production loaders — built through ament index discovery, no explicit XML —
// construct for every stage base and declare the stock IDs M2.3 (#8) added. This
// is test_plugin_discovery.cpp's assertion re-run through the API the host will
// actually use, so if StageLoader ever stops pointing at the same
// package/resource as a bare ClassLoader, it fails here. The contact stage joined
// the list in M3.1 (#12), and `bio_gait` in M4.1 (#16).
//
// Containment, not equality, since M4.2 (#17): out-of-package packages may now
// declare stages of their own against these same bases, so the exact set depends
// on what else is installed rather than on this package being correct. See the
// header comment of test_plugin_discovery.cpp for the full rationale; the two
// assertions moved together on purpose, because they are the same statement made
// through two different APIs.
template <class StageInterface>
void ExpectProductionLoaderDeclares(const char* base_class_type, const std::vector<std::string>& expected) {
  StageLoader<StageInterface> loader(base_class_type);
  const std::vector<std::string> declared = loader.DeclaredClasses();
  for (const auto& stock_id : expected) {
    EXPECT_NE(std::find(declared.begin(), declared.end(), stock_id), declared.end())
        << "stock plugin '" << stock_id << "' is not declared for base " << base_class_type;
  }
}

TEST(StageLoader, ProductionLoadersDeclareStockPluginsForEveryStageBase) {
  ExpectProductionLoaderDeclares<GaitSequencerInterface>(stage_plugin_bases::kGaitSequencer,
                                                         {"simple_gait", "adaptive_gait", "bio_gait"});
  ExpectProductionLoaderDeclares<MPCInterface>(stage_plugin_bases::kMPC, {"acados_mpc"});
  ExpectProductionLoaderDeclares<SwingLegControllerInterface>(stage_plugin_bases::kSwingLegController,
                                                             {"bezier_swing"});
  ExpectProductionLoaderDeclares<WBCInterface>(stage_plugin_bases::kWBC, {"wbc_arc_opt", "inverse_dynamics"});
  ExpectProductionLoaderDeclares<ModelAdaptationInterface>(stage_plugin_bases::kModelAdaptation,
                                                           {"kf_adaptation", "rls_adaptation"});
  ExpectProductionLoaderDeclares<ContactLogicInterface>(stage_plugin_bases::kContactLogic,
                                                        {"default_contact_logic"});
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
