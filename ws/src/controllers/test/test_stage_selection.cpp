// Stage selection tests (issue #9, M2.4).
//
// The host picks each pipeline stage with one string — the `*.type` parameter —
// and derives that string's *default* from the parameter that used to select the
// implementation inside the deleted host factories (`stage_selection.hpp`). Two
// properties have to hold for that bridge to be safe, and both are checked here:
//
//   1. The derivation reproduces the old selection exactly, and passes anything
//      it does not recognise through untouched so the loader fails loudly on it
//      rather than substituting a stage (stage_loading.md §1).
//   2. Every default the host would hand to StageLoader actually names a plugin
//      declared for that stage's base. This is the test that fails if a stock
//      plugin is renamed in plugins/*.xml without updating the host defaults —
//      a mistake that would otherwise only show up as a refusal to start.
//
// (2) uses production ament discovery, so the CMake target appends
// CMAKE_INSTALL_PREFIX to AMENT_PREFIX_PATH, exactly as test_stock_plugins does.
// Nothing here loads or initialises a stage: this is about names, and
// test_stock_plugins.cpp already proves the names load.

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "mit_controller/contact_logic_interface.hpp"
#include "mit_controller/gait_sequencer_interface.hpp"
#include "mit_controller/mpc_interface.hpp"
#include "mit_controller/stage_loader.hpp"
#include "mit_controller/swing_leg_controller_interface.hpp"
#include "mit_controller/wbc_interface.hpp"
#include "model_adaptation/model_adaptation_interface.hpp"
#include "stage_selection.hpp"

namespace {

template <class StageInterface>
void ExpectDeclared(const char* base_class_type, const std::string& lookup_name) {
  StageLoader<StageInterface> loader(base_class_type);
  const std::vector<std::string> declared = loader.DeclaredClasses();
  EXPECT_NE(std::find(declared.begin(), declared.end(), lookup_name), declared.end())
      << "host default '" << lookup_name << "' is not declared for base '" << base_class_type << "'";
}

}  // namespace

// --- 1. The legacy derivation ------------------------------------------------

TEST(StageSelection, GaitSequencerDerivesTheTwoLegacyNames) {
  EXPECT_EQ(stage_selection::GaitSequencerTypeFromLegacy("Simple"), "simple_gait");
  EXPECT_EQ(stage_selection::GaitSequencerTypeFromLegacy("Adaptive"), "adaptive_gait");
}

TEST(StageSelection, GaitSequencerPassesUnknownLegacyValuesThrough) {
  // No correction, no default: the loader must be the one to reject it, with a
  // message listing what is declared. Note "simple" is deliberately *not*
  // case-corrected — the old factory did not accept it either.
  EXPECT_EQ(stage_selection::GaitSequencerTypeFromLegacy("simple"), "simple");
  EXPECT_EQ(stage_selection::GaitSequencerTypeFromLegacy("Bio"), "Bio");
  EXPECT_EQ(stage_selection::GaitSequencerTypeFromLegacy(""), "");
}

TEST(StageSelection, GaitSequencerAcceptsAPluginNameAlreadyInTheNewSpelling) {
  // A user who writes the plugin class name into the legacy key gets it verbatim,
  // so migrating one key at a time works.
  EXPECT_EQ(stage_selection::GaitSequencerTypeFromLegacy("adaptive_gait"), "adaptive_gait");
}

TEST(StageSelection, ModelAdaptationFollowsTheLegacyModeSwitch) {
  // The deleted switch had `case 1:` for recursive least squares and everything
  // else fell into `default:` = Kalman filter.
  EXPECT_EQ(stage_selection::ModelAdaptationTypeFromLegacy(1), "rls_adaptation");
  EXPECT_EQ(stage_selection::ModelAdaptationTypeFromLegacy(0), "kf_adaptation");
  EXPECT_EQ(stage_selection::ModelAdaptationTypeFromLegacy(2), "kf_adaptation");
  EXPECT_EQ(stage_selection::ModelAdaptationTypeFromLegacy(-1), "kf_adaptation");
}

TEST(StageSelection, WBCFollowsTheBuildFlavour) {
  EXPECT_STREQ(stage_selection::WBCTypeForBuild(true), "wbc_arc_opt");
  EXPECT_STREQ(stage_selection::WBCTypeForBuild(false), "inverse_dynamics");
}

// The contact stage's bridge maps *keys*, not values (issue #12, M3.1): the four
// pre-M3.1 flat toggles onto the ratified contact_logic.* spelling. Getting a
// mapping wrong would silently drop a toggle — the host would re-set a key the
// stage does not recognise — so both directions are pinned.
TEST(StageSelection, ContactLogicBridgesEveryLegacyToggle) {
  EXPECT_STREQ(stage_selection::ContactLogicKeyFromLegacy(stage_selection::kLegacyEarlyContactDetectionKey),
               contact_logic_params::kEarlyContactDetection);
  EXPECT_STREQ(stage_selection::ContactLogicKeyFromLegacy(stage_selection::kLegacyLateContactDetectionKey),
               contact_logic_params::kLateContactDetection);
  EXPECT_STREQ(stage_selection::ContactLogicKeyFromLegacy(stage_selection::kLegacyLostContactDetectionKey),
               contact_logic_params::kLostContactDetection);
  EXPECT_STREQ(
      stage_selection::ContactLogicKeyFromLegacy(stage_selection::kLegacyLateContactRescheduleSwingPhaseKey),
      contact_logic_params::kLateContactRescheduleSwingPhase);
}

// Everything else must fall through, or the host would route unrelated
// parameters into the contact stage. In particular the *nested* keys must not
// bridge onto themselves: that branch runs before the routing branch would, and
// would turn every contact parameter change into an infinite re-set.
TEST(StageSelection, ContactLogicBridgeIgnoresEverythingElse) {
  EXPECT_EQ(stage_selection::ContactLogicKeyFromLegacy(contact_logic_params::kEarlyContactDetection), nullptr);
  EXPECT_EQ(stage_selection::ContactLogicKeyFromLegacy(stage_selection::kContactLogicTypeKey), nullptr);
  EXPECT_EQ(stage_selection::ContactLogicKeyFromLegacy("use_model_adaptation"), nullptr);
  EXPECT_EQ(stage_selection::ContactLogicKeyFromLegacy(""), nullptr);
}

// --- 2. The defaults name declared plugins -----------------------------------

TEST(StageSelection, EveryHostDefaultIsADeclaredPlugin) {
  ExpectDeclared<GaitSequencerInterface>(stage_plugin_bases::kGaitSequencer,
                                         stage_selection::GaitSequencerTypeFromLegacy("Simple"));
  ExpectDeclared<GaitSequencerInterface>(stage_plugin_bases::kGaitSequencer,
                                         stage_selection::GaitSequencerTypeFromLegacy("Adaptive"));
  ExpectDeclared<MPCInterface>(stage_plugin_bases::kMPC, stage_selection::kAcadosMPCPlugin);
  ExpectDeclared<SwingLegControllerInterface>(stage_plugin_bases::kSwingLegController,
                                              stage_selection::kBezierSwingPlugin);
  ExpectDeclared<ModelAdaptationInterface>(stage_plugin_bases::kModelAdaptation,
                                           stage_selection::ModelAdaptationTypeFromLegacy(0));
  ExpectDeclared<ModelAdaptationInterface>(stage_plugin_bases::kModelAdaptation,
                                           stage_selection::ModelAdaptationTypeFromLegacy(1));
  // Both WBC flavours are declared for the one WBC base, independent of how this
  // test happens to be compiled — the point of #13 (M3.2). WBCTypeForBuild still
  // maps the build flavour onto a *default* string, so both of its answers must
  // still name something loadable.
  ExpectDeclared<WBCInterface>(stage_plugin_bases::kWBC, stage_selection::WBCTypeForBuild(true));
  ExpectDeclared<WBCInterface>(stage_plugin_bases::kWBC, stage_selection::WBCTypeForBuild(false));
  ExpectDeclared<ContactLogicInterface>(stage_plugin_bases::kContactLogic,
                                        stage_selection::kDefaultContactLogicPlugin);
}
