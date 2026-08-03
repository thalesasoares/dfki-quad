// Test-only stage plugins for the loader test (issue #7, M2.2).
//
// M2.2 adds the loader but no production plugin classes — the stock wrappers are
// M2.3 (issue #8) and until they land every declared-class list is empty. To test
// a loader against an empty world would be to test nothing, so this translation
// unit provides the smallest possible real plugins: built into their own shared
// library, registered with PLUGINLIB_EXPORT_CLASS, and loaded through
// pluginlib's ordinary dlopen path. The loader test therefore exercises the
// whole chain — XML → class lookup → library load → default construction →
// Init — rather than a mock of it.
//
// These are deliberately invisible to production discovery: the accompanying
// stage_loader_test_plugins.xml is *not* passed to
// pluginlib_export_plugin_description_file(), so it never enters the
// `controllers__pluginlib__plugin` ament resource and the M2.1 discovery test
// keeps (correctly) seeing zero declared classes for every stage base. The
// loader test reaches it by handing the file to StageLoader's explicit
// plugin_xml_paths constructor argument.
//
// Everything observable about Init is exposed through the *frozen* stage
// interface (GaitSequenceInterface's GetGaitSequence / GetType), never through a
// cross-library global: the test binary and this library are separate
// translation units in separate shared objects, and the point being tested is
// exactly that the host can drive a loaded stage through its interface.

#include <pluginlib/class_list_macros.hpp>

#include <memory>
#include <string>
#include <utility>

#include "common/model_interface.hpp"
#include "common/state_interface.hpp"
#include "interfaces/msg/gait_state.hpp"
#include "mit_controller/gait_sequence.hpp"
#include "mit_controller/gait_sequencer_interface.hpp"
#include "mit_controller/gait_sequencer_types.hpp"
#include "mit_controller/stage_plugin.hpp"
#include "mit_controller/target.hpp"

namespace stage_loader_test {

/**
 * A gait sequencer plugin that does nothing but report what it was initialised
 * with. Its Init follows the three rules a real stock wrapper's Init will
 * (plugin_lifecycle.md §3): take ownership of the model/state clones, fail loudly
 * on a missing required key, and default an optional key *in the stage* rather
 * than expecting the host to supply it.
 */
class LoaderTestGaitSequencer final : public StagePlugin<GaitSequencerInterface> {
 public:
  // A key the stage cannot start without — the StageInit::Require path.
  static constexpr const char* kRequiredHeightKey = "loader_test.required_height";
  // An optional key whose default lives here, not in the host.
  static constexpr const char* kOptionalTypeKey = "loader_test.optional_type";

  void Init(StageInit init) override {
    required_height_ = init.Require(kRequiredHeightKey).get<double>();

    const auto optional_type = init.params.find(kOptionalTypeKey);
    if (optional_type != init.params.end() && optional_type->second.get<std::string>() == "adaptive") {
      type_ = GS_Type::ADAPTIVE;
    }

    model_ = std::move(init.model);
    state_ = std::move(init.state);
  }

  // The observable proof that Init ran: MOVE means both clones arrived,
  // target_height echoes the required parameter's value. Only these two fields
  // are written — assigning a fresh `GaitSequence{}` over the caller's object
  // would move-assign its uninitialised Eigen members, which -Ofast diagnoses as
  // -Wuninitialized.
  void GetGaitSequence(GaitSequence& gait_sequence) override {
    gait_sequence.sequence_mode = (model_ != nullptr && state_ != nullptr) ? GaitSequence::MOVE : GaitSequence::KEEP;
    gait_sequence.target_height = required_height_;
  }

  void UpdateState(const StateInterface& quad_state) override { (void)quad_state; }
  void UpdateModel(const ModelInterface& quad_model) override { (void)quad_model; }
  void UpdateTarget(const Target& new_target) override { (void)new_target; }
  void GetGaitState(interfaces::msg::GaitState& state) override { state = interfaces::msg::GaitState{}; }
  GS_Type GetType() const override { return type_; }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    (void)name;
    (void)value;
    return false;
  }

 private:
  std::unique_ptr<ModelInterface> model_;
  std::unique_ptr<StateInterface> state_;
  double required_height_ = 0.0;
  GS_Type type_ = GS_Type::SIMPLE;
};

/**
 * A plugin that is declared and loadable but always refuses to initialise. It
 * stands for the "the stage you asked for exists but cannot configure itself"
 * case, which the loader must surface as a StageInitError (not a StageLoadError)
 * and must never paper over by substituting another stage.
 */
class LoaderTestFailingGaitSequencer final : public StagePlugin<GaitSequencerInterface> {
 public:
  static constexpr const char* kFailureMarker = "loader_test.deliberate_init_failure";

  void Init(StageInit init) override {
    (void)init;
    throw StageInitError(kFailureMarker);
  }

  // Unreachable in practice — Init always throws, so the host never gets this
  // instance — but the interface must still be satisfied.
  void GetGaitSequence(GaitSequence& gait_sequence) override { (void)gait_sequence; }
  void UpdateState(const StateInterface& quad_state) override { (void)quad_state; }
  void UpdateModel(const ModelInterface& quad_model) override { (void)quad_model; }
  void UpdateTarget(const Target& new_target) override { (void)new_target; }
  void GetGaitState(interfaces::msg::GaitState& state) override { state = interfaces::msg::GaitState{}; }
  GS_Type GetType() const override { return GS_Type::SIMPLE; }
  bool SetParameter(const std::string& name, const rclcpp::ParameterValue& value) override {
    (void)name;
    (void)value;
    return false;
  }
};

}  // namespace stage_loader_test

// The base_class_type strings here are the frozen schema of
// doc/modularity/plugin_discovery.md §3, spelled exactly as
// stage_plugin_bases::kGaitSequencer and as the XML (which must escape the '<'
// as &lt;). M2.3's stock wrappers register the same way.
PLUGINLIB_EXPORT_CLASS(stage_loader_test::LoaderTestGaitSequencer, StagePlugin<GaitSequencerInterface>)
PLUGINLIB_EXPORT_CLASS(stage_loader_test::LoaderTestFailingGaitSequencer, StagePlugin<GaitSequencerInterface>)
