#pragma once

#include <rclcpp/parameter_value.hpp>

#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include "common/model_interface.hpp"
#include "common/state_interface.hpp"

/**
 * Plugin lifecycle layer for the control-pipeline stages.
 *
 * pluginlib instantiates a plugin through its no-argument constructor, but every
 * concrete stage today is built with constructor injection: model/state clones
 * plus a long list of typed parameters (see stage_contracts.md §3, start-up
 * ordering point 3). This header specifies the two-phase initialisation that
 * reconciles the two, *without* touching the five frozen stage interfaces: a
 * plugin derives from `StagePlugin<Interface>`, is default-constructed by the
 * loader, and receives everything it used to get in its constructor through a
 * single `Init` call.
 *
 * The full contract — call ordering, parameter semantics, reconfiguration and
 * threading — is doc/modularity/plugin_lifecycle.md. This header is the
 * specification's implementation, the same relationship stage_contracts.md §4
 * has to the interface headers. It is deliberately self-contained (no stage
 * interface includes) so it can join the exported plugin surface in M2.1
 * without dragging node internals along.
 */

/**
 * Thrown by `StagePlugin::Init` (and `StageInit::Require`) when a stage cannot
 * initialise: a required parameter is missing or malformed, or a resource the
 * implementation needs is unavailable. The host treats it as fatal for the
 * pipeline bring-up — there is no silent fallback to another stage.
 */
class StageInitError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/**
 * Everything a stage receives at initialisation — the plugin-path equivalent of
 * the constructor arguments the concrete stages take today.
 *
 * `model` and `state` are clones the stage takes ownership of, exactly like the
 * current constructor injection (stage_contracts.md §3). The host calls `Init`
 * only after the first `/quad_state` has been received, so both are valid
 * snapshots — the same guarantee construction has today.
 *
 * `params` carries the host node's parameters as a flat name → value map. The
 * keys are the *same* key-space `SetParameter` uses at runtime (stage_contracts.md
 * §6): start-up configuration and runtime reconfiguration are one vocabulary,
 * not two. The host passes its full parameter set; a stage reads the keys it
 * documents and ignores the rest. Defaults for absent keys live in the stage,
 * not in the host.
 */
struct StageInit {
  std::unique_ptr<ModelInterface> model;
  std::unique_ptr<StateInterface> state;
  std::map<std::string, rclcpp::ParameterValue> params;

  /**
   * Looks up a key the stage cannot start without. Prefer this over `params.at`:
   * the error names the key, which is the "clear runtime error" issue #7 requires
   * from the load path.
   *
   * @param key the full parameter name, e.g. "mpc_alpha"
   * @return the value stored for the key
   * @throws StageInitError if the key is absent
   */
  const rclcpp::ParameterValue& Require(const std::string& key) const {
    const auto it = params.find(key);
    if (it == params.end()) {
      throw StageInitError("missing required stage parameter '" + key + "'");
    }
    return it->second;
  }
};

/**
 * The pluginlib base class for a pipeline stage: the stage's frozen interface
 * plus the initialisation phase. One template covers all stages because the
 * lifecycle is identical; each stage registers its own instantiation as the
 * pluginlib base (e.g. `StagePlugin<GaitSequencerInterface>`,
 * `StagePlugin<WBCInterface<JointTorqueVelocityPositionCommands>>`).
 *
 * Lifecycle (plugin_lifecycle.md §2):
 *   1. The loader default-constructs the plugin. The constructor must be cheap
 *      and infallible — no parameter access, no solver setup, no allocation
 *      that can fail meaningfully.
 *   2. The host calls `Init` exactly once, before any method of the stage
 *      interface, outside the control loops. `Init` does everything the
 *      concrete constructors do today and throws `StageInitError` on failure.
 *   3. Normal operation: the Update* / Get* / SetParameter cycle of
 *      stage_contracts.md §4. `Init` is never called again — reconfiguration
 *      that `SetParameter` cannot express means a fresh instance.
 *   4. Destruction through this base pointer (virtual destructor inherited
 *      from the stage interface, gap G4).
 */
template <class StageInterface>
class StagePlugin : public StageInterface {
 public:
  /**
   * Second construction phase — see the lifecycle above.
   *
   * @param init model/state clones and the host's parameters; taken by value,
   *             the stage moves out what it keeps
   * @throws StageInitError when the stage cannot start (missing/invalid
   *         parameters, unavailable resources)
   */
  virtual void Init(StageInit init) = 0;
};
