#pragma once

#include <pluginlib/class_loader.hpp>
#include <rclcpp/parameter_value.hpp>

#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mit_controller/stage_plugin.hpp"

/**
 * The loader half of the stage plugin layer (issue #7, M2.2).
 *
 * mit_controller/stage_plugin.hpp specified how a stage comes to life once it
 * has been instantiated (`StagePlugin<Interface>` + the two-phase `Init`). This
 * header is the other half: how the host gets from a `type:` string in the YAML
 * to an initialised stage, once, for every stage base, with the same error
 * behaviour everywhere.
 *
 * The rule the whole file exists to enforce is issue #7's second acceptance
 * criterion: **a missing or invalid `type:` is a loud, named, fatal error — never
 * a silent fallback to some other stage.** A pipeline that quietly walks with the
 * wrong controller because a parameter was misspelled is the failure mode this
 * layer is here to make impossible.
 *
 * The full contract — API, error taxonomy, loader lifetime, the parameter naming
 * convention — is doc/modularity/stage_loading.md. Companions:
 * plugin_lifecycle.md (the `Init` contract this calls) and plugin_discovery.md
 * (the base-class-type strings mirrored below).
 */

/**
 * Thrown when the *loader* cannot produce a stage: the selection parameter is
 * absent, is not a string, is empty, names a class no library declares, or
 * pluginlib fails to `dlopen`/instantiate it.
 *
 * Deliberately distinct from `StageInitError`, which is the stage itself
 * refusing to start (plugin_lifecycle.md §1 rule 4). Both are fatal for pipeline
 * bring-up; keeping them separate lets the host — and a human reading a crash
 * log — tell "you asked for a stage that does not exist" apart from "the stage
 * you asked for could not configure itself".
 */
class StageLoadError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/**
 * The pluginlib base-class-type strings, one per stage base.
 *
 * These are *the schema* (doc/modularity/plugin_discovery.md §3): the same
 * strings appear in the `plugins/` description XMLs and, from M2.3, in every stock
 * wrapper's `PLUGINLIB_EXPORT_CLASS`. Spelling them once here means the host
 * (M2.4) and the tests share one definition with the XML instead of scattering
 * string literals; changing one still requires updating the XML, this header and
 * plugin_discovery.md in the same pull request.
 *
 * In XML attributes the `<` must be escaped (`StagePlugin&lt;GaitSequencerInterface&gt;`);
 * tinyxml2 unescapes before pluginlib compares against these strings, so the two
 * spellings do match — see stage_loading.md §5.
 */
namespace stage_plugin_bases {

inline constexpr char kGaitSequencer[] = "StagePlugin<GaitSequencerInterface>";
inline constexpr char kMPC[] = "StagePlugin<MPCInterface>";
inline constexpr char kSwingLegController[] = "StagePlugin<SwingLegControllerInterface>";
inline constexpr char kWBC[] = "StagePlugin<WBCInterface<JointTorqueVelocityPositionCommands>>";
// The Cartesian WBC instantiation, added in M2.3 (issue #8) so `inverse_dynamics`
// — which implements WBCInterface<CartesianCommands>, the ULab/ikin command type
// — can be a stock plugin like every other current algorithm. plugin_lifecycle.md
// §6 anticipated one WBC base per joint-command type; #13 (M3.2) collapses both
// WBC bases into one once WBCInterface stops being a template.
inline constexpr char kWBCCartesian[] = "StagePlugin<WBCInterface<CartesianCommands>>";
inline constexpr char kModelAdaptation[] = "StagePlugin<ModelAdaptationInterface>";
inline constexpr char kContactLogic[] = "StagePlugin<ContactLogicInterface>";

}  // namespace stage_plugin_bases

/**
 * The package whose ament index is searched for stage plugin descriptions.
 *
 * `controllers` owns the interfaces, so its resource name
 * (`controllers__pluginlib__plugin`) is the discovery point. pluginlib resolves
 * that resource across *all* installed packages, so an out-of-package plugin
 * (the M4 goal) is found as soon as it calls
 * `pluginlib_export_plugin_description_file(controllers …)` — no allowlist here
 * needs touching.
 */
inline constexpr char kStagePluginPackage[] = "controllers";

/**
 * One loader per stage base class: resolves a `type:` parameter to a declared
 * plugin, default-constructs it and runs the `Init` phase.
 *
 * Usage from the host (M2.4):
 *
 * ```cpp
 * StageLoader<GaitSequencerInterface> gs_loader_{stage_plugin_bases::kGaitSequencer};  // declared FIRST
 * StageLoader<GaitSequencerInterface>::PluginPtr gs_;                                  // destroyed FIRST
 * ...
 * gs_ = gs_loader_.Load("gs.type", MakeStageInit());
 * gs_->UpdateTarget(target);  // the frozen interface, unchanged
 * ```
 *
 * The stage member is a `PluginPtr`, not a `std::unique_ptr<StageInterface>`:
 * pluginlib's deleter is part of the pointer's *type*, so moving into a plain
 * `unique_ptr` would silently swap in the default deleter. Nothing about how the
 * host *uses* the stage changes — `StagePlugin<I>` is an `I`, so every call site
 * stays the frozen interface (stage_contracts.md §4).
 *
 * **Declaration order is load-bearing.** Destroying a `pluginlib::ClassLoader`
 * unloads the library, and any surviving instance is then a dangling vtable
 * (plugin_lifecycle.md §5). Members are destroyed in reverse declaration order,
 * so the loader must be declared *before* the stage pointers it produced. The
 * class is non-copyable and non-movable so that this relationship cannot be
 * broken accidentally by moving a loader out from under its instances.
 *
 * @tparam StageInterface the frozen stage interface, e.g. `GaitSequencerInterface`.
 *         The pluginlib base is `StagePlugin<StageInterface>`.
 */
template <class StageInterface>
class StageLoader {
 public:
  /** The pluginlib base class this loader creates. */
  using Plugin = StagePlugin<StageInterface>;

  /**
   * Owning pointer carrying pluginlib's own deleter, which keeps the loaded
   * library's reference count straight. The host must hold *this* type for the
   * instance's whole life: moving into a plain `std::unique_ptr<StageInterface>`
   * would substitute the default deleter and destroy the stage outside
   * pluginlib's bookkeeping.
   */
  using PluginPtr = pluginlib::UniquePtr<Plugin>;

  /**
   * @param base_class_type one of the `stage_plugin_bases::k*` strings. Passed
   *        rather than derived from `StageInterface` because C++ has no portable
   *        type name to match the XML against — the string *is* the contract.
   * @param plugin_xml_paths optional explicit description files, bypassing ament
   *        index discovery. Empty in production (discovery is the point); used by
   *        the tests to load a description file that is deliberately not exported.
   * @throws StageLoadError if pluginlib cannot construct the underlying loader
   *         (package not found, malformed XML).
   */
  explicit StageLoader(std::string base_class_type, std::vector<std::string> plugin_xml_paths = {})
      : base_class_type_(std::move(base_class_type)) {
    try {
      class_loader_ = std::make_unique<pluginlib::ClassLoader<Plugin>>(
          kStagePluginPackage, base_class_type_, "plugin", std::move(plugin_xml_paths));
    } catch (const pluginlib::PluginlibException& error) {
      throw StageLoadError("could not create the plugin loader for stage base '" + base_class_type_
                           + "': " + error.what());
    }
  }

  StageLoader(const StageLoader&) = delete;
  StageLoader& operator=(const StageLoader&) = delete;
  StageLoader(StageLoader&&) = delete;
  StageLoader& operator=(StageLoader&&) = delete;

  /**
   * The full load path: resolve `type_key` out of the host's parameters, create
   * the plugin, hand it the initialisation context.
   *
   * Every failure below is fatal and names what went wrong; none of them
   * substitutes a different stage.
   *
   * 1. `init.params` must carry `type_key` → else `StageLoadError`.
   * 2. The value must be a non-empty string → else `StageLoadError`.
   * 3. The string must name a class declared for this base → else
   *    `StageLoadError` listing what *is* declared.
   * 4. pluginlib must load the library and construct the class → else
   *    `StageLoadError` carrying pluginlib's own message.
   * 5. `Init` must not throw → a `StageInitError` from the stage is re-thrown as
   *    a `StageInitError` naming the stage, so the type still tells the host
   *    which side failed.
   *
   * `init` is consumed: the parameter map is read for `type_key` first, then the
   * whole context is moved into the plugin (the stage keeps what it documents,
   * plugin_lifecycle.md §3).
   *
   * @param type_key the stage selection parameter, e.g. `"gs.type"`. Passed in
   *        rather than hard-coded: the key vocabulary belongs to M2.5 (#10), and
   *        renaming it must not touch this loader — see stage_loading.md §4.
   * @param init the initialisation context handed to `Init`
   * @return the initialised stage, ready for the Update / Get cycle of
   *         stage_contracts.md §4
   * @throws StageLoadError, StageInitError as above
   */
  PluginPtr Load(const std::string& type_key, StageInit init) {
    const std::string lookup_name = ResolveTypeKey(type_key, init);
    PluginPtr plugin = Create(lookup_name);
    try {
      plugin->Init(std::move(init));
    } catch (const StageInitError& error) {
      throw StageInitError("stage '" + lookup_name + "' (selected by '" + type_key + "', base '"
                           + base_class_type_ + "') failed to initialise: " + error.what());
    }
    return plugin;
  }

  /**
   * Create without initialising — the reconfiguration path of
   * plugin_lifecycle.md §5: build a fresh instance, `Init` it off-loop, swap the
   * pointer under the stage's lock, then destroy the old instance. Splitting
   * creation from `Init` lets the host do the expensive part before it takes the
   * lock.
   *
   * @param lookup_name the pluginlib class name (the resolved `type:` value)
   * @throws StageLoadError if the class is not declared or cannot be instantiated
   */
  PluginPtr Create(const std::string& lookup_name) {
    if (!class_loader_->isClassAvailable(lookup_name)) {
      throw StageLoadError("no stage plugin named '" + lookup_name + "' is declared for base '" + base_class_type_
                           + "'; " + DescribeDeclaredClasses());
    }
    try {
      PluginPtr plugin = class_loader_->createUniqueInstance(lookup_name);
      if (plugin == nullptr) {
        throw StageLoadError("plugin '" + lookup_name + "' (base '" + base_class_type_
                             + "') was declared but produced no instance");
      }
      return plugin;
    } catch (const pluginlib::PluginlibException& error) {
      throw StageLoadError("failed to load stage plugin '" + lookup_name + "' for base '" + base_class_type_
                           + "': " + error.what());
    }
  }

  /** The class names declared for this base, for diagnostics and tests. */
  std::vector<std::string> DeclaredClasses() const { return class_loader_->getDeclaredClasses(); }

  /** The base-class-type string this loader was constructed with. */
  const std::string& BaseClassType() const { return base_class_type_; }

 private:
  /**
   * Steps 1–2 of `Load`: the selection parameter must be present, a string, and
   * non-empty. No default is applied at any point — that is the "no silent
   * fallback" criterion, and it is why this returns by throwing rather than by
   * an empty string.
   */
  std::string ResolveTypeKey(const std::string& type_key, const StageInit& init) const {
    const auto it = init.params.find(type_key);
    if (it == init.params.end()) {
      throw StageLoadError("no stage selected: parameter '" + type_key + "' is not set (base '" + base_class_type_
                           + "'); " + DescribeDeclaredClasses());
    }
    if (it->second.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      throw StageLoadError("stage selection parameter '" + type_key + "' (base '" + base_class_type_
                           + "') must be a string, got " + rclcpp::to_string(it->second.get_type()));
    }
    const std::string lookup_name = it->second.get<std::string>();
    if (lookup_name.empty()) {
      throw StageLoadError("stage selection parameter '" + type_key + "' (base '" + base_class_type_
                           + "') is empty; " + DescribeDeclaredClasses());
    }
    return lookup_name;
  }

  /**
   * The "what could I have picked?" half of every error message. Naming the
   * declared classes turns a misspelled `type:` from a hunt through the XML into
   * a one-line diff, and "none are declared" points straight at a plugin library
   * that failed to install.
   */
  std::string DescribeDeclaredClasses() const {
    const std::vector<std::string> declared = class_loader_->getDeclaredClasses();
    if (declared.empty()) {
      return "no stage plugins are declared for this base (is the plugin library installed and its description "
             "XML exported?)";
    }
    std::ostringstream message;
    message << "declared stage plugins: ";
    for (size_t i = 0; i < declared.size(); ++i) {
      message << (i == 0 ? "'" : ", '") << declared[i] << "'";
    }
    return message.str();
  }

  std::string base_class_type_;
  // Held by pointer only so the loader stays assignable inside the constructor's
  // try/catch; it is created there and never replaced.
  std::unique_ptr<pluginlib::ClassLoader<Plugin>> class_loader_;
};
