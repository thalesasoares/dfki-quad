// Parameter helpers shared by the M2.3 stock plugin wrappers (issue #8).
//
// The stock wrappers' Init bodies are the host factory code of
// mit_controller_node.cpp with `get_parameter(x).as_T()` replaced by a lookup
// against StageInit::params (plugin_lifecycle.md §3-4). Two lookup shapes recur:
//
//   * a key the stage cannot start without  -> Require<T> (StageInit::Require)
//   * an optional key whose default lives in the stage, not the host -> GetOr<T>
//
// Centralising them keeps every wrapper's Init a straight transcription of the
// original factory block and keeps the "names the offending key" error contract
// (plugin_lifecycle.md §1 rule 4) in one place. This header is internal to the
// plugin libraries: it lives under src/plugins/ and is never installed.
#pragma once

#include <rclcpp/parameter_value.hpp>

#include <exception>
#include <string>

#include "mit_controller/stage_plugin.hpp"

namespace stock_plugins {

/**
 * Reads a required key. A missing key is reported by StageInit::Require; a
 * present-but-wrong-type value is converted here into a StageInitError that
 * still names the key, so the loader's fail-fast path (stage_loading.md) sees a
 * StageInitError either way rather than a raw rclcpp exception.
 */
template <class T>
T Require(const StageInit& init, const std::string& key) {
  const rclcpp::ParameterValue& value = init.Require(key);
  try {
    return value.get<T>();
  } catch (const std::exception& error) {
    throw StageInitError("required stage parameter '" + key + "' has the wrong type: " + error.what());
  }
}

/**
 * Reads an optional key, returning `fallback` when it is absent. This is the
 * "default lives in the stage" rule (plugin_lifecycle.md §3): the fallbacks
 * passed here mirror the code defaults the host used to carry in its
 * declare_parameter() calls.
 */
template <class T>
T GetOr(const StageInit& init, const std::string& key, T fallback) {
  const auto it = init.params.find(key);
  if (it == init.params.end()) {
    return fallback;
  }
  try {
    return it->second.get<T>();
  } catch (const std::exception& error) {
    throw StageInitError("optional stage parameter '" + key + "' has the wrong type: " + error.what());
  }
}

}  // namespace stock_plugins
