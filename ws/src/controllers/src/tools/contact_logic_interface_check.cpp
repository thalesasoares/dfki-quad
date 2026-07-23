// Compile-smoke check for the ContactLogicInterface stage contract (issue #4,
// M1.4).
//
// The interface is a header stub only in M1.4 — no in-package source includes it
// yet, and the host is unchanged (extraction is issue #12, M3.1). A header that
// nothing compiles rots silently, so this translation unit includes it and
// pins the shape the contract promises: an abstract base class with a virtual
// destructor, and event/output aggregates sized off pipeline_constants.hpp.
//
// Unlike src/tools/pipeline_types_surface_check.cpp, this target uses the
// package's normal include paths: ContactLogicInterface deliberately depends on
// the in-package (not yet exported) interface header
// swing_leg_controller_interface.hpp, so the restricted, export-only include
// path would be the wrong lens here. The interface header joins the exported
// plugin surface in M2.1 (issue #6); a fake-stub contract test that instantiates
// the interface supersedes this file in M1.5 (issue #5).

#include "mit_controller/contact_logic_interface.hpp"  // IWYU pragma: keep

#include <type_traits>

namespace {

// The stage is an abstract interface — protected default constructor, all
// methods pure virtual — so it cannot be instantiated and must be deleted safely
// through the base pointer the host will own it by (cf. gap G4, M1.1).
static_assert(std::is_abstract_v<ContactLogicInterface>);
static_assert(std::has_virtual_destructor_v<ContactLogicInterface>);
static_assert(!std::is_default_constructible_v<ContactLogicInterface>);

// The reconciled outputs and the diagnostic aggregates are per-leg, sized off
// pipeline_constants.hpp. A change to N_LEGS that is not mirrored here breaks the
// build, the same guard the pipeline type surface check uses.
static_assert(std::tuple_size_v<ContactLogicInterface::FootContacts> == N_LEGS);
static_assert(std::tuple_size_v<ContactLogicInterface::Wrenches> == N_LEGS);
static_assert(std::tuple_size_v<decltype(ContactLogicInterface::ContactEvents::early_contact_detected)> == N_LEGS);
static_assert(std::tuple_size_v<decltype(ContactLogicInterface::ContactEvents::lost_contact_detected)> == N_LEGS);
static_assert(
    std::tuple_size_v<decltype(ContactLogicInterface::ContactEvents::swing_scheduled_before_slc_started)> == N_LEGS);

}  // namespace
