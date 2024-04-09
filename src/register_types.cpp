#include "register_types.hpp"

#include "fdm.hpp"
#include "gdexample.hpp"

#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_fdm_module(ModuleInitializationLevel p_level) {
  if (p_level != MODULE_INITIALIZATION_LEVEL_EDITOR) {
    return;
  }

  ClassDB::register_class<GDExample>();
  ClassDB::register_class<FDM>();
}

void uninitialize_fdm_module(ModuleInitializationLevel p_level) {
  if (p_level != MODULE_INITIALIZATION_LEVEL_EDITOR) {
    return;
  }
}

extern "C" {
// Initialization.
GDExtensionBool GDE_EXPORT
fdm_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address,
                 const GDExtensionClassLibraryPtr p_library,
                 GDExtensionInitialization *r_initialization) {
  godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library,
                                                 r_initialization);

  init_obj.register_initializer(initialize_fdm_module);
  init_obj.register_terminator(uninitialize_fdm_module);
  init_obj.set_minimum_library_initialization_level(
      MODULE_INITIALIZATION_LEVEL_SCENE);

  return init_obj.init();
}
}
