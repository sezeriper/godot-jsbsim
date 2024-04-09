#include "fdm.hpp"
#include "utils.hpp"

#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGFCS.h>
#include <models/FGAuxiliary.h>


#include <cstdio>

#include <gdextension_interface.h>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/global_constants.hpp>
#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

void FDM::_bind_methods() {
  // model path
  ClassDB::bind_method(D_METHOD("get_model_name"), &FDM::get_model_name);
  ClassDB::bind_method(D_METHOD("set_model_name", "p_model_name"),
                       &FDM::set_model_name);
  ClassDB::add_property("FDM", PropertyInfo(Variant::STRING, "model_name"),
                        "set_model_name", "get_model_name");

  // aileron input
  ClassDB::bind_method(D_METHOD("get_input_aileron"), &FDM::get_input_aileron);
  ClassDB::bind_method(D_METHOD("set_input_aileron", "p_input_aileron"),
                       &FDM::set_input_aileron);
  ClassDB::add_property("FDM", PropertyInfo(Variant::FLOAT, "aileron", PROPERTY_HINT_RANGE, "-1,1,0.01"),
                        "set_input_aileron", "get_input_aileron");

  // elevator input
  ClassDB::bind_method(D_METHOD("get_input_elevator"), &FDM::get_input_elevator);
  ClassDB::bind_method(D_METHOD("set_input_elevator", "p_input_elevator"),
                       &FDM::set_input_elevator);
  ClassDB::add_property("FDM", PropertyInfo(Variant::FLOAT, "elevator", PROPERTY_HINT_RANGE, "-1,1,0.01"),
                        "set_input_elevator", "get_input_elevator");

  // rudder input
  ClassDB::bind_method(D_METHOD("get_input_rudder"), &FDM::get_input_rudder);
  ClassDB::bind_method(D_METHOD("set_input_rudder", "p_input_rudder"),
                       &FDM::set_input_rudder);
  ClassDB::add_property("FDM", PropertyInfo(Variant::FLOAT, "rudder", PROPERTY_HINT_RANGE, "-1,1,0.01"),
                        "set_input_rudder", "get_input_rudder");

  // throttle input
  ClassDB::bind_method(D_METHOD("get_input_throttle"), &FDM::get_input_throttle);
  ClassDB::bind_method(D_METHOD("set_input_throttle", "p_input_throttle"),
                       &FDM::set_input_throttle);
  ClassDB::add_property("FDM", PropertyInfo(Variant::FLOAT, "throttle", PROPERTY_HINT_RANGE, "0,1,0.001"),
                        "set_input_throttle", "get_input_throttle");
  
  // smoothing amount
  ClassDB::bind_method(D_METHOD("get_responsiveness"), &FDM::get_responsiveness);
  ClassDB::bind_method(D_METHOD("set_responsiveness", "p_responsiveness"),
                       &FDM::set_responsiveness);
  ClassDB::add_property("FDM", PropertyInfo(Variant::FLOAT, "responsiveness", PROPERTY_HINT_RANGE, "0,100,0.1"),
                        "set_responsiveness", "get_responsiveness");
}

FDM::FDM() : fdm_exec(nullptr), model_name(""), initialized(false), responsiveness(20.0) {}

void FDM::_ready() {
  if (Engine::get_singleton()->is_editor_hint()) {
    return;
  }

  if (model_name.is_empty()) {
    UtilityFunctions::print("Model path not found");
    return;
  }

  UtilityFunctions::print("Model path is: ", model_name);
  UtilityFunctions::print("Global model path is: ", utils::jsb_path_from_gd_string(model_name).c_str());

  // set custom path for JSBSim data
  fdm_exec = std::make_unique<JSBSim::FGFDMExec>();
  fdm_exec->SetRootDir(utils::jsb_path_from_gd_string("res://jsbsim_data"));
  fdm_exec->SetAircraftPath(SGPath("aircraft"));
  fdm_exec->SetEnginePath(SGPath("engine"));
  fdm_exec->SetSystemsPath(SGPath("systems"));
  double physics_ticks = static_cast<double>(Engine::get_singleton()->get_physics_ticks_per_second());
  fdm_exec->Setdt(1.0 / physics_ticks);

  bool result = fdm_exec->LoadModel(
    SGPath("aircraft"),
    SGPath("engine"),
    SGPath("systems"),
    model_name.utf8().get_data());

  if (!result) {
    UtilityFunctions::print("Failed to load model");
    return;
  }

  fdm_exec->GetIC()->Load(SGPath("reset00.xml"));
  fdm_exec->RunIC();

  copy_from_jsbsim();
  initial_location = get_aircraft_location();
  initialized = true;
}

FDM::~FDM() {}

void FDM::_physics_process(double delta) {
  if (Engine::get_singleton()->is_editor_hint()) {
    return;
  }

  if (!initialized) {
    return;
  }

  copy_to_jsbsim(delta);
  fdm_exec->Run();
  copy_from_jsbsim();
}

void FDM::copy_to_jsbsim(double delta) {
  smooth_aileron = Math::lerp(smooth_aileron, input_aileron, delta * responsiveness);
  smooth_elevator = Math::lerp(smooth_elevator, input_elevator, delta * responsiveness);
  smooth_rudder = Math::lerp(smooth_rudder, input_rudder, delta * responsiveness);
  smooth_throttle = Math::lerp(smooth_throttle, input_throttle, delta * responsiveness);

  fdm_exec->GetFCS()->SetDaCmd(smooth_aileron);
  fdm_exec->GetFCS()->SetDeCmd(smooth_elevator);
  fdm_exec->GetFCS()->SetDrCmd(smooth_rudder);
  fdm_exec->GetFCS()->SetThrottleCmd(-1, smooth_throttle);

  // UtilityFunctions::print("Aileron: ", input_aileron);
  // UtilityFunctions::print("Elevator: ", input_elevator);
  // UtilityFunctions::print("Rudder: ", input_rudder);
  // UtilityFunctions::print("Throttle: ", input_throttle);
}

void FDM::copy_from_jsbsim() {
  // double x = 0.0;
  // double y = 0.0;
  // double z = 0.0;
  double x = fdm_exec->GetPropagate()->GetEuler(2);
  double y = fdm_exec->GetPropagate()->GetEuler(3);
  double z = fdm_exec->GetPropagate()->GetEuler(1);
  set_rotation(Vector3(x, y, z));

  // use initial location as starting point
  JSBSim::FGColumnVector3 location_dt = initial_location.LocationToLocal(get_aircraft_location());
  set_position(utils::godot_vec_from_jsb_vec(utils::feet_to_meters_vec(location_dt)));
}

JSBSim::FGLocation FDM::get_aircraft_location() const {
  return fdm_exec->GetPropagate()->GetLocation();
}

String FDM::get_model_name() const { return model_name; }
void FDM::set_model_name(const String &p_model_name) {
  model_name = p_model_name;
}

double FDM::get_input_aileron() const { return input_aileron; }
void FDM::set_input_aileron(double p_input_aileron) {
  input_aileron = p_input_aileron;
}

double FDM::get_input_elevator() const { return input_elevator; }
void FDM::set_input_elevator(double p_input_elevator) {
  input_elevator = p_input_elevator;
}

double FDM::get_input_rudder() const { return input_rudder; }
void FDM::set_input_rudder(double p_input_rudder) {
  input_rudder = p_input_rudder;
}

double FDM::get_input_throttle() const { return input_throttle; }
void FDM::set_input_throttle(double p_input_throttle) {
  input_throttle = p_input_throttle;
}

double FDM::get_responsiveness() const { return responsiveness; }
void FDM::set_responsiveness(double p_responsiveness) {
  responsiveness = p_responsiveness;
}