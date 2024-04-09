#include "utils.hpp"
#include <FGJSBBase.h>

SGPath utils::jsb_path_from_gd_string(const String &p_path) {
    String path = ProjectSettings::get_singleton()->globalize_path(p_path);
    std::string path_str(path.utf8().get_data());
    return SGPath(path_str);
}

Vector3 utils::godot_vec_from_jsb_vec(const JSBSim::FGColumnVector3 &p_vec) {
    return Vector3(-p_vec(2), -p_vec(3), -p_vec(1));
}

JSBSim::FGColumnVector3 utils::feet_to_meters_vec(const JSBSim::FGColumnVector3 &p_vec) {
    return JSBSim::FGColumnVector3(
        JSBSim::FGJSBBase::FeetToMeters(p_vec(1)),
        JSBSim::FGJSBBase::FeetToMeters(p_vec(2)),
        JSBSim::FGJSBBase::FeetToMeters(p_vec(3)));
}