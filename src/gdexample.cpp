#include "gdexample.hpp"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void GDExample::_bind_methods() {
  // amplitude property
  ClassDB::bind_method(D_METHOD("get_amplitude"), &GDExample::get_amplitude);
  ClassDB::bind_method(D_METHOD("set_amplitude", "p_amplitude"),
                       &GDExample::set_amplitude);
  ClassDB::add_property("GDExample", PropertyInfo(Variant::FLOAT, "amplitude"),
                        "set_amplitude", "get_amplitude");

  // speed property
  ClassDB::bind_method(D_METHOD("get_speed"), &GDExample::get_speed);
  ClassDB::bind_method(D_METHOD("set_speed", "p_speed"), &GDExample::set_speed);
  ClassDB::add_property(
      "GDExample",
      PropertyInfo(Variant::FLOAT, "speed", PROPERTY_HINT_RANGE, "0,20,0.01"),
      "set_speed", "get_speed");
}

GDExample::GDExample() {
  // Initialize any variables here.
  time_passed = 0.0;
  amplitude = 10.0;
  speed = 1.0;
}

GDExample::~GDExample() {
  // Add your cleanup here.
}

void GDExample::_process(double delta) {
  time_passed += speed * delta;

  Vector2 new_position =
      Vector2(amplitude + (amplitude * sin(time_passed * 2.0)),
              amplitude + (amplitude * cos(time_passed * 1.5)));

  set_position(new_position);
}

void GDExample::set_amplitude(const double p_amplitude) {
  amplitude = p_amplitude;
}

double GDExample::get_amplitude() const { return amplitude; }

void GDExample::set_speed(const double p_speed) { speed = p_speed; }

double GDExample::get_speed() const { return speed; }
