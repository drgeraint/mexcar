/** \file DldCar_usage.hh
 *
 * contains the usage strings for the functions
 * which are exported via the Matlab and
 * GNU Octave dynamic link library interfaces.
 */

#ifndef DLDCAR_HH
#define DLDCAR_HH
#endif

#if (! defined (OCTAVE) & ! defined (MATLAB))
#error Define OCTAVE or MATLAB
#endif

#include <map>
#include <string>

/** serves to prevent errors from occurring if
 * usage_text is called by a function for which
 * a description of the usage has not been yet
 * been written.
 */
const char *
usage_text(std::string function,
	   std::string arguments,
	   std::string null)
{return "";};

/** returns information about the usage of functions. */
const char *
usage_text(std::string function)
{
  static std::map<std::string,std::string> arguments;
  static std::map<std::string,std::string> description;
  
  static bool is_initialised = false;
  if (! is_initialised) {
    is_initialised = true;

    arguments["get_A"] = "";
    description["get_A"] =
      "Return the state matrix of the linear\n"
      "velocity-based vehicle model.\n";

    arguments["get_B"] = "";
    description["get_B"] = 
      "Return the input matrix of the linear\n"
      "velocity-based vehicle model.\n";

    arguments["get_friction_coefficient"] = "";
    description["get_friction_coefficient"] =
      "Return the friction coefficient\n"
      "between the tyres and the road.\n";

    arguments["get_position"] = "";
    description["get_position"] =
      "Return the position of the vehicle\n"
      "(in the Earth axis system).\n";

    arguments["get_velocity"] = "";
    description["get_velocity"] =
      "Return the velocity of the vehicle\n"
      "(in its body axis system).\n";

    arguments["get_acceleration"] = "";
    description["get_acceleration"] =
      "Return the acceleration of the vehicle\n"
      "(in its body axis system).\n";

    arguments["get_wheel_lateral_forces"] = "";
    description["get_wheel_lateral_forces"] =
      "Return the lateral force of each wheel\n"
      "as a column vector [FL; FR; RL; RR].\n";

    arguments["get_wheel_lateral_speeds"] = "";
    description["get_wheel_lateral_speeds"] =
      "Return the longitudinal speed of each wheel\n"
      "as a column vector [FL; FR; RL; RR].\n";

    arguments["get_wheel_longitudinal_forces"] = "";
    description["get_wheel_longitudinal_forces"] =
      "Return the longitudinal force of each wheel\n"
      "as a column vector [FL; FR; RL; RR].\n";

    arguments["get_wheel_longitudinal_speeds"] = "";
    description["get_wheel_longitudinal_speeds"] =
      "Return the longitudinal speed of each wheel\n"
      "as a column vector [FL; FR; RL; RR].\n";

    arguments["get_wheel_slip_angles"] = "";
    description["get_wheel_slip_angles"] =
      "Return the slip angle of each wheel\n"
      "as a column vector [FL; FR; RL; RR].\n";

    arguments["get_wheel_steering_angles"] = "";
    description["get_wheel_steering_angles"] =
      "Return the steering angle of each wheel\n"
      "as a column vector [FL; FR; RL; RR].\n";

    arguments["get_wheel_vertical_forces"] = "";
    description["get_wheel_vertical_forces"] =
      "Return the vertical load on each wheel\n"
      "as a column vector [FL; FR; RL; RR].\n";

    arguments["integrate_euler_linear"] = "DT, N";
    description["integrate_euler_linear"] =
      "Perform N steps of Euler integration with\n"
      "timestep DT seconds using the derived linear\n"
      "model of the vehicle.\n";

    arguments["integrate_euler_nonlinear"] = "DT, N";
    description["integrate_euler_nonlinear"] =
      "Perform N steps of Euler integration with\n"
      "timestep DT seconds using the underlying\n"
      "nonlinear model of the vehicle.\n";

    arguments["set_acceleration"] = "A";
    description["set_acceleration"] =
      "Set the vehicle acceleration with the\n"
      "column vector A [longitudinal; lateral; yaw]\n"
      "(in the body axis system).\n";

    arguments["set_friction_coefficient"] = "mu";
    description["set_friction_coefficient"] =
      "Set the friction coefficient\n"
      "between the tyres and the road.\n";

    arguments["set_velocity"] = "V";
    description["set_velocity"] =
      "Set the vehicle velocity with the\n"
      "column vector V [longitudinal; lateral; yaw]\n"
      "(in the body axis system).\n";

    arguments["set_wheel_longitudinal_forces"] = "F";
    description["set_wheel_longitudinal_forces"] =
      "Set the longitudinal tyre forces with the\n"
      "column vector F [FL; FR; RL; RR]\n"
      "(in the wheel axis systems).\n";

    arguments["set_wheel_slip_angles"] = "ALPHA";
    description["set_wheel_slip_angles"] =
      "Set the slip angles (radians) with the\n"
      "column vector ALPHA [FL; FR; RL; RR].\n";

    arguments["set_wheel_speeds"] = "VX";
    description["set_wheel_speeds"] =
      "Set the wheel speeds (m/s) with the\n"
      "column vector VX [FL; FR; RL; RR]\n"
      "(in the wheel axis systems).\n";      

    arguments["set_wheel_steering_angle"] = "DELTA";
    description["set_wheel_steering_angle"] =
      "Set the steering angle of the front wheels\n"
      "to DELTA radians.\n";

    arguments["set_wheel_vertical_forces"] = "F";
    description["set_wheel_vertical_forces"] =
      "Set the vertical wheel loads with the\n"
      "column vector F [FL; FR; RL; RR]\n"
      "(in the wheel axis systems).\n";

    arguments["write_parameters"] = "";
    description["write_parameters"] = 
      "Writes internal parameters.\n"
      "If no car is initialised,\n"
      " a new one is created.\n";
  }

  std::string s("");
#if OCTAVE
  s += "OctCar_" + function + "(" + arguments[function] + ")\n";
#endif
#if MATLAB
  s += "MexCar('" + function + "'";
  if (arguments[function] != "") {
    s += ", " + arguments[function];
  }
  s += ")\n";
#endif
  s = s + "\n" + description[function];
  return s.c_str();
}

