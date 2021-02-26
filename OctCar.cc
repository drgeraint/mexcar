// -*-c++-*-

/************************************************************
 * OctCar.cc
 *
 * Geraint Paul Bevan <geraint.bevan@gcu.ac.uk>
 * Initial Revision : <2005-06-24>
 * Latest Time-stamp: <2021/02/25 22:12:36 geraint>
 *
 ***********************************************************/

/** \file OctCar.cc implements an interface to the
 * LinearisableCar class.  An instance of the model can be
 * created and initialised by calling the function "OctCar"
 * from the octave prompt.
 *
 * Functions of LinearisableCar for which a handle has been
 * created can then be called by appending the function name
 * to the prefix OctCar_.
 *
 * For example, after initialising the model with
 * - octave> OctCar();
 * the state matrix can be obtained by:
 * - octave> OctCar_get_A();
 *
 * A list of available functions can be obtained by typing
 * OctCar and pressing the tab key.
 */

/** \file MexCar.m
 * implements a simple wrapper so that the OctCar
 * functions defined in OctCar.cc can be called
 * with the same names and arguments as the functions
 * defined in MexCar.cc.
 * Thus common scripts can be written to work identically
 * in either GNU Octave or Matlab.
 *
 * Example: to call the function OctCar_get_B() using the
 * Mex-style interface, type MexCar('get_B').
 */

#include <octave/oct.h>
#include <LinearisableCar.hh>
#include <DldCar_usage.hh>

static LinearisableCar *car;

/// function list.
/**
 * Calling this function presents a list of functions which
 * may be called from the Octave interpreter after the
 * LinearisableCar has been initialised.
 */
const char *usage_OctCar =
"OctCar()\n"
"\n"
"An Octave interface to a linearisable car model:\n"
"Initialises a velocity-based model of a car\n"
"and displays the vehicle parameters.\n"
"\n"
"Related functions:\n"
" OctCar_get_A\n"
" OctCar_get_B\n"
" OctCar_get_W\n"
" OctCar_get_position\n"
" OctCar_get_velocity\n"
" OctCar_get_acceleration\n"
" OctCar_get_wheel_friction_coefficients\n"
" OctCar_get_wheel_lateral_forces\n"
" OctCar_get_wheel_lateral_speeds\n"
" OctCar_get_wheel_longitudinal_forces\n"
" OctCar_get_wheel_longitudinal_speeds\n"
" OctCar_get_wheel_slip_angles\n"
" OctCar_get_wheel_steering_angles\n"
" OctCar_get_wheel_vertical_forces\n"
" OctCar_integrate_euler_nonlinear\n"
" OctCar_integrate_euler_linear\n"
" OctCar_set_acceleration\n"
" OctCar_set_velocity\n"
" OctCar_set_wheel_friction_coefficients\n"
" OctCar_set_wheel_longitudinal_forces\n"
" OctCar_set_wheel_slip_angles\n"
" OctCar_set_wheel_speeds\n"
" OctCar_set_wheel_steering_angle\n"
" OctCar_set_wheel_vertical_forces\n"
" OctCar_write_parameters\n";

/** initialises the LinearisableCar. */
DEFUN_DLD(OctCar, args, ,
	  usage_OctCar)
{
  octave_value_list retval;
  if (args.length() != 0) {
    print_usage(usage_OctCar);
  }
  if (car != NULL) {
    delete(car);
  }
  car = new LinearisableCar();
  car->write_parameters();
  retval(0) = true;
  return retval;
}

DEFUN_DLD(OctCar_get_A, args, ,
	  usage_text("get_A"))
{
  octave_value_list retval;
  if (args.length() != 0) {
    print_usage(usage_text("get_A"));
  }
  double A_array[LinearisableCar::NX*LinearisableCar::NX];
  car->get_A(A_array);
  Matrix m(LinearisableCar::NX,LinearisableCar::NX);
  int counter = 0;
  for (int i = 0; i < LinearisableCar::NX; i++) {
    for (int j = 0; j < LinearisableCar::NX; j++) {
      m(j,i) = A_array[counter++];
    }
  }
  retval(0) = m;
  return retval;
}

DEFUN_DLD(OctCar_get_B, args, ,
	  usage_text("get_B"))
{
  octave_value_list retval;
  if (args.length() != 0) {
    print_usage(usage_text("get_B"));
  }
  double B_array[LinearisableCar::NX*LinearisableCar::NU];
  car->get_B(B_array);
  Matrix m(LinearisableCar::NX,LinearisableCar::NU);
  int counter = 0;
  for (int i = 0; i < LinearisableCar::NU; i++) {
    for (int j = 0; j < LinearisableCar::NX; j++) {
      m(j,i) = B_array[counter++];
    }
  }
  retval(0) = m;
  return retval;
}

DEFUN_DLD(OctCar_get_W, args, ,
	  usage_text("get_W"))
{
  octave_value_list retval;
  if (args.length() != 0) {
    print_usage(usage_text("get_W"));
  }
  double W_array[LinearisableCar::NX*LinearisableCar::NW];
  car->get_W(W_array);
  Matrix m(LinearisableCar::NX,LinearisableCar::NW);
  int counter = 0;
  for (int i = 0; i < LinearisableCar::NW; i++) {
    for (int j = 0; j < LinearisableCar::NX; j++) {
      m(j,i) = W_array[counter++];
    }
  }
  retval(0) = m;
  return retval;
}

DEFUN_DLD(OctCar_get_position, args, ,
	  usage_text("get_position"))
{
  octave_value_list retval;
  if (args.length() != 0) {
    print_usage(usage_text("get_position"));
  }
  ColumnVector p(3);
  Car::axis position;
  position = car->get_position();
  p(0) = position.X;
  p(1) = position.Y;
  p(2) = position.Psi;
  retval(0) = p;
  return retval;
}

DEFUN_DLD(OctCar_get_velocity, args, ,
	  usage_text("get_velocity"))
{
  octave_value_list retval;
  if (args.length() != 0) {
    print_usage(usage_text("get_velocity"));
  }
  ColumnVector v(3);
  Car::axis velocity;
  velocity = car->get_velocity();
  v(0) = velocity.X;
  v(1) = velocity.Y;
  v(2) = velocity.Psi;
  retval(0) = v;
  return retval;
}

DEFUN_DLD(OctCar_get_acceleration, args, ,
	  usage_text("get_acceleration"))
{
  octave_value_list retval;
  if (args.length() != 0) {
    print_usage(usage_text("get_acceleration"));
  }
  ColumnVector a(3);
  Car::axis acceleration;
  acceleration = car->get_acceleration();
  a(0) = acceleration.X;
  a(1) = acceleration.Y;
  a(2) = acceleration.Psi;
  retval(0) = a;
  return retval;
}

DEFUN_DLD(OctCar_get_wheel_friction_coefficients, args, ,
	  usage_text("get_wheel_friction_coefficients"))
{
  octave_value_list retval;
  if (args.length() != 0) {
    print_usage(usage_text("get_wheel_friction_coefficient"));
  }
  ColumnVector mu(4);
  mu(0) = car->Car::get_wheel_friction_coefficient(Car::FL);
  mu(1) = car->Car::get_wheel_friction_coefficient(Car::FR);
  mu(2) = car->Car::get_wheel_friction_coefficient(Car::RL);
  mu(3) = car->Car::get_wheel_friction_coefficient(Car::RR);
  retval(0) = mu;
  return retval;
}

DEFUN_DLD(OctCar_get_wheel_lateral_forces, args, ,
	  usage_text("get_wheel_lateral_forces"))
{
  octave_value_list retval;
  if (args.length() != 0) {
    print_usage(usage_text("get_wheel_lateral_forces"));
  }
  ColumnVector fy(4);
  fy(0) = car->get_wheel_lateral_force(Car::FL);
  fy(1) = car->get_wheel_lateral_force(Car::FR);
  fy(2) = car->get_wheel_lateral_force(Car::RL);
  fy(3) = car->get_wheel_lateral_force(Car::RR);
  retval(0) = fy;
  return retval;
}

DEFUN_DLD(OctCar_get_wheel_lateral_speeds, args, ,
	  usage_text("get_wheel_lateral_speeds"))
{
  octave_value_list retval;
  if (args.length() != 0) {
    print_usage(usage_text("get_wheel_lateral_forces"));
  }
  ColumnVector vy(4);
  vy(0) = car->get_wheel_lateral_speed(Car::FL);
  vy(1) = car->get_wheel_lateral_speed(Car::FR);
  vy(2) = car->get_wheel_lateral_speed(Car::RL);
  vy(3) = car->get_wheel_lateral_speed(Car::RR);
  retval(0) = vy;
  return retval;
}

DEFUN_DLD(OctCar_get_wheel_longitudinal_forces, args, ,
	  usage_text("get_wheel_longitudinal_forces"))
{
  octave_value_list retval;
  if (args.length() != 0) {
    print_usage(usage_text("get_wheel_longitudinal_forces"));
  }
  ColumnVector fx(4);
  fx(0) = car->get_wheel_longitudinal_force(Car::FL);
  fx(1) = car->get_wheel_longitudinal_force(Car::FR);
  fx(2) = car->get_wheel_longitudinal_force(Car::RL);
  fx(3) = car->get_wheel_longitudinal_force(Car::RR);
  retval(0) = fx;
  return retval;
}

DEFUN_DLD(OctCar_get_wheel_longitudinal_speeds, args, ,
	  usage_text("get_wheel_longitudinal_speeds"))
{
  octave_value_list retval;
  if (args.length() != 0) {
    print_usage(usage_text("get_wheel_longitudinal_speeds"));
  }
  ColumnVector vx(4);
  vx(0) = car->get_wheel_longitudinal_speed(Car::FL);
  vx(1) = car->get_wheel_longitudinal_speed(Car::FR);
  vx(2) = car->get_wheel_longitudinal_speed(Car::RL);
  vx(3) = car->get_wheel_longitudinal_speed(Car::RR);
  retval(0) = vx;
  return retval;
}

DEFUN_DLD(OctCar_get_wheel_slip_angles, args, ,
	  usage_text("get_wheel_slip_angles"))
{
  octave_value_list retval;
  if (args.length() != 0) {
    print_usage(usage_text("get_wheel_slip_angles"));
  }
  ColumnVector alpha(4);
  alpha(0) = car->get_wheel_slip_angle(Car::FL);
  alpha(1) = car->get_wheel_slip_angle(Car::FR);
  alpha(2) = car->get_wheel_slip_angle(Car::RL);
  alpha(3) = car->get_wheel_slip_angle(Car::RR);
  retval(0) = alpha;
  return retval;
}

DEFUN_DLD(OctCar_get_wheel_steering_angles, args, ,
	  usage_text("get_wheel_steering_angles"))
{
  octave_value_list retval;
  if (args.length() != 0) {
    print_usage(usage_text("get_wheel_steering_angles"));
  }
  ColumnVector delta(4);
  delta(0) = car->get_wheel_steering_angle(Car::FL);
  delta(1) = car->get_wheel_steering_angle(Car::FR);
  delta(2) = car->get_wheel_steering_angle(Car::RL);
  delta(3) = car->get_wheel_steering_angle(Car::RR);
  retval(0) = delta;
  return retval;
}

DEFUN_DLD(OctCar_get_wheel_vertical_forces, args, ,
	  usage_text("get_wheel_vertical_forces"))
{
  octave_value_list retval;
  if (args.length() != 0) {
    print_usage(usage_text("get_wheel_vertical_forces"));
  }
  ColumnVector fz(4);
  fz(0) = car->get_wheel_vertical_force(Car::FL);
  fz(1) = car->get_wheel_vertical_force(Car::FR);
  fz(2) = car->get_wheel_vertical_force(Car::RL);
  fz(3) = car->get_wheel_vertical_force(Car::RR);
  retval(0) = fz;
  return retval;
}

DEFUN_DLD(OctCar_integrate_euler_linear, args, ,
	  usage_text("integrate_euler_linear"))
{
  octave_value_list retval;
  if (args.length() != 2) {
    print_usage(usage_text("integrate_euler_linear"));
    error("incorrect number of arguments");
    retval(0) = false;
    return retval;
  }
  double dt = args(0).double_value();
  int n = static_cast<int>(args(1).double_value());
  for (int i = 0; i < n; i++) {
    car->integrate_euler(dt);
  }
  retval(0) = true;
  return retval;
}

DEFUN_DLD(OctCar_integrate_euler_nonlinear, args, ,
	  usage_text("integrate_euler_nonlinear"))
{
  octave_value_list retval;
  if (args.length() != 2) {
    print_usage(usage_text("integrate_euler_nonlinear"));
    error("incorrect number of arguments");
    retval(0) = false;
    return retval;
  }
  double dt = args(0).double_value();
  int n = static_cast<int>(args(1).double_value());
  for (int i = 0; i < n; i++) {
    car->Car::integrate_euler(dt);
  }
  retval(0) = true;
  return retval;
}

DEFUN_DLD(OctCar_set_accleration, args, ,
	  usage_text("set_acceleration"))
{
  octave_value_list retval;
  if (args.length() != 1) {
    print_usage(usage_text("set_acceleration"));
    error("incorrect number of arguments");
    retval(0) = false;
    return retval;
  }
  ColumnVector a(3, 0.0);
  a = args(0).column_vector_value();
  Car::axis acceleration;
  acceleration.X = a(0);
  acceleration.Y = a(1);
  acceleration.Psi = a(2);
  car->set_acceleration(acceleration);
  retval(0) = true;
  return retval;
}

DEFUN_DLD(OctCar_set_friction_coefficient, args, ,
	  usage_text("set_friction_coefficient"))
{
  octave_value_list retval;
  if (args.length() != 1) {
    print_usage(usage_text("set_friction_coefficient"));
    error("incorrect number of arguments");
    retval(0) = false;
    return retval;
  }
  double mu = args(0).double_value();
  car->set_friction_coefficient(mu);
  retval(0) = true;
  return retval;
}

DEFUN_DLD(OctCar_set_velocity, args, ,
	  usage_text("set_velocity"))
{
  octave_value_list retval;
  if (args.length() != 1) {
    print_usage(usage_text("set_velocity"));
    error("incorrect number of arguments");
    retval(0) = false;
    return retval;
  }
  ColumnVector v(3, 0.0);
  v = args(0).column_vector_value();
  Car::axis velocity;
  velocity.X = v(0);
  velocity.Y = v(1);
  velocity.Psi = v(2);
  car->set_velocity(velocity);
  retval(0) = true;
  return retval;
}

DEFUN_DLD(OctCar_set_wheel_friction_coefficients, args, ,
	  usage_text("set_wheel_friction_coefficients"))
{
  octave_value_list retval;
  if (args.length() != 1) {
    print_usage(usage_text("set_wheel_friction_coefficients"));
    error("incorrect number of arguments");
    retval(0) = false;
    return retval;
  }
  ColumnVector mu(4, 1.0);
  mu = args(0).column_vector_value();
  car->Car::set_wheel_friction_coefficient(Car::FL, mu(0));
  car->Car::set_wheel_friction_coefficient(Car::FR, mu(1));
  car->Car::set_wheel_friction_coefficient(Car::RL, mu(2));
  car->Car::set_wheel_friction_coefficient(Car::RR, mu(3));
  retval(0) = true;
  return retval;
}
  
DEFUN_DLD(OctCar_set_wheel_lateral_forces, args, ,
	  usage_text("set_wheel_lateral_forces"))
{
  octave_value_list retval;
  if (args.length() != 1) {
    print_usage(usage_text("set_wheel_lateral_forces"));
    error("incorrect number of arguments");
    retval(0) = false;
    return retval;
  }
  ColumnVector f(4, 0.0);
  f = args(0).column_vector_value();
  car->set_wheel_lateral_force(Car::FL, f(0));
  car->set_wheel_lateral_force(Car::FR, f(1));
  car->set_wheel_lateral_force(Car::RL, f(2));
  car->set_wheel_lateral_force(Car::RR, f(3));
  retval(0) = true;
  return retval;
}

DEFUN_DLD(OctCar_set_wheel_longitudinal_forces, args, ,
	  usage_text("set_wheel_longitudinal_forces"))
{
  octave_value_list retval;
  if (args.length() != 1) {
    print_usage(usage_text("set_wheel_longitudinal_forces"));
    error("incorrect number of arguments");
    retval(0) = false;
    return retval;
  }
  ColumnVector f(4, 0.0);
  f = args(0).column_vector_value();
  car->set_wheel_longitudinal_force(Car::FL, f(0));
  car->set_wheel_longitudinal_force(Car::FR, f(1));
  car->set_wheel_longitudinal_force(Car::RL, f(2));
  car->set_wheel_longitudinal_force(Car::RR, f(3));
  retval(0) = true;
  return retval;
}

DEFUN_DLD(OctCar_set_wheel_slip_angles, args, ,
	  usage_text("set_wheel_slip_angles"))
{
  octave_value_list retval;
  if (args.length() != 1) {
    print_usage(usage_text("set_wheel_slip_angles"));
    error("incorrect number of arguments");
    retval(0) = false;
    return retval;
  }
  ColumnVector alpha(4, 0.0);
  alpha = args(0).column_vector_value();
  car->set_wheel_slip_angle(Car::FL, alpha(0));
  car->set_wheel_slip_angle(Car::FR, alpha(1));
  car->set_wheel_slip_angle(Car::RL, alpha(2));
  car->set_wheel_slip_angle(Car::RR, alpha(3));
  retval(0) = true;
  return retval;
}

DEFUN_DLD(OctCar_set_wheel_speeds, args, ,
	  usage_text("set_wheel_speeds"))
{
  octave_value_list retval;
  if (args.length() != 1) {
    print_usage(usage_text("set_wheel_speeds"));
    //    error("incorrect number of arguments");
    retval(0) = false;
    return retval;
  }
  ColumnVector vx(4, 0.0);
  vx = args(0).column_vector_value();
  car->set_wheel_speed(Car::FL, vx(0));
  car->set_wheel_speed(Car::FR, vx(1));
  car->set_wheel_speed(Car::RL, vx(2));
  car->set_wheel_speed(Car::RR, vx(3));
  retval(0) = true;
  return retval;
}

DEFUN_DLD(OctCar_set_wheel_steering_angle, args, ,
	  usage_text("set_wheel_steering_angle"))
{
  octave_value_list retval;
  if (args.length() != 1) {
    print_usage(usage_text("set_wheel_steering_angle"));
    error("incorrect number of arguments");
    retval(0) = false;
    return retval;
  }
  double angle = args(0).double_value();
  car->set_wheel_steering_angle(angle);
  retval(0) = true;
  return retval;
}

DEFUN_DLD(OctCar_set_wheel_vertical_forces, args, ,
	  usage_text("set_wheel_vertical_forces"))
{
  octave_value_list retval;
  if (args.length() != 1) {
    print_usage(usage_text("set_wheel_vertical_forces"));
    error("incorrect number of arguments");
    retval(0) = false;
    return retval;
  }
  ColumnVector f(4, 0.0);
  f = args(0).column_vector_value();
  car->set_wheel_vertical_force(Car::FL, f(0));
  car->set_wheel_vertical_force(Car::FR, f(1));
  car->set_wheel_vertical_force(Car::RL, f(2));
  car->set_wheel_vertical_force(Car::RR, f(3));
  retval(0) = true;
  return retval;
}

DEFUN_DLD(OctCar_write_parameters, args, ,
	  usage_text("write_parameters"))
{
  octave_value_list retval;
  if (car == NULL) {
    car = new LinearisableCar();
  }
  car->write_parameters();
  retval(0) = true;
  return retval;
}
