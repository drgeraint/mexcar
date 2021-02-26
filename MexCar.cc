// -*-c++-*-

/**********************************************************************
 * MexCar.cc
 *
 * Geraint Paul Bevan <geraint.bevan@gcu.ac.uk>
 * Initial Revision : <2005-08-10>
 * Latest Time-stamp: <2021/02/26 02:33:12 geraint>
 *
 ***********************************************************************/

/** \file MexCar.cc
 * implements an interface to the LinearisableCar class.
 * An instance of the model can be created
 * and initialised by calling the function
 * "MexCar" from the matlab prompt.
 *
 * Functions of LinearisableCar for which a handle has
 * been created can then be called by passing the
 * function name as an argument.
 *
 * For example, after initialising the model with
 * matlab> MexCar();
 * the state matrix can be obtained by:
 * matlab> MexCar('get_A');
 */

#include <iostream>
#include <string>
#include <map>

#include <mex.h>
#include <LinearisableCar.hh>
#include <DldCar_usage.hh>

#ifdef DEBUG
#define DEBUGPRINT printf
#else
#define DEBUGPRINT //
#endif

static LinearisableCar *car;

/// function list.
/**
 * Calling this function presents a list of functions which
 * may be called from the Matlab interpreter after the
 * LinearisableCar has been initialised.
 */
const char *usage_MexCar =
"MexCar()\n"
"\n"
"A Matlab interface to a linearisable car model:\n"
"Initialises a velocity-based model of a car\n"
"and displays the vehicle parameters.\n"
"\n"
"Related function:\n"
" MexCar_get_A\n"
" MexCar_get_B\n"
" MexCar_get_position\n"
" MexCar_get_velocity\n"
" MexCar_get_acceleration\n"
" MexCar_get_friction_coefficient\n"
" MexCar_get_wheel_lateral_forces\n"
" MexCar_get_wheel_lateral_speeds\n"
" MexCar_get_wheel_longitudinal_forces\n"
" MexCar_get_wheel_longitudinal_speeds\n"
" MexCar_get_wheel_slip_angles\n"
" MexCar_get_wheel_steering_angles\n"
" MexCar_get_wheel_vertical_forces\n"
" MexCar_integrate_euler_nonlinear\n"
" MexCar_integrate_euler_linear\n"
" MexCar_set_acceleration\n"
" MexCar_set_friction_coefficient\n"
" MexCar_set_velocity\n"
" MexCar_set_wheel_longitudinal_forces\n"
" MexCar_set_wheel_slip_angles\n"
" MexCar_set_wheel_speeds\n"
" MexCar_set_wheel_steering_angle\n"
" MexCar_set_wheel_vertical_forces\n"
" MexCar_write_parameters\n";

/** function calling routine */
void CallMexSubfunction(const char *subfunction,
			int nlhs, mxArray *plhs[],
			int nrhs, const mxArray *prhs[]);

/** initialises the LinearisableCar. */
void mexFunction(int nlhs, mxArray *plhs[],
		 int nrhs, const mxArray *prhs[])
{
  if (nrhs == 0) {
    if (car != NULL) {
      delete(car);
    }
    std::cout << usage_MexCar << std::endl;
    car = new LinearisableCar();
    car->write_parameters();
  } else {
    char buf[128];
    mxGetString(prhs[0], buf, sizeof(buf)-1);
    std::string s(buf);
    CallMexSubfunction(buf, nlhs, plhs, nrhs-1, prhs+1);
  }
  return;
}

void
MexCar_get_A(int nlhs, mxArray *plhs[],
	     int nrhs, const mxArray *prhs[])
{
  if (nrhs != 0) {
    mexWarnMsgTxt(usage_text("get_A"));
    mexWarnMsgTxt("ignoring extra arguments");
  }
  
  double *A_array;
  plhs[0] = mxCreateDoubleMatrix(LinearisableCar::NX,LinearisableCar::NX,mxREAL);
  A_array = mxGetPr(plhs[0]);
  car->get_A(A_array);
  return;
}

void
MexCar_get_B(int nlhs, mxArray *plhs[],
	     int nrhs, const mxArray *prhs[])
{
  if (nrhs != 0) {
    mexWarnMsgTxt(usage_text("get_B"));
    mexWarnMsgTxt("ignoring extra arguments");
  }
  
  double *B_array;
  plhs[0] = mxCreateDoubleMatrix(LinearisableCar::NX,LinearisableCar::NU,mxREAL);
  B_array = mxGetPr(plhs[0]);
  car->get_B(B_array);
  return;
}

void
MexCar_get_position(int nlhs, mxArray *plhs[],
		    int nrhs, const mxArray *prhs[])
{
  if (nrhs != 0) {
    mexWarnMsgTxt(usage_text("get_position"));
    mexWarnMsgTxt("ignoring extra arguments");
  }
  
  Car::axis position;
  position = car->get_position();

  double *P_array;
  plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
  P_array = mxGetPr(plhs[0]);
  P_array[0] = position.X;
  P_array[1] = position.Y;
  P_array[2] = position.Psi;
  return;
}

void
MexCar_get_velocity(int nlhs, mxArray *plhs[],
		    int nrhs, const mxArray *prhs[])
{
  if (nrhs != 0) {
    mexWarnMsgTxt(usage_text("get_velocity"));
    mexWarnMsgTxt("ignoring extra arguments");
  }
  
  Car::axis velocity;
  velocity = car->get_velocity();

  double *V_array;
  plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
  V_array = mxGetPr(plhs[0]);
  V_array[0] = velocity.X;
  V_array[1] = velocity.Y;
  V_array[2] = velocity.Psi;
  return;
}

void
MexCar_get_acceleration(int nlhs, mxArray *plhs[],
			int nrhs, const mxArray *prhs[])
{
  if (nrhs != 0) {
    mexWarnMsgTxt(usage_text("get_acceleration"));
    mexWarnMsgTxt("ignoring extra arguments");
  }
  
  Car::axis acceleration;
  acceleration = car->get_acceleration();

  double *A_array;
  plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
  A_array = mxGetPr(plhs[0]);
  A_array[0] = acceleration.X;
  A_array[1] = acceleration.Y;
  A_array[2] = acceleration.Psi;
  return;
}

void
MexCar_get_friction_coefficient(int nlhs, mxArray *plhs[],
				int nrhs, const mxArray *prhs[])
{
  if (nrhs != 0) {
    mexWarnMsgTxt(usage_text("get_friction_coefficient"));
    mexWarnMsgTxt("ignoring extra arguments");
  }
  
  double *mu;
  plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
  mu = mxGetPr(plhs[0]);
  mu[0] = car->get_friction_coefficient();;
  return;
}

void
MexCar_get_wheel_lateral_forces(int nlhs, mxArray *plhs[],
				int nrhs, const mxArray *prhs[])
{
  if (nrhs != 0) {
    mexWarnMsgTxt(usage_text("get_wheel_lateral_forces"));
    mexWarnMsgTxt("ignoring extra arguments");
  }

  double *Fy;
  plhs[0] = mxCreateDoubleMatrix(4,1,mxREAL);
  Fy = mxGetPr(plhs[0]);
  Fy[0] = car->get_wheel_lateral_force(Car::FL);
  Fy[1] = car->get_wheel_lateral_force(Car::FR);
  Fy[2] = car->get_wheel_lateral_force(Car::RL);
  Fy[3] = car->get_wheel_lateral_force(Car::RR);
  return;
}

void
MexCar_get_wheel_lateral_speeds(int nlhs, mxArray *plhs[],
				int nrhs, const mxArray *prhs[])
{
  if (nrhs != 0) {
    mexWarnMsgTxt(usage_text("get_wheel_lateral_speeds"));
    mexWarnMsgTxt("ignoring extra arguments");
  }

  double *vy;
  plhs[0] = mxCreateDoubleMatrix(4,1,mxREAL);
  vy = mxGetPr(plhs[0]);
  vy[0] = car->get_wheel_lateral_speed(Car::FL);
  vy[1] = car->get_wheel_lateral_speed(Car::FR);
  vy[2] = car->get_wheel_lateral_speed(Car::RL);
  vy[3] = car->get_wheel_lateral_speed(Car::RR);
  return;
}

void
MexCar_get_wheel_longitudinal_forces(int nlhs, mxArray *plhs[],
				     int nrhs, const mxArray *prhs[])
{
  if (nrhs != 0) {
    mexWarnMsgTxt(usage_text("get_wheel_longitudinal_forces"));
    mexWarnMsgTxt("ignoring extra arguments");
  }

  double *Fx;
  plhs[0] = mxCreateDoubleMatrix(4,1,mxREAL);
  Fx = mxGetPr(plhs[0]);
  Fx[0] = car->get_wheel_longitudinal_force(Car::FL);
  Fx[1] = car->get_wheel_longitudinal_force(Car::FR);
  Fx[2] = car->get_wheel_longitudinal_force(Car::RL);
  Fx[3] = car->get_wheel_longitudinal_force(Car::RR);
  return;
}

void
MexCar_get_wheel_longitudinal_speeds(int nlhs, mxArray *plhs[],
				     int nrhs, const mxArray *prhs[])
{
  if (nrhs != 0) {
    mexWarnMsgTxt(usage_text("get_wheel_longitudinal_speeds"));
    mexWarnMsgTxt("ignoring extra arguments");
  }

  double *vx;
  plhs[0] = mxCreateDoubleMatrix(4,1,mxREAL);
  vx = mxGetPr(plhs[0]);
  vx[0] = car->get_wheel_longitudinal_speed(Car::FL);
  vx[1] = car->get_wheel_longitudinal_speed(Car::FR);
  vx[2] = car->get_wheel_longitudinal_speed(Car::RL);
  vx[3] = car->get_wheel_longitudinal_speed(Car::RR);
  return;
}

void
MexCar_get_wheel_slip_angles(int nlhs, mxArray *plhs[],
			     int nrhs, const mxArray *prhs[])
{
  if (nrhs != 0) {
    mexWarnMsgTxt(usage_text("get_wheel_slip_angles"));
    mexWarnMsgTxt("ignoring extra arguments");
  }

  double *alpha;
  plhs[0] = mxCreateDoubleMatrix(4,1,mxREAL);
  alpha = mxGetPr(plhs[0]);
  alpha[0] = car->get_wheel_slip_angle(Car::FL);
  alpha[1] = car->get_wheel_slip_angle(Car::FR);
  alpha[2] = car->get_wheel_slip_angle(Car::RL);
  alpha[3] = car->get_wheel_slip_angle(Car::RR);
  return;
}

void
MexCar_get_wheel_steering_angles(int nlhs, mxArray *plhs[],
				 int nrhs, const mxArray *prhs[])
{
  if (nrhs != 0) {
    mexWarnMsgTxt(usage_text("get_wheel_steering_angles"));
    mexWarnMsgTxt("ignoring extra arguments");
  }

  double *delta;
  plhs[0] = mxCreateDoubleMatrix(4,1,mxREAL);
  delta = mxGetPr(plhs[0]);
  delta[0] = car->get_wheel_steering_angle(Car::FL);
  delta[1] = car->get_wheel_steering_angle(Car::FR);
  delta[2] = car->get_wheel_steering_angle(Car::RL);
  delta[3] = car->get_wheel_steering_angle(Car::RR);
  return;
}

void
MexCar_get_wheel_vertical_forces(int nlhs, mxArray *plhs[],
				 int nrhs, const mxArray *prhs[])
{
  if (nrhs != 0) {
    mexWarnMsgTxt(usage_text("get_wheel_vertical_forces"));
    mexWarnMsgTxt("ignoring extra arguments");
  }

  double *Fz;
  plhs[0] = mxCreateDoubleMatrix(4,1,mxREAL);
  Fz = mxGetPr(plhs[0]);
  Fz[0] = car->get_wheel_vertical_force(Car::FL);
  Fz[1] = car->get_wheel_vertical_force(Car::FR);
  Fz[2] = car->get_wheel_vertical_force(Car::RL);
  Fz[3] = car->get_wheel_vertical_force(Car::RR);
  return;
}

void
MexCar_integrate_euler_linear(int nlhs, mxArray *plhs[],
			      int nrhs, const mxArray *prhs[])
{
  if (nrhs != 2) {
    mexWarnMsgTxt(usage_text("integrate_euler_linear"));
    mexErrMsgTxt("incorrect number of arguments");
    return;
  }
  double *arg1;
  double *arg2;
  arg1 = mxGetPr(prhs[0]);
  arg2 = mxGetPr(prhs[1]);
  double dt = arg1[0];
  int n = static_cast<int>(arg2[0]);
  for (int i = 0; i < n; i++) {
    DEBUGPRINT("n = %i\n", n);
    car->integrate_euler(dt);
  }
  return;
}

void
MexCar_integrate_euler_nonlinear(int nlhs, mxArray *plhs[],
				 int nrhs, const mxArray *prhs[])
{
  if (nrhs != 2) {
    mexWarnMsgTxt(usage_text("integrate_euler_nonlinear"));
    mexErrMsgTxt("incorrect number of arguments");
    return;
  }
  double *args;
  double dt = *mxGetPr(prhs[0]);
  int n = static_cast<int>(*mxGetPr(prhs[1]));
  for (int i = 0; i < n; i++) {
    car->Car::integrate_euler(dt);
  }
  return;
}

void
MexCar_set_acceleration(int nlhs, mxArray *plhs[],
			int nrhs, const mxArray *prhs[])
{
  if (nrhs != 1) {
    mexWarnMsgTxt(usage_text("set_acceleration"));
    mexErrMsgTxt("incorrect number of arguments");
    return;
  }
  
  double *A;
  A = mxGetPr(prhs[0]);
  
  Car::axis acceleration;
  acceleration.X = A[0];
  acceleration.Y = A[1];
  acceleration.Psi = A[2];
  DEBUGPRINT("Setting acceleration = [%f;%f;%f] ... ", A[0], A[1], A[2]);
  car->set_acceleration(acceleration);
  DEBUGPRINT("done.\n");
  return;
}

void
MexCar_set_friction_coefficient(int nlhs, mxArray *plhs[],
				int nrhs, const mxArray *prhs[])
{
  if (nrhs != 1) {
    mexWarnMsgTxt(usage_text("set_friction_coefficient"));
    mexErrMsgTxt("incorrect number of arguments");
    return;
  }
  double *mu;
  mu = mxGetPr(prhs[0]);
  DEBUGPRINT("Setting mu = %f ... ", *mu);
  car->set_friction_coefficient(*mu);
  DEBUGPRINT("done.\n");
  return;
}

void
MexCar_set_velocity(int nlhs, mxArray *plhs[],
		    int nrhs, const mxArray *prhs[])
{
  if (nrhs != 1) {
    mexWarnMsgTxt(usage_text("set_velocity"));
    mexErrMsgTxt("incorrect number of arguments");
    return;
  }
  
  double *V;
  V = mxGetPr(prhs[0]);
  
  Car::axis velocity;
  velocity.X = V[0];
  velocity.Y = V[1];
  velocity.Psi = V[2];
  DEBUGPRINT("Setting velocity = [%f;%f;%f] ... ", V[0], V[1], V[2]);
  car->set_velocity(velocity);
  DEBUGPRINT("done.\n");
  return;
}

void
MexCar_set_wheel_longitudinal_forces(int nlhs, mxArray *plhs[],
				     int nrhs, const mxArray *prhs[])
{
  if (nrhs != 1) {
    mexWarnMsgTxt(usage_text("set_wheel_longitudinal_forces"));
    mexErrMsgTxt("incorrect number of arguments");
    return;
  }

  double *Fx;
  Fx = mxGetPr(prhs[0]);
  car->set_wheel_longitudinal_force(Car::FL, Fx[0]);
  car->set_wheel_longitudinal_force(Car::FR, Fx[1]);
  car->set_wheel_longitudinal_force(Car::RL, Fx[2]);
  car->set_wheel_longitudinal_force(Car::RR, Fx[3]);
  return;
}

void
MexCar_set_wheel_slip_angles(int nlhs, mxArray *plhs[],
			     int nrhs, const mxArray *prhs[])
{
  if (nrhs != 1) {
    mexWarnMsgTxt(usage_text("set_wheel_slip_angles"));
    mexErrMsgTxt("incorrect number of arguments");
    return;
  }

  double *alpha;
  alpha = mxGetPr(prhs[0]);
  car->set_wheel_slip_angle(Car::FL, alpha[0]);
  car->set_wheel_slip_angle(Car::FR, alpha[1]);
  car->set_wheel_slip_angle(Car::RL, alpha[2]);
  car->set_wheel_slip_angle(Car::RR, alpha[3]);
  return;
}

void
MexCar_set_wheel_speeds(int nlhs, mxArray *plhs[],
			int nrhs, const mxArray *prhs[])
{
  if (nrhs != 1) {
    mexWarnMsgTxt(usage_text("set_wheel_speeds"));
    mexErrMsgTxt("incorrect number of arguments");
    return;
  }

  double *vx;
  vx = mxGetPr(prhs[0]);
  car->set_wheel_speed(Car::FL, vx[0]);
  car->set_wheel_speed(Car::FR, vx[1]);
  car->set_wheel_speed(Car::RL, vx[2]);
  car->set_wheel_speed(Car::RR, vx[3]);
  return;
}

void
MexCar_set_wheel_steering_angle(int nlhs, mxArray *plhs[],
				int nrhs, const mxArray *prhs[])
{
  if (nrhs != 1) {
    mexWarnMsgTxt(usage_text("set_wheel_steering_angle"));
    mexErrMsgTxt("incorrect number of arguments");
    return;
  }
  double *angle;
  angle = mxGetPr(prhs[0]);
  DEBUGPRINT("Setting delta = %f ... ", *angle);
  car->set_wheel_steering_angle(*angle);
  DEBUGPRINT("done.\n");
  return;
}

void
MexCar_set_wheel_vertical_forces(int nlhs, mxArray *plhs[],
				 int nrhs, const mxArray *prhs[])
{
  if (nrhs != 1) {
    mexWarnMsgTxt(usage_text("set_wheel_vertical_forces"));
    mexErrMsgTxt("incorrect number of arguments");
    return;
  }

  double *Fz;
  Fz = mxGetPr(prhs[0]);
  car->set_wheel_vertical_force(Car::FL, Fz[0]);
  car->set_wheel_vertical_force(Car::FR, Fz[1]);
  car->set_wheel_vertical_force(Car::RL, Fz[2]);
  car->set_wheel_vertical_force(Car::RR, Fz[3]);
  return;
}

void
MexCar_write_parameters(int nlhs, mxArray *plhs[],
			int nrhs, const mxArray *phrs[])
{
  if (car == NULL) {
    car = new LinearisableCar();
  }
  car->write_parameters();
  return;
}

typedef void (*pMexF)(int, mxArray*[], int, const mxArray*[]);

void
CallMexSubfunction(const char *subfunction,
		   int nlhs, mxArray *plhs[],
		   int nrhs, const mxArray *prhs[])
{
  pMexF f;
  if (! strcmp(subfunction, "get_A")) {
    f = &MexCar_get_A;
  } else if (! strcmp(subfunction, "get_B")) {
    f = &MexCar_get_B;
  } else if (! strcmp(subfunction, "get_position")) {
    f = &MexCar_get_position;
  } else if (! strcmp(subfunction, "get_velocity")) {
    f = &MexCar_get_velocity;
  } else if (! strcmp(subfunction, "get_acceleration")) {
    f = &MexCar_get_acceleration;
  } else if (! strcmp(subfunction, "get_friction_coefficient")) {
    f = &MexCar_get_friction_coefficient;
  } else if (! strcmp(subfunction, "get_wheel_lateral_forces")) {
    f = &MexCar_get_wheel_lateral_forces;
  } else if (! strcmp(subfunction, "get_wheel_lateral_speeds")) {
    f = &MexCar_get_wheel_lateral_speeds;
  } else if (! strcmp(subfunction, "get_wheel_longitudinal_forces")) {
    f = &MexCar_get_wheel_longitudinal_forces;
  } else if (! strcmp(subfunction, "get_wheel_longitudinal_speeds")) {
    f = &MexCar_get_wheel_longitudinal_speeds;    
  } else if (! strcmp(subfunction, "get_wheel_slip_angles")) {
    f = &MexCar_get_wheel_slip_angles;    
  } else if (! strcmp(subfunction, "get_wheel_steering_angles")) {
    f = &MexCar_get_wheel_steering_angles;    
  } else if (! strcmp(subfunction, "get_wheel_vertical_forces")) {
    f = &MexCar_get_wheel_vertical_forces;
  } else if (! strcmp(subfunction, "integrate_euler_linear")) {
    f = &MexCar_integrate_euler_linear;    
  } else if (! strcmp(subfunction, "integrate_euler_nonlinear")) {
    f = &MexCar_integrate_euler_nonlinear;    
  } else if (! strcmp(subfunction, "set_acceleration")) {
    f = &MexCar_set_acceleration;    
  } else if (! strcmp(subfunction, "set_friction_coefficient")) {
    f = &MexCar_set_friction_coefficient;    
  } else if (! strcmp(subfunction, "set_velocity")) {
    f = &MexCar_set_velocity;    
  } else if (! strcmp(subfunction, "set_wheel_longitudinal_forces")) {
    f = &MexCar_set_wheel_longitudinal_forces;    
  } else if (! strcmp(subfunction, "set_wheel_slip_angles")) {
    f = &MexCar_set_wheel_slip_angles;    
  } else if (! strcmp(subfunction, "set_wheel_speeds")) {
    f = &MexCar_set_wheel_speeds;    
  } else if (! strcmp(subfunction, "set_wheel_steering_angle")) {
    f = &MexCar_set_wheel_steering_angle;    
  } else if (! strcmp(subfunction, "set_wheel_vertical_forces")) {
    f = &MexCar_set_wheel_vertical_forces;    
  } else if (! strcmp(subfunction, "write_parameters")) {
    f = &MexCar_write_parameters;    
  } else {
    mexErrMsgTxt("Unknown subfunction");
  }
  f(nlhs, plhs, nrhs, prhs);	  
  return;
}

