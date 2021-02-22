// -*-c++-*-

/***********************************************************
 * LinearisableCar.cc
 *
 * Geraint Paul Bevan <g.bevan@mech.gla.ac.uk>
 * Initial Revision : <2005-06-22>
 * Latest Time-stamp: <2007-08-05 21:39:40 geraint>
 **********************************************************/

#include <cmath>
#include <cstring>
#include <iostream>
#include "LinearisableCar.hh"

/// initialises the LinearisableCar.
/**
 * Creates and initialises the LinearisableCar and updates
 * the state ("A") and input ("B") matrices.
 */
LinearisableCar::LinearisableCar() {
  update_A();
  update_B();
  warned_linearised_integration = false;
}


/// updates and returns the system state matrix.
/** 
 * Returns the state ("A") matrix of a velocity-based
 * linearisation of the nonlinear function that describes
 * the Car rates, created by partially differentiating the
 * function with respect to the states.
 */
void
LinearisableCar::get_A(double *A_array) {  
  update_A();
  memcpy(A_array,A,sizeof(A));
}

/// updates and returns the system input matrix.
/**
 * Returns the input ("B") matrix of a velocity-based
 * linearisation of the nonlinear function that describes
 * the Car rates, created by partially differentiating the
 * function with respect to the inputs.
 */
void
LinearisableCar::get_B(double *B_array) {  
  update_B();
  memcpy(B_array,B,sizeof(B));
}

/// performs one Euler integration step.
/**
 * The vehicle acceleration is updated and then integrated
 * over a period of dt seconds to obtain new vehicle
 * velocities; these are integrated in turn to obtain a new
 * vehicle position.
 *
 * CAUTION: This integrates the LINEARISED model, not the
 * underlying nonlinear model. For that, use the function
 * Car::integrate_euler()
 */
void
LinearisableCar::integrate_euler(const double dt) {
  if (warned_linearised_integration == false) {
    std::cout << "*** Warning: LinearisableCar::integrate_euler"
	      << std::endl
	      << "This function performs an integration of the" << std::endl
	      << "linearised vehicle model. For simulation you" << std::endl
	      << "probably want to integrate the underlying" << std::endl
	      << "nonlinear model, using the function" << std::endl
	      << "void Car::integrate_euler(double dt)" << std::endl
	      << std::endl;
    warned_linearised_integration = true;
  }
  update_A();
  update_B();

  static double old_fx[4];
  static double old_delta[4];
  static bool init = false;
  if (init == false)
    for (wheel i = FL; i <= RR; ++i) {
      old_fx[i] = fx[i];
      old_delta[i] = delta[i];
      init = true;
    }

  update_tyre_forces();
  
  double fxdot[4];
  double deltadot[4];
  for (wheel i = FL; i <= RR; ++i) {
    fxdot[i] = (fx[i] - old_fx[i]) / dt;
    deltadot[i] = (delta[i] - old_delta[i]) / dt;
    old_fx[i] = fx[i];
    old_delta[i] = delta[i];
  }

  axis w;
  w.X =
    + A[0][0]*acceleration.X
    + A[1][0]*acceleration.Y
    + A[2][0]*acceleration.Psi
    + B[0][0]*fxdot[FL]
    + B[1][0]*fxdot[FR]
    + B[2][0]*fxdot[RL]
    + B[3][0]*fxdot[RR]
    + B[4][0]*(deltadot[FL]+deltadot[FR])/2.0;
  w.Y =
    + A[0][1]*acceleration.X
    + A[1][1]*acceleration.Y
    + A[2][1]*acceleration.Psi
    + B[0][1]*fxdot[FL]
    + B[1][1]*fxdot[FR]
    + B[2][1]*fxdot[RL]
    + B[3][1]*fxdot[RR]
    + B[4][1]*(deltadot[FL]+deltadot[FR])/2.0;
  w.Psi =
    + A[0][2]*acceleration.X
    + A[1][2]*acceleration.Y
    + A[2][2]*acceleration.Psi
    + B[0][2]*fxdot[FL]
    + B[1][2]*fxdot[FR]
    + B[2][2]*fxdot[RL]
    + B[3][2]*fxdot[RR]
    + B[4][2]*(deltadot[FL]+deltadot[FR])/2.0;

  position.X += dt * velocity.X * cos(position.Psi)
    - dt * velocity.Y * sin(position.Psi);

  position.Y += dt * velocity.X*sin(position.Psi)
    + dt * velocity.Y * cos(position.Psi);

  position.Psi += dt * velocity.Psi;

  velocity.X += dt * acceleration.X;
  velocity.Y += dt * acceleration.Y;
  velocity.Psi += dt * acceleration.Psi;

  acceleration.X += dt * w.X;
  acceleration.Y += dt * w.Y;
  acceleration.Psi += dt * w.Psi;
}

/// prints the system state matrix to standard output. */
void
LinearisableCar::print_A(void) const {
  const char tab = '\t';
  std::cout.setf(std::ios::scientific);
  std::cout.precision(6);
  std::cout << std::endl << "A matrix:" << std::endl << std::endl
	    << A[0][0] << tab << A[1][0] << tab << A[2][0] << std::endl
	    << A[0][1] << tab << A[1][1] << tab << A[2][1] << std::endl
	    << A[0][2] << tab << A[1][2] << tab << A[2][2] << std::endl;
}

/// prints the system input matrix to standard output. */
void
LinearisableCar::print_B(void) const {
  const char tab = '\t';
  std::cout.setf(std::ios::scientific);
  std::cout.precision(6);
  std::cout << std::endl << "B matrix" << std::endl << std::endl
	    << B[0][0] << tab << B[1][0] << tab << B[2][0] << tab 
	    << B[3][0] << tab << B[4][0] << std::endl
	    << B[0][1] << tab << B[1][1] << tab << B[2][1] << tab
	    << B[3][1] << tab << B[4][1] << std::endl
	    << B[0][2] << tab << B[1][2] << tab << B[2][2] << tab
	    << B[3][2] << tab << B[4][2] << std::endl;
}

/// updates the system state matrix.
/**
 * Calculates the state ("A") matrix after updating the tyre
 * forces.
 */
void
LinearisableCar::update_A(void) {
  update_tyre_forces();
  A[0][0] = ddotX_dotX();
  A[1][0] = ddotX_dotY();
  A[2][0] = ddotX_dotPsi();

  A[0][1] = ddotY_dotX();
  A[1][1] = ddotY_dotY();
  A[2][1] = ddotY_dotPsi();

  A[0][2] = ddotPsi_dotX();
  A[1][2] = ddotPsi_dotY();
  A[2][2] = ddotPsi_dotPsi();
}

/// updates the system input matrix.
/**
 * Calculates the input ("B") matrix after updating the tyre
 * forces.
 */
void
LinearisableCar::update_B(void) {
  update_tyre_forces();
  B[0][0] = ddotX_fx(FL);
  B[1][0] = ddotX_fx(FR);
  B[2][0] = ddotX_fx(RL);
  B[3][0] = ddotX_fx(RR);
  B[4][0] = ddotX_delta();

  B[0][1] = ddotY_fx(FL);
  B[1][1] = ddotY_fx(FR);
  B[2][1] = ddotY_fx(RL);
  B[3][1] = ddotY_fx(RR);
  B[4][1] = ddotY_delta();

  B[0][2] = ddotPsi_fx(FL);
  B[1][2] = ddotPsi_fx(FR);
  B[2][2] = ddotPsi_fx(RL);
  B[3][2] = ddotPsi_fx(RR);
  B[4][2] = ddotPsi_delta();
}

// functions for A matrix

///partial derivative.
/**
 * Partial derivative of body longitudinal acceleration with
 * respect to body longitudinal velocity.
 */
double
LinearisableCar::ddotX_dotX(void) const {
  double retval = 0.0;
  for (wheel i = FL; i <= RR; ++i) {
    retval += ddotX_fy(i) * fy_alpha(i) *
      (alpha_vx(i) * vx_dotX(i) +
       alpha_vy(i) * vy_dotX(i));
  }
  return retval;
}

/// partial derivative.
/**
 * Partial derivative of body longitudinal acceleration with
 * respect to body lateral velocity.
 */
double
LinearisableCar::ddotX_dotY(void) const {
  double retval = 0.0;
  for (wheel i = FL; i <= RR; ++i) {
    retval += ddotX_fy(i) * fy_alpha(i) *
      (alpha_vx(i) * vx_dotY(i) +
       alpha_vy(i) * vy_dotY(i));
  }
  return retval;
}

/// partial derivative.
/**
 * Partial derivative of body longitudinal acceleration with
 * respect to body yaw velocity.
 */
double
LinearisableCar::ddotX_dotPsi(void) const {
  double retval = 0.0;
  for (wheel i = FL; i <= RR; ++i) {
    retval += ddotX_fy(i) * fy_alpha(i) *
      (alpha_vx(i) * vx_dotPsi(i) +
       alpha_vy(i) * vy_dotPsi(i));
  }
  return retval;
}


/// partial derivative.
/**
 * Partial derivative of body lateral acceleration with
 * respect to body longitudinal velocity.
 */
double
LinearisableCar::ddotY_dotX(void) const {
  double retval = 0.0;
  for (wheel i = FL; i <= RR; ++i) {
    retval += ddotY_fy(i) * fy_alpha(i) *
      (alpha_vx(i) * vx_dotX(i) +
       alpha_vy(i) * vy_dotX(i));
  }
  return retval;
}

/// partial derivative.
/**
 * Partial derivative of body lateral acceleration with
 * respect to body lateral velocity.
 */
double
LinearisableCar::ddotY_dotY(void) const {
  double retval = 0.0;
  for (wheel i = FL; i <= RR; ++i) {
    retval += ddotY_fy(i) * fy_alpha(i) *
      (alpha_vx(i) * vx_dotY(i) +
       alpha_vy(i) * vy_dotY(i));
  }
  return retval;
}

/// partial derivative.
/**
 * Partial derivative of body lateral acceleration with
 * respect to body yaw velocity.
 */
double
LinearisableCar::ddotY_dotPsi(void) const {
  double retval = 0.0;
  for (wheel i = FL; i <= RR; ++i) {
    retval += ddotY_fy(i) * fy_alpha(i) *
      (alpha_vx(i) * vx_dotPsi(i) +
       alpha_vy(i) * vy_dotPsi(i));
  }
  return retval;
}


/// partial derivative.
/**
 * Partial derivative of body yaw acceleration with respect
 * to body longitudinal velocity.
 */
double
LinearisableCar::ddotPsi_dotX(void) const {
  double retval = 0.0;
  for (wheel i = FL; i <= RR; ++i) {
    retval += ddotPsi_fy(i) * fy_alpha(i) *
      (alpha_vx(i) * vx_dotX(i) +
       alpha_vy(i) * vy_dotX(i));
  }
  return retval;
}

/// partial derivative.
/**
 * Partial derivative of body yaw acceleration with respect
 * to body lateral velocity.
 */
double
LinearisableCar::ddotPsi_dotY(void) const {
  double retval = 0.0;
  for (wheel i = FL; i <= RR; ++i) {
    retval += ddotPsi_fy(i) * fy_alpha(i) *
      (alpha_vx(i) * vx_dotY(i) +
       alpha_vy(i) * vy_dotY(i));
  }
  return retval;
}

/// partial derivative.
/**
 * Partial derivative of body yaw acceleration with respect
 * to body yaw velocity.
 */
double
LinearisableCar::ddotPsi_dotPsi(void) const {
  double retval = 0.0;
  for (wheel i = FL; i <= RR; ++i) {
    retval += ddotPsi_fy(i) * fy_alpha(i) *
      (alpha_vx(i) * vx_dotPsi(i) +
       alpha_vy(i) * vy_dotPsi(i));
  }
  return retval;
}

// functions for B matrix

/// partial derivative.
/**
 * Partial derivative of body longitudinal acceleration with
 * respect to wheel longitudinal force.
 */
double
LinearisableCar::ddotX_fx(const wheel i) const{
  return cos(delta[i]) / m;
}

/// partial derivative.
/**
 * Partial derivative of body lateral acceleration with
 * respect to wheel longitudinal force.
 */
double
LinearisableCar::ddotY_fx(const wheel i) const {
  return sin(delta[i]) / m;
}

/// partial derivative.
/**
 * Partial derivative of body yaw acceleration with respect
 * to wheel longitudinal force.
 */
double
LinearisableCar::ddotPsi_fx(const wheel i) const {
  return (-Y[i]*cos(delta[i]) + X[i]*sin(delta[i])) / Izz;
}

/// partial derivative.
/**
 * Partial derivative of body longitudinal acceleration with
 * respect to wheel steering angle.
 */
double
LinearisableCar::ddotX_delta(void) const {
  double retval = 0.0;
  for (wheel i = FL; i <= RR; ++i) {
    retval += ddotX_fy(i) * fy_alpha(i) *
      (alpha_vx(i) * vx_delta(i) +
       alpha_vy(i) * vy_delta(i));
  }
  return retval;
}

/// partial derivative.
/**
 * Partial derivative of body lateral acceleration with
 * respect to wheel steering angle.
 */
double
LinearisableCar::ddotY_delta(void) const {
  double retval = 0.0;
  for (wheel i = FL; i <= RR; ++i) {
    retval += ddotY_fy(i) * fy_alpha(i) *
      (alpha_vx(i) * vx_delta(i) +
       alpha_vy(i) * vy_delta(i));
  }
  return retval;
}

/// partial derivative.
/**
 * Partial derivative of body yaw acceleration with respect
 * to wheel steering angle.
 */
double
LinearisableCar::ddotPsi_delta(void) const {
  double retval = 0.0;
  for (wheel i = FL; i <= RR; ++i) {
    retval += ddotPsi_fy(i) * fy_alpha(i) *
      (alpha_vx(i) * vx_delta(i) +
       alpha_vy(i) * vy_delta(i));
  }
  return retval;
}

// functions for A and B matrix functions

/// partial derivative.
/**
 * Partial derivative of wheel slip angle with respect to
 * wheel longitudinal velocity.
 */
double
LinearisableCar::alpha_vx(const wheel i) const {
  return - vy[i] / (vx[i]*vx[i] + vy[i]*vy[i]);
}

/// partial derivative.
/**
 * Partial derivative of wheel slip angle with respect to
 * wheel lateral velocity.
 */
double
LinearisableCar::alpha_vy(const wheel i) const {
  return + vx[i] / (vx[i]*vx[i] + vy[i]*vy[i]);
}

/// partial derivative.
/** 
 * Partial derivative of body longitudinal acceleration with
 * respect to wheel lateral force.
 */
double
LinearisableCar::ddotX_fy(const wheel i) const {
  return - sin(delta[i]) / m;
}

/// partial derivative.
/**
 * Partial derivative of body lateral acceleration with
 * respect to wheel lateral force.
 */
double
LinearisableCar::ddotY_fy(const wheel i) const {
  return + cos(delta[i]) / m;
}

/// partial derivative.
/**
 * Partial derivative of body yaw acceleration with respect
 * to wheel lateral force.
 */
double
LinearisableCar::ddotPsi_fy(const wheel i) const {
  return (X[i]*cos(delta[i]) + Y[i]*sin(delta[i])) / Izz;
}

/// partial derivative.
/**
 * Partial derivative of wheel lateral force with respect to
 * wheel slip angle.
 */
double
LinearisableCar::fy_alpha(const wheel i) const {
  double phi = By * alpha[i];
  double p1 = Ey * (phi - atan(phi));
  double p2 = Cy * atan(phi - p1);
  double f = - Dy * sin(p2);

  double p1_phi = Ey * (1.0 - 1.0 / (1.0 + phi*phi));
  double p2_phi = (Cy / (1.0 + (phi - p1)*(phi - p1))) * (1.0 - p1_phi);
  double f_phi = - Dy * cos(p2) * p2_phi;

  return By * f_phi;
}

/// partial derivative.
/**
 * Partial derivative of wheel longitudinal velocity with
 * respect to wheel steering angle.
 */
double
LinearisableCar::vx_delta(const wheel i) const {
  return
    + (velocity.Y + X[i]*velocity.Psi) * cos(delta[i])
    - (velocity.X - Y[i]*velocity.Psi) * sin(delta[i]);
}

/// partial derivative.
/** 
 * Partial derivative of wheel lateral velocity with respect
 * to wheel steering angle.
 */
double
LinearisableCar::vy_delta(const wheel i) const {
  return
    - (velocity.X - Y[i]*velocity.Psi) * cos(delta[i])
    - (velocity.Y + X[i]*velocity.Psi) * sin(delta[i]);
}

/// partial derivative.
/** 
 * Partial derivative of wheel longitudinal velocity with
 * respect to body longitudinal velocity.
 */
double
LinearisableCar::vx_dotX(const wheel i) const {
  return + cos(delta[i]);
}

/// partial derivative
/** 
 * Partial derivative of wheel lateral velocity with respect
 * to body longitudinal velocity.
 */
double
LinearisableCar::vy_dotX(const wheel i) const {
  return - sin(delta[i]);
}

/// partial derivative
/** 
 * Partial derivative of wheel longitudinal velocity with
 * respect to body lateral velocity.
 */
double
LinearisableCar::vx_dotY(const wheel i) const {
  return + sin(delta[i]);
}

/// partial derivative
/** 
 * Partial derivative of wheel lateral velocity with respect
 * to body lateral velocity.
 */
double
LinearisableCar::vy_dotY(const wheel i) const {
  return + cos(delta[i]);
}

/// partial derivative 
/** 
 * derivative of wheel longitudinal velocity with respect to
 * body yaw velocity.
 */
double
LinearisableCar::vx_dotPsi(const wheel i) const {
  return - Y[i]*cos(delta[i]) + X[i]*sin(delta[i]);
}

/// partial derivative
/** 
 * Partial derivative of wheel lateral velocity with respect
 * to body yaw velocity.
 */
double
LinearisableCar::vy_dotPsi(const wheel i) const {
  return + X[i]*cos(delta[i]) + Y[i]*sin(delta[i]);
}
