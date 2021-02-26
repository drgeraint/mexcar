// -*-c++-*-

/************************************************************
 * LinearisableCar.hh
 *
 * Geraint Paul Bevan <geraint.bevan@gcu.ac.uk>
 * Initial Revision : <2005-06-22>
 * Latest Time-stamp: <2021/02/25 22:16:51 geraint>
 ***********************************************************/

#ifndef _LINEARISABLECAR_H_
#define _LINEARISABLECAR_H_

#include "Car.hh"

/// linear velocity-based model of a car.
/** 
 * The LinearisableCar model creates a velocity-based
 * linearisation of the Car model.
 * 
 * The continuous states are the longitudinal, lateral and
 * yaw acceleration of the vehicle body and the rates are
 * the corresponding rates of change of acceleration.
 *
 * The inputs are the rates of change of the wheel
 * longitudinal forces (in their own axis system) and the
 * rate of change of steering angle.
 */
class LinearisableCar : public Car
{
public:

  LinearisableCar();

  /// number of states and inputs of the linearised model.
  /**
   * NX is the number of states; NU is the number of inputs;
   * NW is the number of disturbances.
   */
  enum vector_size { NX = 3, NU = 5, NW = 1 };
  /** size of state, input and disturbance vectors */
  typedef enum vector_size vector_size;

  /** state matrix */
  typedef double A_matrix[NX][NX];
  /** input matrix */
  typedef double B_matrix[NU][NX];
  /** disturbance matrix */
  typedef double W_matrix[NW][NX];
  
  void integrate_euler(const double dt);

  void get_A(double *A_array);
  void get_B(double *B_array);
  void get_W(double *NX);

  void print_A(void) const;
  void print_B(void) const;
  void print_W(void) const;

private:

  /** the state matrix of the linearised model */
  A_matrix A;

  /** the input matrix of the linearised model */
  B_matrix B;

  /** the disturbance matrix of the linearised model */
  W_matrix W;

  void update_A(void);
  void update_B(void);
  void update_W(void);
  
  // functions for A matrix

  double ddotX_dotX(void) const;
  double ddotX_dotY(void) const;
  double ddotX_dotPsi(void) const;

  double ddotY_dotX(void) const;
  double ddotY_dotY(void) const;
  double ddotY_dotPsi(void) const;

  double ddotPsi_dotX(void) const;
  double ddotPsi_dotY(void) const;
  double ddotPsi_dotPsi(void) const;

  // functions for B matrix

  double ddotX_fx(const wheel i) const;
  double ddotY_fx(const wheel i) const;
  double ddotPsi_fx(const wheel i) const;

  double ddotX_delta(void) const;
  double ddotY_delta(void) const;
  double ddotPsi_delta(void) const;

  // functions for A and B matrix functions

  double alpha_vx(const wheel i) const;
  double alpha_vy(const wheel i) const;

  double ddotX_fy(const wheel i) const;
  double ddotY_fy(const wheel i) const;
  double ddotPsi_fy(const wheel i) const;

  double fy_alpha(const wheel i) const;

  double vx_delta(const wheel i) const;
  double vy_delta(const wheel i) const;

  double vx_dotX(const wheel i) const;
  double vy_dotX(const wheel i) const;

  double vx_dotY(const wheel i) const;
  double vy_dotY(const wheel i) const;

  double vx_dotPsi(const wheel i) const;
  double vy_dotPsi(const wheel i) const;

  bool warned_linearised_integration;

  // function for W matrix
  double fy_mu(const wheel i) const;
};

#endif // _LINEARISABLECAR_H_
