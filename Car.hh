// -*-c++-*-

/************************************************************
 * Car.hh declares Car - a non-linear vehicle model.
 *
 * Geraint Paul Bevan <g.bevan@mech.gla.ac.uk>
 * Initial Revision : <2005-06-21>
 * Latest Time-stamp: <2007-08-05 21:18:12 geraint>
 *
 * $Id: Car.hh,v 1.1 2008-01-09 14:21:13 gbevan Exp $
 *
 ************************************************************/


#ifndef _CAR_HH_
#define _CAR_HH_

/// nonlinear two-track model of a car.
/**
 * Car implements a non-linear two-track model of vehicle
 * dynamics. The model is intended primarily for lateral
 * dynamics analysis.
 *
 * During simulation, the continuous states are the
 * longitudinal, lateral and yaw velocity of the vehicle
 * body and the rates are the corresponding accelerations.
 *
 * The inputs are the wheel longitudinal forces (in their
 * own axis system) and the steering angle. Wheel speeds and
 * forces are retained, thus acting as discrete states.
 */
class Car
{
public:

  Car();

  /// axis is a structure containing three vector elements.
  /**
   * The axis elements are used for containing longitudinal,
   * lateral and yaw components of position, velocity and
   * acceleration.
   */
  struct axis {
    /** longitudinal component */
    double X;
    /** lateral component */
    double Y;
    /** yaw component */
    double Psi;
  };
  /** a structure to contain vector components. */
  typedef struct axis axis;

  /// wheel is an enumeration for indexing wheels.
  /**
   * - FL: Front left;
   * - FR: Front right;
   * - RL: Rear left;
   * - RR: Rear right.
   *
   * A prefix increment operator is defined so that it is
   * possible to loop over each of the wheels with a "for"
   * construct: for (i = FL; i <= RR; ++i) { ... }
   */
  enum wheel { FL = 0, FR, RL, RR };
  /** 
   * an enumeration to provide clear and unambiguous
   * reference to each of the vehicle's wheels.
   */
  typedef enum wheel wheel;

  axis get_position(void) const;
  axis get_velocity(void) const;
  axis get_acceleration(void) const;
  
  double get_friction_coefficient(void) const;
  double get_wheel_lateral_force(const wheel i) const;
  double get_wheel_lateral_speed(const wheel i) const;
  double get_wheel_longitudinal_force(const wheel i) const;
  double get_wheel_longitudinal_speed(const wheel i) const;
  double get_wheel_slip_angle(const wheel i) const;
  double get_wheel_steering_angle(const wheel i) const;
  double get_wheel_vertical_force(const wheel i) const;

  virtual void integrate_euler(double dt);

  void set_acceleration(const axis &accel);
  void set_friction_coefficient(const double friction_coefft);
  void set_velocity(const axis &vel);
  void set_wheel_lateral_force(const wheel i, const double force);
  void set_wheel_longitudinal_force(const wheel i, const double force);
  void set_wheel_slip_angle(const wheel i, const double angle);
  void set_wheel_speed(const wheel i, const double speed);
  void set_wheel_steering_angle(const double angle);
  void set_wheel_vertical_force(const wheel i, const double force);

  void write_parameters(void) const;

protected:

  void update_acceleration(void);
  void update_tyre_forces(void);
  void update_weight_distribution(void);
  void update_wheel_speeds(void);

  /// stores the vehicle position (fixed Earth).
  /**
   * The position of the centre of gravity of the Car is
   * measured relative to the Earth axis system.
   */
  axis position;
  
  /// stores the vehicle velocity.
  /** 
   * The velocity of the centre of gravity of the Car is
   * measured relative to its body axis system.
   */
  axis velocity;

  /// stores the vehicle acceleration.
  /** 
   * The acceleration of the centre of gravity of the Car is
   * measured relative to its body axis system.
   */
  axis acceleration;


  // parameters

  /** vehicle mass (kilogrammes) */
  double m;

  /** moment of inertia (kilogrammes metres squared)*/
  double Izz;

  /** longitudinal moment arm to wheels (metres) */
  double X[4];

  /** lateral moment arm to wheels (metres) */
  double Y[4];

  /** height of centre of mass (metres) */
  double Z0;

  /** tyre lateral parameter (non-dimensional, Pacejka) */
  double By, Cy, Dy, Ey;

  /** gravitational constant (metres per second squared)*/
  double g;

  /** friction coefficient (-) */
  double mu;

  // inputs

  /** wheel steering angle (radians) */
  double delta[4];

  /** tyre longitudinal force (Newtons) */
  double fx[4];


  // intermediate variables

  /** tyre slip angles (radians) */
  double alpha[4];

  /** tyre lateral forces (Newtons) */
  double fy[4];

  /** wheel longitudinal speeds (metres per second) */
  double vx[4];

  /** wheel lateral speeds (metres per second) */
  double vy[4];


  /** tyre vertical load (Newtons) */
  double fz[4];
};

Car::wheel &operator++(Car::wheel &w);

#endif // _CAR_HH_
