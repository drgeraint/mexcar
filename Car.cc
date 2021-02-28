// -*-c++-*-

/************************************************************
 * Car.cc defines Car - a non-linear vehicle model.
 *
 * Geraint Paul Bevan <geraint.bevan@gcu.ac.uk>
 * Initial Revision : <2005-06-21>
 * Latest Time-stamp: <2021/02/27 23:29:56 geraint>
 *
 ***********************************************************/

#include <iostream>
#include <cmath>
#include "Crash.hh"
#include "Car.hh"

/// initialises the Car.
/**
 * The mass and moment of inertia are defined along with the
 * moment arms to each of the wheels. Parameters are set to
 * define the lateral tyre characteristics.  The steering
 * and slip angles of each wheel are set to zero, as are the
 * forces and speeds of each wheel.
 */
Car::Car(void) {
  // initial conditions
  position.X = 0.0;
  position.Y = 0.0;
  position.Psi = 0.0;

  velocity.X = 1.0;
  velocity.Y = 0.0;
  velocity.Psi = 0.0;

  acceleration.X = 0.0;
  acceleration.Y = 0.0;
  acceleration.Psi = 0.0;

  // mass and moment of inertia
  m = 2360;			// standard: 1900; v220ts
  Izz = 4700.0;	  // standard 2870.0; v220ts from kfzpw220.h

  // gravitational constant
  g = 9.81;
  
  // friction coefficient
  mu[FL] = 1.0;
  mu[FR] = 1.0;
  mu[RL] = 1.0;
  mu[RR] = 1.0;

  // moment arms to wheels
  X[FL] = 1.67;
  X[FR] = X[FL];

  X[RL] = -1.41;
  X[RR] = X[RL];

  Y[FL] = -0.80;
  Y[FR] = -Y[FL];

  Y[RL] = Y[FL];
  Y[RR] = Y[FR];

  // height of centre of mass
  Z0	= 0.5;

  // tyre lateral parameters
  By =  18.0;
  Cy =  1.0;
  Dy =  0.9;
  Ey = -1.0;

  // aerodynamic drag coefficient
  Cd = 0.22;

  // frontal area
  A = 2.5;

  // air density
  rho = 1.225;
  
  // wheel initialisation
  for (wheel i = FL; i <= RR; ++i) {
    alpha[i] = 0.0;
    delta[i] = 0.0;
    fx[i] = 0.0;
    fy[i] = 0.0;
    vx[i] = 0.0;
    vy[i] = 0.0;
    fz[i] = m * g / 4.0;
  }
}

/// performs one Euler integration step.
/**
 * The vehicle acceleration is updated and then integrated
 * over a period of dt seconds to obtain new vehicle
 * velocities; these are integrated in turn to obtain a new
 * vehicle position.
 */
void
Car::integrate_euler(double dt) {
  update_acceleration();

  velocity.X += dt * acceleration.X;
  velocity.Y += dt * acceleration.Y;
  velocity.Psi += dt * acceleration.Psi;

  position.X += dt * (+ velocity.X * cos(position.Psi)
		      - velocity.Y * sin(position.Psi));

  position.Y += dt * (+ velocity.X * sin(position.Psi)
		      + velocity.Y * cos(position.Psi));

  position.Psi += dt * velocity.Psi;
};

/// returns the acceleration of the Car's centre of gravity.
/**
 * The acceleration components are returned in SI units:
 * metres per second squared and radians per second squared,
 * for translational and rotation units respectively.  These
 * acceleration components are defined relative to the Car's
 * body axis system.
 */
Car::axis
Car::get_acceleration(void) const {
  return acceleration;
}

/// returns the friction coefficient between the tyres and road.
/**
 * A coefficient of 1.0 corresponds to dry asphalt; 0.6 is
 * reasonable for wet roads; for ice it may be 0.05.
 */
double
Car::get_wheel_friction_coefficient(const wheel i) const {
  return mu[i];
}

/// returns the position of the Car's centre of gravity.
/**
 * The position components are returned in SI units: metres
 * and radians, for translational and rotation units
 * respectively.  These position components are defined
 * relative to the Earth axis system in which the Car was
 * initialised.
 */
Car::axis
Car::get_position(void) const {
  return position;
}

/// returns the velocity of the Car's centre of gravity.
 /**
 * The velocity components are returned in SI units: metres
 * per second and radians per second, for translational and
 * rotational units respectively.  These velocity components
 * are defined relative to the Car's body axis system.
 */
Car::axis
Car::get_velocity(void) const {
  return velocity;
}

/// returns the lateral force between a wheel and the road.
/**
 * The force is returned in SI units: Newtons.  This force
 * component is defined relative to the wheel's axis system
 * and is therefore perpendicular to the rolling motion of
 * the wheel.
 */
double
Car::get_wheel_lateral_force(const wheel i) const {
  return fy[i];
}

/// returns the lateral speed of a wheel.
/**
 * The speed is returned in SI units: metres per second.
 * This velocity component is defined relative to the
 * wheel's axis system and is therefore perpendicular to the
 * rolling motion of the wheel.
 */
double
Car::get_wheel_lateral_speed(const wheel i) const {
  return vy[i];
}

/// returns the longitudinal force between a wheel and the road.
/**
 * The force is returned in SI units: Newtons.  This force
 * component is defined relative to the wheel's axis system
 * and is therefore co-linear with the rolling motion of the
 * wheel.
 */
double
Car::get_wheel_longitudinal_force(const wheel i) const {
  return fx[i];
}

/// returns the longitudinal speed of a wheel.
/**
 * The speed is returned in SI units: metres per second.
 * This velocity component is defined relative to the
 * wheel's axis system and is therefore co-linear with the
 * rolling motion of the wheel.
 */
double
Car::get_wheel_longitudinal_speed(const wheel i) const {
  return vx[i];
}

/// returns the wheel slip angle of a wheel relative to the road.
/**
 * The angle is returned in SI units: radians.  The slip
 * angle is the angular difference between the wheel's
 * orientation and its direction of motion.
 */
double
Car::get_wheel_slip_angle(const wheel i) const {
  return alpha[i];
}

/// returns the angle at which a wheel is oriented.
/**
 * The angle is returned in SI units: radians.  The steering
 * angle is defined relative to the Car's body axis system.
 */
double
Car::get_wheel_steering_angle(const wheel i) const {
  return delta[i];
}

/// returns the vertical load on a wheel.
/**
 * The force is returned in SI units: Newtons.
 */
double
Car::get_wheel_vertical_force(const wheel i) const {
  return fz[i];
}

/// sets the acceleration of the Car's centre of gravity.
/**
 * The acceleration components are specified in SI units:
 * metres per second squared and radians per second squared,
 * for translational and rotation units respectively.  These
 * acceleration components are defined relative to the Car's
 * body axis system.
 */
void
Car::set_acceleration(const axis &accel) {
  acceleration = accel;
}

/// sets the friction coefficient between the tyres and road.
/**
 * A coefficient of 1.0 corresponds to dry asphalt; 0.6 is
 * reasonable for wet roads; for ice it may be as 0.05.
 */
void
Car::set_friction_coefficient(const double friction_coefft) {
  mu[FL] = friction_coefft;
  mu[FR] = friction_coefft;
  mu[RL] = friction_coefft;
  mu[RR] = friction_coefft;
}

/// sets the velocity of the Car's centre of gravity.
/**
 * The velocity components are specified in SI units: metres
 * per second and radians per second, for translational and
 * rotational units respectively.  These velocity components
 * are defined relative to the Car's body axis system.  The
 * wheel speeds are updated accordingly.
 */
void
Car::set_velocity(const axis &vel) {
  velocity = vel;
  update_wheel_speeds();
}

/// sets the friction coefficient between a wheel and the road
/**
 * A coefficient of 1.0 corresponds to dry asphalt; 0.6 is
 * reasonable for wet roads; for ice it may be as 0.05.
 */
void
Car::set_wheel_friction_coefficient(const wheel i, const double friction_coefft) {
  mu[i] = friction_coefft;
}


/// sets the lateral force between a wheel and the road.
/**
 * The force is specified in SI units: Newtons.  This force
 * component is defined relative to the wheel's axis system
 * and is therefore perpendicular to the rolling motion of
 * the wheel.
 */
void
Car::set_wheel_lateral_force(const wheel i, const double force) {
  fy[i] = force;
}

/// sets the longitudinal force between a wheel and the road.
/**
 * The force is specified in SI units: Newtons.  This force
 * component is defined relative to the wheel's axis system
 * and is therefore co-linear with the rolling motion of the
 * wheel.
 */
void
Car::set_wheel_longitudinal_force(const wheel i, const double force) {
  fx[i] = force;
}

/// sets the wheel slip angle of a wheel relative to the road.
/**
 * The angle is specified in SI units: radians.  The slip
 * angle is the angular difference between the wheel's
 * orientation and its direction of motion.
 */
void
Car::set_wheel_slip_angle(const wheel i, const double angle) {
  alpha[i] = angle;
}

/// sets the longitudinal speed of a wheel relative to the road.
/**
 * The speed is specified in SI units: metres per second.
 * This velocity component is defined relative to the
 * wheel's axis system and is therefore co-linear with the
 * rolling motion of the wheel.
 */
void
Car::set_wheel_speed(const wheel i, const double speed) {
  vx[i] = speed;
}

/// sets the angle at which the front wheels are oriented.
/**
 * The front wheels are constrained to have the same
 * steering angle. The rear wheels remain fixed straight
 * ahead.  The steering angle is defined relative to the
 * Car's body axis system.
 */
void
Car::set_wheel_steering_angle(const double angle) {
  delta[FL] = angle;
  delta[FR] = angle;
  delta[RL] = 0.0;
  delta[RR] = 0.0;
}

/// sets the vertical load on a wheel.
/**
 * The force is specified in SI units: Newtons.
 */
void
Car::set_wheel_vertical_force(const wheel i, const double force) {
  fz[i] = force;
}

/// updates the vehicle acceleration.
/**
 * Components of acceleration are calculated from the forces
 * generated between the tyres and road at each of the
 * wheels.
 */
void
Car::update_acceleration(void) {
  update_weight_distribution();
  update_tyre_forces();

  axis F = {0.0, 0.0, 0.0};

  for (wheel i = FL; i <= RR; ++i) {

    // traction saturation
    double ft = sqrt(fx[i]*fx[i] + fy[i]*fy[i]);
    double fmax = mu[i] * fz[i];
    if (ft > fmax) {
      std::clog << "Traction saturation" << std::endl;
      fx[i] *= fmax/ft;
      fy[i] *= fmax/ft;
    }    

    // wheel rotation
    double FX = + fx[i]*cos(delta[i]) - fy[i]*sin(delta[i]);
    double FY = + fx[i]*sin(delta[i]) + fy[i]*cos(delta[i]);
    double MZ = -Y[i]*FX + X[i]*FY;
    F.X += FX;
    F.Y += FY;
    F.Psi += MZ;
  }

  // aerodynamic drag
  double VV = velocity.X*velocity.X;
  F.X -= Cd*A*0.5*rho*VV;
  
  // Newton II
  acceleration.X = F.X / m;
  acceleration.Y = F.Y / m;
  acceleration.Psi = F.Psi / Izz;
}

/// updates the lateral tyre forces.
/**
 * Lateral tyre forces are calculated using Pacejka's magic
 * formula and the slip angles at each of the wheels.
 */
void
Car::update_tyre_forces(void) {
  update_wheel_speeds();
  double phi;
  for (wheel i = FL; i <= RR; ++i) {
    if (vx[i] != 0.0) {
      alpha[i] = atan(vy[i] / vx[i]);
    } else if (vy[i] == 0.0) {
      alpha[i] = 0.0;
    } else {
      alpha[i] = 0.5 * M_PI;
      if (vy[i] < 0.0) {
	alpha[i] = - alpha[i];
      }
    }

    // Pacejka's magic formula
    phi = By * alpha[i];
    fy[i] = - Dy * sin(Cy * atan(phi - Ey * (phi - atan(phi))));

    fy[i] *= mu[i] * fz[i];
  }
};

/// dgelss (LAPACK) is used for pseudoinversion
/**
 * See netlib.org for more details about LAPACK
 * http://www.netlib.org/lapack/index.html
 */
extern "C" {
  void dgelss_(int *m, int *n, int *nrhs,
	       double *a, int *lda, double *b, int *ldb,
	       double *s, double *rcond, int *rank,
	       double *w, int* lwork, int *info);
}

/// updates the weight distribution.
/**
 * The vehicle is assumed to be rigid so the moments from
 * vertical loads balance those from lateral and
 * longitudinal forces.
 *
 *       ||  1  1  1  1            m*g  ||
 *  min  || Y1 Y2 Y3 Y4 * Fz  -  -Z0*Fy ||
 *   Fz  || X1 X2 X3 X4          -Z0*Fx ||2
 *
 * dgelss (LAPACK) is used to calculate the pseudoinverse
 */
void
Car::update_weight_distribution(void) {  
  double Fx = m*acceleration.X;
  double Fy = m*acceleration.Y;
  
  int nrow=3, ncol=4, nrhs=4;
  double rcond = 0.0;
  
  double A[3][4] = {		// A will be pseudoinverted
    {  1.0 ,  1.0 ,  1.0 ,  1.0  },
    { Y[FL], Y[FR], Y[RL], Y[RR] },
    { X[FL], X[FR], X[RL], X[RR] }
  };
  double B[4][4] = {		// B will become the answer
    { 1.0, 0.0, 0.0, 0.0 },
    { 0.0, 1.0, 0.0, 0.0 },
    { 0.0, 0.0, 1.0, 0.0 },
    { 0.0, 0.0, 0.0, 1.0 }
  };
  double S[3][4] = {		// singular values
    { 0.0, 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0, 0.0 }
  };

  // Fortran arrays are the transpose of C arrays
  double AT[ncol*nrow], BT[nrhs*nrhs], ST[ncol*nrow];
  for (int i = 0; i < ncol; ++i) {
    for (int j = 0; j < nrow; ++j) {
      AT[j + nrow*i] = A[j][i];
      ST[j + nrow*i] = S[j][i];
    }
  }
  for (int i = 0; i < nrhs; ++i) {
    for (int j = 0; j < nrhs; ++j) {
      BT[j + nrhs*i] = B[j][i];
    }
  }

  // LAPACK: solves min_x || Ax  -b ||
  int lda=nrow, ldb=ncol, lwork=16;
  int rank=0, info=0;
  double work[lwork];
  dgelss_(&nrow, &ncol, &nrhs,
	  AT, &lda, BT, &ldb, ST, &rcond, &rank,
	  work, &lwork, &info);

  for (int i = 0; i < nrhs; ++i) {
    for (int j = 0; j < nrhs; ++j) {
      B[j][i] = BT[j + nrhs*i];	
    }
  } // B contains pseudoinverse of A

  for (wheel i = FL; i <= RR; ++i) {
    fz[i] = B[i][0] * m*g
      + B[i][1] * (-Z0*Fy)
      + B[i][2] * (-Z0*Fx);
  }

#ifdef DEBUG
  for (int i = 0; i < ncol; ++i) {
    for (int j = 0; j < nrow; ++j) {
      A[j][i] = AT[j + nrow*i];
      S[j][i] = ST[j + nrow*i];
    }
  }
#endif
}

/// updates the velocity at each wheel.
/**
 * The velocity of each wheel is calculated from the Car's
 * body velocity and consideration of geometry.
 */
void
Car::update_wheel_speeds(void) {
  for (wheel i = FL; i <= RR; ++i) {
    vx[i] =
      + (velocity.X - Y[i]*velocity.Psi) * cos(delta[i])
      + (velocity.Y + X[i]*velocity.Psi) * sin(delta[i]);
    vy[i] =
      - (velocity.X - Y[i]*velocity.Psi) * sin(delta[i])
      + (velocity.Y + X[i]*velocity.Psi) * cos(delta[i]);
  }
}

/// writes vehicle parameters to standard output.
void
Car::write_parameters(void) const {
  std::cout << "=== Vehicle parameters ===" << std::endl
	    << "m\t" << m << std::endl
	    << "Izz\t" << Izz << std::endl
	    << "mu[FL]\t" << mu[FL] << std::endl
	    << "mu[FR]\t" << mu[FR] << std::endl
	    << "mu[RL]\t" << mu[RL] << std::endl
	    << "mu[RR]\t" << mu[RR] << std::endl
	    << "X[FL]\t" << X[FL] << std::endl
	    << "X[FR]\t" << X[FR] << std::endl
	    << "X[RL]\t" << X[RL] << std::endl
	    << "X[RR]\t" << X[RR] << std::endl
	    << "Y[FL]\t" << Y[FL] << std::endl
	    << "Y[FR]\t" << Y[FR] << std::endl
	    << "Y[RL]\t" << Y[RL] << std::endl
	    << "Y[RR]\t" << Y[RR] << std::endl
	    << "By\t" << By << std::endl
	    << "Cy\t" << Cy << std::endl
	    << "Dy\t" << Dy << std::endl
	    << "Ey\t" << Ey << std::endl
	    << "==========================" << std::endl;
}

/// allows "for" constructs to loop over the wheels.
/**
 * Implementation of the prefix increment operator allows
 * constructs such as:
 *
 * for (wheel i = FL; i < RR; ++i) {
 *    do_something_to_wheel(i);
 * }
*/
Car::wheel &
operator++(Car::wheel &w) {
  int i = w;
  w = static_cast<Car::wheel>(i + 1);
  return w;
}
