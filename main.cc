// -*-c++-*-

/**********************************************************************
 * main.cc
 *
 * Geraint Paul Bevan <geraint.bevan@gcu.ac.uk>
 * Initial Revision : <2005-06-21>
 * Latest Time-stamp: <2021/02/26 02:31:56 geraint>
 ***********************************************************************/

#include <cmath>
#include <fstream>
#include <string>
#include "LinearisableCar.hh"

void
plot_trajectories(void);

/** Comparison of Car and LinearisableCar behaviour.
 * Performs identical simulations using a Car and a
 * LinearisableCar so that the results can be compared.
 * The results are written to two data files:
 * "nonlinear.dat" and "linear.dat".
 */
int main(void) {

  Car *cars[2];
  
  cars[0] = new(Car);
  cars[1] = new(LinearisableCar);

  std::ofstream *datafiles[2];

  datafiles[0] = new(std::ofstream);
  datafiles[1] = new(std::ofstream);

  datafiles[0]->open("nonlinear.dat");
  datafiles[1]->open("linear.dat");

  for (int c = 0; c < 2; c++) {
    cars[c]->set_wheel_longitudinal_force(Car::FL, 2000.0);
    cars[c]->set_wheel_longitudinal_force(Car::FR, 2000.0);
    cars[c]->set_wheel_longitudinal_force(Car::RL, 2000.0);
    cars[c]->set_wheel_longitudinal_force(Car::RR, 2000.0);

    Car::axis p;

    double tlast = 3.0;
    double tstep = 0.1;
    for (double t = 0.0; t < tlast; t += tstep) {
      double frac = t / tlast;
      cars[c]->set_wheel_steering_angle(frac*M_PI/8);    
      cars[c]->integrate_euler(tstep);
      
      p = cars[c]->get_position();
      *datafiles[c] << t
		    << '\t' << p.X
		    << '\t' << p.Y
		    << '\t' << p.Psi
		    << std::endl;
    }  
    datafiles[c]->close();
  }
  plot_trajectories();
  return 0;
}

/** plots the trajectories resulting from the simulation. */
void
plot_trajectories(void) {
  const char *command =
    "echo \"\
set xlabel 'Y [m]';\
set ylabel 'X [m]';\
plot \
'nonlinear.dat' using 3:2 title 'nonlinear' with linespoints,\
'linear.dat'    using 3:2 title    'linear' with linespoints\"\
| gnuplot -persist";
  system(command);
}
