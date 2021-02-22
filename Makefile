all: .autoclean OctCar.oct OctCar_get_A.oct OctCar_get_B.oct OctCar_get_position.oct OctCar_get_velocity.oct OctCar_get_acceleration.oct OctCar_get_friction_coefficient.oct OctCar_get_wheel_lateral_forces.oct OctCar_get_wheel_lateral_speeds.oct OctCar_get_wheel_longitudinal_forces.oct OctCar_get_wheel_longitudinal_speeds.oct OctCar_get_wheel_slip_angles.oct OctCar_get_wheel_steering_angles.oct OctCar_get_wheel_vertical_forces.oct OctCar_integrate_euler_nonlinear.oct OctCar_integrate_euler_linear.oct OctCar_set_acceleration.oct OctCar_set_friction_coefficient.oct OctCar_set_velocity.oct OctCar_set_wheel_longitudinal_forces.oct OctCar_set_wheel_slip_angles.oct OctCar_set_wheel_speeds.oct OctCar_set_wheel_steering_angle.oct OctCar_set_wheel_vertical_forces.oct OctCar_write_parameters.oct


OctCar.oct: OctCar.cc Car.cc LinearisableCar.cc
	mkoctfile -DOCTAVE -I. $^

OctCar_%.oct: OctCar.oct
	ln -s $< $@

view: car
	./car
	sleep 10s

car: main.cc Car.o LinearisableCar.o
	g++ -g -o $@ $^ -llapack # -lg2c

Car.o: Car.cc Car.hh
	g++ -c -g -o $@ $<

.autoclean: Makefile
	make clean
	touch .autoclean

clean:
	rm -f *.oct *.o car *.dat *~ .autoclean
