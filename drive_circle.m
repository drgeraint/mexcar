## -*-octave-*-

clear all

## reference speeds (forwards and yaw)
REF_Vx		= 100/3.6;		# 100 km/hr
REF_Vpsi	= 5*pi/180;		# 5 deg/s

if (~ exist('car'))
  OctCar()
end
OctCar_set_velocity([REF_Vx,0,0]) 
OctCar_set_wheel_longitudinal_forces([0,0,0,0])

disp('Initial conditions')
VEL		= OctCar_get_velocity()
DELTA		= OctCar_get_wheel_steering_angles()
Fx		= OctCar_get_wheel_longitudinal_forces()
Fy		= OctCar_get_wheel_lateral_forces()
A		= OctCar_get_A()
B		= OctCar_get_B()

OctCar_set_wheel_steering_angle(3*pi/180) # 3 degree steering angle

DT 		= 0.2;
TIME 		= [0:DT:600];

data 		= zeros(length(TIME),8);

for i = 1:length(TIME)
  OctCar_integrate_euler_nonlinear(DT,1);
  POS		= OctCar_get_position(); # X, Y, psi
  VEL		= OctCar_get_velocity();
  DELTA		= OctCar_get_wheel_steering_angles()(1);
  Fx		= OctCar_get_wheel_longitudinal_forces();

  ## control of forward speed using rear wheel forces
  ERR_Vx	= REF_Vx - VEL(1);
  dFx		= 0.1*ERR_Vx;
  Fx		+= [0;0;dFx;dFx];
  OctCar_set_wheel_longitudinal_forces(Fx);

  ## control of yaw rate using front wheel steering angle
  ERR_Vpsi 	= REF_Vpsi-VEL(3);
  dDELTA 	= 1e-2*ERR_Vpsi;
  DELTA 	+= dDELTA;
  OctCar_set_wheel_steering_angle(DELTA);

  data(i,:) = [POS', DELTA(1), sum(Fx), VEL'];

end

# write final state
VEL		= OctCar_get_velocity()
DELTA		= OctCar_get_wheel_steering_angles()
Fx		= OctCar_get_wheel_longitudinal_forces()
FY		= OctCar_get_wheel_lateral_forces()
A		= OctCar_get_A()
B		= OctCar_get_B()

X		= data(:,1);
Y		= data(:,2);
PSI		= data(:,3);
delta		= data(:,4);
FxSum		= data(:,5);
Vx		= data(:,6);
Vy		= data(:,7);
Vpsi		= data(:,8);

## plot trajectory
figure(1)
plot(Y,X,'.k',Y(1),X(1),'xr',Y(end),X(end),'or')
legend({'path','start','end'},'location','southwest')
xlabel('Lateral position Y [m]')
ylabel('Longitudinal position X [m]')
title('Driving in a circle')
axis('square')
print -dpng 'path.png'

## plot speed and control inputs
figure(2)
subplot(221)
plot(TIME,delta*180/pi);
ylabel('\delta [deg]')
title('Steering angle')
subplot(222)
plot(TIME,FxSum)
ylabel('\Sigma F_x [N]')
title('Longitudinal force')
subplot(223)
plot(TIME,Vpsi*180/pi,[0,TIME(end)],[REF_Vpsi,REF_Vpsi]*180/pi,':r');
ylabel('V_\psi [deg/s]')
xlabel('Time [s]')
title('Yaw rate')
subplot(224)
plot(TIME,Vx,[0,TIME(end)],[REF_Vx,REF_Vx],':r')
ylabel('V_x [m/s]')
xlabel('Time [s]')
title('Forward speed')
print -dpng 'control.png'
