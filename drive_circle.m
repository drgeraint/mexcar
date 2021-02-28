## -*-octave-*-

clear all

## reference speeds (forwards and yaw)
REF_Vx		= 100/3.6;		# 100 km/hr
REF_Vpsi	= 15*pi/180;		# 15 deg/s

if (~ exist('car'))
  OctCar()
end
OctCar_set_velocity([REF_Vx,0,0]) 
OctCar_set_wheel_longitudinal_forces([0,0,130,130])

disp('Initial conditions')
VEL		= OctCar_get_velocity()
DELTA		= OctCar_get_wheel_steering_angles()
Fx		= OctCar_get_wheel_longitudinal_forces()
Fy		= OctCar_get_wheel_lateral_forces()
A		= OctCar_get_A()
B		= OctCar_get_B()
W		= OctCar_get_W()

OctCar_set_wheel_steering_angle(3*pi/180) # 3 degree steering angle

DT 		= 0.1;
TIME 		= [0:DT:300];

data 		= zeros(length(TIME),11);

for i = 1:length(TIME)
  OctCar_integrate_euler_nonlinear(DT,1);
  POS		= OctCar_get_position(); # X, Y, psi
  VEL		= OctCar_get_velocity();
  DELTA		= OctCar_get_wheel_steering_angles()(1);
  Fx		= OctCar_get_wheel_longitudinal_forces();

  ## control of forward speed using rear wheel forces
  ERR_Vx	= REF_Vx - VEL(1);
  dFx		= 2.0*ERR_Vx;
  Fx		+= [0;0;dFx;dFx];
  OctCar_set_wheel_longitudinal_forces(Fx);

  ## control of yaw rate using front wheel steering angle
  ERR_Vpsi 	= REF_Vpsi-VEL(3);
  dDELTA 	= 1e-2*ERR_Vpsi;
  DELTA 	+= dDELTA;
  OctCar_set_wheel_steering_angle(DELTA);

  W		= OctCar_get_W();
  data(i,:) 	= [POS', DELTA(1), sum(Fx), VEL', W'];
end

# write final state
VEL		= OctCar_get_velocity()
DELTA		= OctCar_get_wheel_steering_angles()
Fx		= OctCar_get_wheel_longitudinal_forces()
FY		= OctCar_get_wheel_lateral_forces()
A		= OctCar_get_A()
B		= OctCar_get_B()
W		= OctCar_get_W()

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
legend({'path (dry)','start','end'},'location','southwest','orientation','horizontal')
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

figure(3)
subplot(311)
plot(TIME(100:end),data(100:end,9))
ylabel('dX"/d\mu')
title('Disturbance matrix W');
subplot(312)
plot(TIME(100:end),data(100:end,10))
ylabel('dY"/d\mu')
subplot(313)
plot(TIME(100:end),data(100:end,11))
ylabel('dPsi"/d\mu')
xlabel('Time t [s]')
print -dpng 'W.png'
