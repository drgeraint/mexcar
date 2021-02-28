## -*-octave-*-

clear all

## reference speeds (forwards and yaw)
REF_Vx		= 100/3.6;		# 100 km/hr
REF_Vpsi	= 15*pi/180;		# 5 deg/s

if (~ exist('car'))
  OctCar()
end

## icy
MU_DRY = 1.0;
MU_WET = 0.6;
MU_ICE = 0.01;
OctCar_set_wheel_friction_coefficients([MU_WET,MU_WET,MU_WET,MU_WET]);

OctCar_set_velocity([REF_Vx,0,0]);
OctCar_set_wheel_longitudinal_forces([0,0,130,130]);

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
TIME 		= [0:DT:60];

data 		= zeros(length(TIME),12);

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

  ## ice
  X		= POS(1);
  Y 		= POS(2);
  PSI		= POS(3);
  X_ice_min	= +50;
  X_ice_max	= +100;
  Y_ice_min	= +50;
  Y_ice_max	= +200;
  if ((X_ice_min<X)&&(X<X_ice_max)...
      &&(Y_ice_min<Y)&&(Y<Y_ice_max))
    on_ice = true;
  else
    on_ice = false;
  end
		       

  if (on_ice)
    MU = [MU_ICE,MU_ICE,MU_ICE,MU_ICE];
  else
    MU = [MU_WET,MU_WET,MU_WET,MU_WET];
  end
  OctCar_set_wheel_friction_coefficients(MU);

  W 		= OctCar_get_W();
  data(i,:) 	= [POS', DELTA(1), sum(Fx), VEL', W', on_ice];
end

# write final state
OctCar_write_parameters();

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
ON_ICE		= data(:,12);
NO_ICE		= !ON_ICE;

## plot trajectory
figure(1)
X_no_ice = X(find(NO_ICE));
Y_no_ice = Y(find(NO_ICE));
X_on_ice = X(find(ON_ICE));
Y_on_ice = Y(find(ON_ICE));
plot(Y_no_ice,X_no_ice,'.k',Y_on_ice,X_on_ice,'.b',Y(1),X(1),'xr',Y(end),X(end),'or')
legend({'wet','ice','start','end'},'location','southwest','orientation','horizontal')
xlabel('Lateral position Y [m]')
ylabel('Longitudinal position X [m]')
title('Driving in a circle with ice patch')
axis('square')
ax = axis();
Xmin = ax(3);
Xmax = ax(4);
hold on
plot([Y_ice_min,Y_ice_max],[X_ice_min,X_ice_min],'b')
plot([Y_ice_min,Y_ice_max],[X_ice_max,X_ice_max],'b')
plot([Y_ice_min,Y_ice_min],[X_ice_min,X_ice_max],'b')
plot([Y_ice_max,Y_ice_max],[X_ice_min,X_ice_max],'b')
print -dpng 'path-ice.png'

## plot speed and control inputs
figure(2)
subplot(221)
plot(TIME,delta*180/pi);
ax = axis();
cla
hold on
for t = TIME(find(ON_ICE))
  plot([t,t],[ax(3),ax(4)],'c');
end
plot(TIME,delta*180/pi);
hold off
ylabel('\delta [deg]')
title('Steering angle')
subplot(222)
plot(TIME,FxSum)
ax = axis();
cla
hold on
for t = TIME(find(ON_ICE))
  plot([t,t],[ax(3),ax(4)],'c');
end
plot(TIME,FxSum)
ylabel('\Sigma F_x [N]')
title('Longitudinal force')
subplot(223)
plot(TIME,Vpsi*180/pi,[0,TIME(end)],[REF_Vpsi,REF_Vpsi]*180/pi,':r');
ax = axis();
cla
hold on
for t = TIME(find(ON_ICE))
  plot([t,t],[ax(3),ax(4)],'c');
end
plot(TIME,Vpsi*180/pi,[0,TIME(end)],[REF_Vpsi,REF_Vpsi]*180/pi,':r');
ylabel('V_\psi [deg/s]')
xlabel('Time [s]')
title('Yaw rate')
subplot(224)
plot(TIME,Vx,[0,TIME(end)],[REF_Vx,REF_Vx],':r')
ax = axis();
cla
hold on
for t = TIME(find(ON_ICE))
  plot([t,t],[ax(3),ax(4)],'c');
end
plot(TIME,Vx,[0,TIME(end)],[REF_Vx,REF_Vx],':r')
ylabel('V_x [m/s]')
xlabel('Time [s]')
title('Forward speed')
print -dpng 'control-ice.png'

figure(3)
subplot(311)
plot(TIME(100:end),data(100:end,9))
ax = axis();
cla
hold on
for t = TIME(find(ON_ICE))
  plot([t,t],[ax(3),ax(4)],'c');
end
plot(TIME(100:end),data(100:end,9))
ylabel('dX"/d\mu')
title('Disturbance matrix W with ice patches')
subplot(312)
plot(TIME(100:end),data(100:end,10))
ax = axis();
cla
hold on
for t = TIME(find(ON_ICE))
  plot([t,t],[ax(3),ax(4)],'c');
end
plot(TIME(100:end),data(100:end,10))
ylabel('dY"/d\mu')
subplot(313)
plot(TIME(100:end),data(100:end,11))
ax = axis();
cla
hold on
for t = TIME(find(ON_ICE))
  plot([t,t],[ax(3),ax(4)],'c');
end
plot(TIME(100:end),data(100:end,11))
ylabel('dPsi"/d\mu')
xlabel('Time t [s]')
print -dpng 'W-ice.png'
