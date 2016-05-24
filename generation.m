%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Filename: generation.m
% Authors: Sylvain Marleau, Vincent Zalzal
% Description:  This script simulates the model and it generate mesures for
%               the matlab Kalman filter example.
%
% A plane flights in a 2D space where the x axis is the distance traveled
% by the plane and y axis is its altitude.  This system can be represented
% by the fallowing equations:
% (This is just an example)
%
% xpp = F/m - bx/m * xp^2
% ypp = p/m * xp^2 - g
%
% where m is the plane's weight (1000 kg)
%       bx is the drag coefficient (0.35 N/m?s?
%       p is the lift force (3.92 N/m?s?
%       g is the gravitational acceleration (9.8 m/s?
%       F is the motor's thrust
%
% A station on the ground (at the origin) mesures the angle between the
% plane and the ground (x axis) and the distance between the plane and the station.
% These measures are based and the fallowing equations:
%
% theta = atan2(y,x)
% r = sqrt(x^2+y^2)
%
% The variance error matrix of the mesures is:
%
% R = [0.01^2  0
%      0       50^2]
%
% The variance error matrix of the plane's model is: WQW'
%
% Q = [0.01^2    0;
%      0         0.01^2];
%
% W = [0 0;
%      1 0;
%      0 0;
%      0 1];
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
global N T m b p w ww tw1 tw2 tw3 w1 w2 w3 w4 w5 w6 w7 g samplingtime sim

% Nb samples and sampling periode
N = 12000;
T = 0.005;
samplingtime = 0.005;

% Parameters
m = 1000;
b = 0.35;
p = 3.92;
g = [0;0;9.8];
w = sqrt( 980/ 66.6) ;
ww = 0.005;

sim = 0;    % 0 : experence, 1 : simulation

% time vector generation
time = zeros(1,N);
for t=1:N
    time(t) = t*T;
end

%%  simulation
if sim == 1
% noises
br = 0.01;
bt = 0.1;      
bd = 50;   
tw1 = 0.00001;
tw2 = 0.00001;
tw3 = 0.00001;
w1 = 0.00001;
w2 = 0.00001;
w3 = 0.00001;
w4 = 0.00001;
w5 = 0.00001;
w6 = 0.00001;
w7 = 0.00001;




% generation of the plane thrust vector
F = zeros(7,N);
tF = zeros(1,N);
for t=1:N
%     F(1,t) = 0;
%     F(2,t) = 0;
%     F(3,t) = 0;
%     F(4,t) = 0;
%     F(5,t) = 0;
%     F(6,t) = 0.1*pi/180;
end

% Initiales conditions [x xdot xzmp]
tx0 = [1; 0; 0];

% Initiales conditions [x(roll) y(pitch) z(yaw) wx wy wz]
x0 = [ 0.5; 0; 0; 0.5; 0.2; 0];

% AngleToEuler
angle = zeros(3,1);
angle(:,1) = x0(1:3,1);
q0 = [EulerToQuaternion(angle); x0(4:6,1)];

% cc =  QuaternionToEuler(q0(1:4))*180/pi;  %test eulertoquaternion

% Simulation and generation of the rotation and translation trajectory
% rotation
trajectory = zeros(7,N);
trajectory(:,1) = q0;
% translation
tTrajectory = zeros(3,N);
tTrajectory(:,1) = tx0;


% Generation of measures
measures = zeros(6,N);     %rotation (anglex, angley, anglez, wx, wy, wz)
tMeasures = zeros(3,N);    %translation (cog, zmp, acc)
trueRotations = zeros(6,N);      %rotation ref (anglex, angley, anglez, wx, wy, wz)
trueTranslations = zeros(3,N);   %translation ref (cog, zmp, acc)


for t=2:N
%     if t==2
%         tF(1,t) = (tTrajectory(3,t-1)-0)/0.2;
%     else
%         tF(1,t)= (tTrajectory(3,t-1)-tTrajectory(3,t-2))/0.2;
%     end
    trajectory(:,t) =  model(trajectory(:,t-1), F(:,t));
    tTrajectory(:,t) =  tModel(tTrajectory(:,t-1), tF(t));

end



tH = createtH();
for t=1:N
    measures(:,t) = [QuaternionToEuler(trajectory(1:4,t));trajectory(5:7,t)] + random('Normal', 0, br);
    trueRotations(:,t) = [QuaternionToEuler(trajectory(1:4,t));trajectory(5:7,t)];
    tMeasures(:,t) = tH*tTrajectory(:,t) + random('Normal', 0, bt);
    trueTranslations(:,t) = tH*tTrajectory(:,t);
    
end 


% Generation of the measured trajectory
% trajectory_measure = zeros(6,N);
% tTrajectory_measure = zeros(3,N);
% for t=1:N
%    trajectory_measure(:,t) = measures(:,t);
%    tTrajectory_measure(:,t) = tMeasures(:,t);
% end

% rotation figure
figure(1);
subplot(3,1,1);
plot(time, trueRotations(4,:), time, measures(4,:) );
title('Real and measured plane''s wx');
legend('Real x', 'Measured x');

subplot(3,1,2);
plot(time, trueRotations(5,:), time, measures(5,:) );
title('Real and measured plane''s wy');
legend('Real y', 'Measured y');

subplot(3,1,3);
plot(time, trueRotations(6,:), time, measures(6,:) );
title('Real and measured plane''s wz');
legend('Real z', 'Measured z');

figure(2);
subplot(3,1,1);
plot(time, trueRotations(1,:)*180/pi, time, measures(1,:)*180/pi );
title('Real and measured plane''s anglex');
legend('Real x', 'Measured x');

subplot(3,1,2);
plot(time, trueRotations(2,:)*180/pi, time, measures(2,:)*180/pi );
title('Real and measured plane''s angley');
legend('Real y', 'Measured y');

subplot(3,1,3);
plot(time, trueRotations(3,:)*180/pi, time, measures(3,:)*180/pi );
title('Real and measured plane''s anglez');
legend('Real z', 'Measured z');

% figure(3);
% plot(time, F(1,:),time, F(2,:),time, F(3,:),time, F(4,:),time, F(5,:),time, F(6,:))
% title('Thrust');

% translation figure
figure(4);
plot(time, trueTranslations(1,:), time, tMeasures(1,:), time, trueTranslations(2,:), time, tMeasures(2,:), time, trueTranslations(3,:) , time, tMeasures(3,:));
title('Real and measured cog zmp accel');

% figure(5);
% plot(time, tF(1,:))
% title('Thrust');



% anglex=[time;measures(1,:)];
% angley=[time;measures(2,:)];
% anglez=[time;measures(3,:)];

%save data
save data.mat time F measures trajectory tMeasures trueTranslations tF trueRotations


fid = fopen('data.m', 'w');

exportMatlab(fid, 'F', F);
%exportMatlab(fid, 'trajectory_kalman', trajectory_kalman);
exportMatlab(fid, 'measures', measures);

fclose(fid);

%%  experience
else
measures = zeros(6,N);     %rotation (anglex, angley, anglez, wx, wy, wz)
tMeasures = zeros(2,N);    %translation (cog, acc)
load('.\expdata\Control_IMU\absangle_roll.txt')
load('.\expdata\Control_IMU\absangle_pitch.txt')
load('.\expdata\Control_IMU\accel_roll.txt')
load('.\expdata\Control_IMU\accel_pitch.txt')
load('.\expdata\A push_recovery\COG\AngleX.txt')    % 角度ekf完之angle
load('.\expdata\A push_recovery\COG\AngleY.txt')    % 角度ekf完之angle
load('.\expdata\A push_recovery\FilterAccelLateral.txt')    % 角度ekf完之accel
load('.\expdata\A push_recovery\FilterAccelSagittal.txt')   % 角度ekf完之accel
load('.\expdata\A push_recovery\COG\AgEnc_FKCOG.txt')       % encoder
load('.\expdata\A push_recovery\COG\Estimated_COG_lateral.txt')     % ekf完之COG
load('.\expdata\A push_recovery\COG\Estimated_COG_sagittal.txt')    % ekf完之COG

for t=1:N
    measures(:,t) = [ absangle_roll(t), absangle_pitch(t), 0, 0, 0, 0];
   
    if t <= 11617
        tMeasures(:,t) = [ AgEnc_FKCOG(t,2), FilterAccelLateral(t)*1000 ];
    else 
        tMeasures(:,t) = [0, FilterAccelLateral(t)*1000];
    end
    
end 
% rotation figure
figure(1);
subplot(3,1,1);
plot(time, measures(4,:) );
title(' measured plane''s wx');
legend( 'Measured x');

subplot(3,1,2);
plot( time, measures(5,:) );
title(' measured plane''s wy');
legend( 'Measured y');

subplot(3,1,3);
plot(time, measures(6,:) );
title('measured plane''s wz');
legend( 'Measured z');

figure(2);
subplot(3,1,1);
plot(time, measures(1,:)*180/pi );
title(' measured plane''s anglex');
legend('Measured x');

subplot(3,1,2);
plot(time, measures(2,:)*180/pi );
title('measured plane''s angley');
legend('Measured y');

subplot(3,1,3);
plot( time, measures(3,:)*180/pi );
title(' measured plane''s anglez');
legend( 'Measured z');

% figure(3);
% plot(time, F(1,:),time, F(2,:),time, F(3,:),time, F(4,:),time, F(5,:),time, F(6,:))
% title('Thrust');

% translation figure
figure(3);
subplot(2,1,1)
plot( time, tMeasures(1,:) );
title(' measured cog ');

subplot(2,1,2)
plot( time, tMeasures(2,:) );
title(' measured accel');

%save data
save data.mat time measures tMeasures

fid = fopen('data.m', 'w');

exportMatlab(fid, 'measures', measures);

fclose(fid);

end

