%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Filename: kalman.m
% Authors: Sylvain Marleau, Vincent Zalzal
% Description:  This script is a matlab Kalman filter example based on the
%               article : Welch Bishop, An Introduction to the Kalman Filter, 
%               University of North Carolina at Chapel Hill, May 2003.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
global ww N

% loading thrust and measure
% data
load data.mat

q = 0.0001;


% Parameters

% Q = [ 1/3*ww^3   1/3*ww^3   1/3*ww^3   1/3*ww^3   1/2*ww^2   1/2*ww^2   1/2*ww^2; 
%       1/3*ww^3   1/3*ww^3   1/3*ww^3   1/3*ww^3   1/2*ww^2   1/2*ww^2   1/2*ww^2; 
%       1/3*ww^3   1/3*ww^3   1/3*ww^3   1/3*ww^3   1/2*ww^2   1/2*ww^2   1/2*ww^2; 
%       1/3*ww^3   1/3*ww^3   1/3*ww^3   1/3*ww^3   1/2*ww^2   1/2*ww^2   1/2*ww^2; 
%       1/2*ww^2   1/2*ww^2   1/2*ww^2   1/2*ww^2      ww          0          0   ;
%       1/2*ww^2   1/2*ww^2   1/2*ww^2   1/2*ww^2      0           ww         0   ;
%       1/2*ww^2   1/2*ww^2   1/2*ww^2   1/2*ww^2      0           0          ww    ] ;
%  
%   tQ = [ 1/3*ww^3   1/3*ww^3   1/2*ww^2; 
%          1/3*ww^3   1/3*ww^3   1/2*ww^2; 
%          1/2*ww^2   1/2*ww^2       ww   ] ;

  
Q = [ q      q/50   q/50   q/50   q/50   q/50   q/50; 
      q/50   q      q/50   q/50   q/50   q/50   q/50; 
      q/50   q/50   q      q/50   q/50   q/50   q/50; 
      q/50   q/50   q/50   q      q/50   q/50   q/50; 
      q/50   q/50   q/50   q/50   q      0      0   ;
      q/50   q/50   q/50   q/50   0      q      0   ;
      q/50   q/50   q/50   q/50   0      0      q  ] ;
 
  tQ = [ q      q/50   q/50; 
         q/50   q      q/50; 
         q/50   q/50   q   ] ;
 
 
 W = [ 1 0 0 0 0 0 0; 
       0 1 0 0 0 0 0;
       0 0 1 0 0 0 0;
       0 0 0 1 0 0 0;
       0 0 0 0 1 0 0;
       0 0 0 0 0 1 0;
       0 0 0 0 0 0 1 ];

V = [1 0 0 0 0 0 0;
     0 1 0 0 0 0 0;
     0 0 1 0 0 0 0;
     0 0 0 1 0 0 0;
     0 0 0 0 1 0 0;
     0 0 0 0 0 1 0;
     0 0 0 0 0 0 1 ];
 

%  sum = measures(:,1);
%  
%  for t = 2:N
%      sum = sum +  measures(:,t);
%  end
%  sum = sum / N;
%  sum2=zeros(6,1);
%  for t = 1:N
%     dif = measures(:,t)-sum;
%     dif = dif.^2;
%     sum2 = sum2 + dif;
%  end
% r = sum2/N
 

 
 
 
 R = [  0.01    0       0       0       0       0       0;
        0       0.01    0       0       0       0       0;
        0       0       0.01    0       0       0       0;
        0       0       0       0.01    0       0       0;
        0       0       0       0       0.01    0       0;
        0       0       0       0       0       0.01    0;
        0       0       0       0       0       0       0.01     
      ];   %% covariance matrix 
  
  tR = [  0.01      0        ;
          0        0.000022   ];   %% covariance matrix 

% initial estimate
x = [ EulerToQuaternion(measures(1:3,1));measures(4:6,1)];
tx = [tMeasures(1,1); 0; tMeasures(2,1)];

% estim?de l'erreur initiale
% P = [100^2     0       0       0;
%      0         10^2    0      0;
%      0         0       25^2    0;
%      0         0       0       10^2];
% 

P =  0.1*eye(7);
tP = 0.1*eye(3);

trajectory_kalman = zeros(6,N);
ttrajectory_kalman = zeros(3,N);
trajectory_kalman(:,1) = measures(:,1);
ttrajectory_kalman(:,1) = [tMeasures(1,1), 0, tMeasures(2,1)];

for t=2:N
%% Rotation
% Prediction

%xw = model(x,F(1:3,t));  %euler angle
%x_ = [EulerToQuaternion(xw(1:3));xw(4:6)];

A = createA(x);
x_ = A*x;
P_ = A*P*A' + W*Q*W';

% Error prediction 
H = createH();

% Kalman's gain
K = P_*H'/(H*P_*H'+V*R*V');

% measure estimate
% estime= [QuaternionToEuler(x_(1:4)); x_(5:7)];
estime = H*x_;

z = [EulerToQuaternion(measures(1:3,t));measures(4:6,t)];

correction = K*(z - estime);

% New estimate of state vector
x = x_ + correction;

% Error estimate
P = (eye(7) - K*H)*P_;

trajectory_kalman(:,t) = [QuaternionToEuler(x(1:4)); x(5:7)];
%trajectory_kalman(:,t) = [QuaternionToEuler(x_(1:4)); x_(5:7)];
%trajectory_kalman(:,t) = estime(:);
%trajectory_kalman(:,t) = [QuaternionToEuler(correction(1:4)); correction(5:7)];

%% Translation
% Prediction
tA = createtA();
tx_ = tA*tx ;%+ [ 0; 0; 1]*tF(1,t);
tP_ = tA*tP*tA' + tQ;

% Error prediction 
tH = createtH();

% Kalman's gain
tK = tP_*tH'/(tH*tP_*tH'+tR);

% measure estimate
testime= tH*tx_;
%estime=H*x_;
tcorrection = tK*(tMeasures(:,t) - testime);

% New estimate of state vector
tx = tx_ + tcorrection;

% Error estimate
tP = (eye(3) - tK*tH)*tP_;

ttrajectory_kalman(:,t) = tx;
%ttrajectory_kalman(:,t) = testime;


end

 fid = fopen('trajectory_matlab.m', 'w');
% 
 exportMatlab(fid, 'trajectory_kalman', trajectory_kalman);
 exportMatlab(fid, 'ttrajectory_kalman', ttrajectory_kalman);
% 
 fclose(fid);
c = 'complete'