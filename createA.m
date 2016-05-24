function A = createA(q)

global  samplingtime 

% q(1) = q0
% q(2) = q1
% q(3) = q2 
% q(4) = q3
% q(5) = wx
% q(6) = wy
% q(7) = wz
%ww = sqrt( 980/ 66.6) ;


deltatheta = samplingtime*(q(5)^2+q(6)^2+q(7)^2)^0.5;


A = [ cos(1/2*deltatheta)            0                   0                  0            -q(2)*1/2*samplingtime   -q(3)*1/2*samplingtime   -q(4)*1/2*samplingtime;
               0           cos(1/2*deltatheta)           0                  0             q(1)*1/2*samplingtime   -q(4)*1/2*samplingtime    q(3)*1/2*samplingtime;
               0                     0         cos(1/2*deltatheta)          0             q(4)*1/2*samplingtime    q(1)*1/2*samplingtime    q(2)*1/2*samplingtime;
               0                     0                   0          cos(1/2*deltatheta)  -q(3)*1/2*samplingtime    q(2)*1/2*samplingtime    q(1)*1/2*samplingtime;
               0                     0                   0                  0                      1                      0                      0         ;
               0                     0                   0                  0                      0                      1                      0         ;
               0                     0                   0                  0                      0                      0                      1         

];