function n_q = model(q, control)

global  w1 w2  w3 w4 w5 w6 w7  samplingtime




% q = [ EulerToQuaternion(angle(1:3)); angle(4:6)];


deltatheta = samplingtime*sqrt(q(5)*q(5)+q(6)*q(6)+q(7)*q(7));

% cos(1/2*deltatheta)*q(1) - q(2)*sin(1/2*deltatheta)*samplingtime/deltatheta*q(5) - q(3)*sin(1/2*deltatheta)*samplingtime/deltatheta*q(6) - q(4)*sin(1/2*deltatheta)*samplingtime/deltatheta*q(7) + random('Normal', 0 ,w1) ;
%cos(1/2*deltatheta)*q(2) + q(1)*sin(1/2*deltatheta)*samplingtime/deltatheta*q(5) - q(4)*sin(1/2*deltatheta)*samplingtime/deltatheta*q(6) + q(3)*sin(1/2*deltatheta)*samplingtime/deltatheta*q(7) + random('Normal', 0 ,w2) ;
%cos(1/2*deltatheta)*q(3) + q(4)*sin(1/2*deltatheta)*samplingtime/deltatheta*q(5) + q(1)*sin(1/2*deltatheta)*samplingtime/deltatheta*q(6) + q(2)*sin(1/2*deltatheta)*samplingtime/deltatheta*q(7) + random('Normal', 0 ,w3);
%cos(1/2*deltatheta)*q(4) - q(3)*sin(1/2*deltatheta)*samplingtime/deltatheta*q(5) + q(2)*sin(1/2*deltatheta)*samplingtime/deltatheta*q(6) + q(1)*sin(1/2*deltatheta)*samplingtime/deltatheta*q(7) + random('Normal', 0 ,w4) ;
        


n_q = [cos(1/2*deltatheta)*q(1) - q(2)*1/2*samplingtime*q(5) - q(3)*1/2*samplingtime*q(6) - q(4)*1/2*samplingtime*q(7) + random('Normal', 0 ,w1) ;
        cos(1/2*deltatheta)*q(2) + q(1)*1/2*samplingtime*q(5) - q(4)*1/2*samplingtime*q(6) + q(3)*1/2*samplingtime*q(7) + random('Normal', 0 ,w2) ;
        cos(1/2*deltatheta)*q(3) + q(4)*1/2*samplingtime*q(5) + q(1)*1/2*samplingtime*q(6) + q(2)*1/2*samplingtime*q(7) + random('Normal', 0 ,w3);
        cos(1/2*deltatheta)*q(4) - q(3)*1/2*samplingtime*q(5) + q(2)*1/2*samplingtime*q(6) + q(1)*1/2*samplingtime*q(7) + random('Normal', 0 ,w4) ;
        q(5) + control(4) + random('Normal', 0 ,w5);
        q(6) + control(5) + random('Normal', 0 ,w6);
        q(7) + control(6) + random('Normal', 0 ,w7)
    ];
% n_q = [ QuaternionToEuler(n_q_(1:4)); n_q_(5:7)];

 
% 	x_(1) = x(1) +  samplingtime*x(2);
% 	x_(2) = ww*ww*samplingtime*x(1)+x(2)-ww*ww*samplingtime*x(3);
% 	x_(3) = x_(3); 


%  q(3)+random('Normal', 0 ,w3) + F  %%加入CONTROL INPUT 為ZMP之微分 