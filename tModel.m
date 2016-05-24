function x_ = tModel( x, F)

global w tw1 tw2 tw3 samplingtime
x_ = zeros(3,1);    % x xdot zmp
x_(1) = x(1) + samplingtime*x(2) + random('Normal', 0 ,tw1);
x_(2) = w*w*samplingtime*x(1) + x(2) - w*w*samplingtime*x(3) + random('Normal', 0 ,tw2);
x_(3) = x(3) + random('Normal', 0 ,tw3) + F; 


 



%  q(3)+random('Normal', 0 ,w3) + F  %%加入CONTROL INPUT 為ZMP之微分 