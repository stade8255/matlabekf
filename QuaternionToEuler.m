function EULER = QuaternionToEuler(q)

R = [ q(1)^2+q(2)^2-q(3)^2-q(4)^2   2*(q(2)*q(3)-q(1)*q(4))        2*(q(2)*q(4)+q(1)*q(3));
      2*(q(2)*q(3)+q(1)*q(4))       q(1)^2-q(2)^2+q(3)^2-q(4)^2   2*(q(3)*q(4)-q(1)*q(2));
      2*(q(2)*q(4)-q(1)*q(3))       2*(q(3)*q(4)+q(1)*q(2))        q(1)^2-q(2)^2-q(3)^2+q(4)^2 ];

tmp = abs(R(3,1)); 

if tmp > 0.999999 
    x = 0;                              %roll
    y = -((pi/2) * R(3,1)/tmp);         %pitch
    z = atan2(-R(1,2), -R(3,1)*R(1,3)); %yaw
else
    x = atan2(R(3,2), R(3,3));
    y = asin(-R(3,1)); 
    z = atan2(R(2,1), R(1,1));
    
end

EULER = [ x;
          y; 
          z ];