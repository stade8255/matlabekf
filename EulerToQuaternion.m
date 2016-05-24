function Quaternion = EulerToQuaternion(angle)

w = cos(angle(1)/2)*cos(angle(2)/2)*cos(angle(3)/2)+sin(angle(1)/2)*sin(angle(2)/2)*sin(angle(3)/2);
x = sin(angle(1)/2)*cos(angle(2)/2)*cos(angle(3)/2)-cos(angle(1)/2)*sin(angle(2)/2)*sin(angle(3)/2);
y = cos(angle(1)/2)*sin(angle(2)/2)*cos(angle(3)/2)+sin(angle(1)/2)*cos(angle(2)/2)*sin(angle(3)/2);
z = cos(angle(1)/2)*cos(angle(2)/2)*sin(angle(3)/2)-sin(angle(1)/2)*sin(angle(2)/2)*cos(angle(3)/2);

Quaternion = [ w;
               x;
               y;
               z ];