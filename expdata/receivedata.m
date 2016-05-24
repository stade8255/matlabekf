clear;

load '.\Control_IMU\absangle_roll.txt';
load '.\Control_IMU\absangle_pitch.txt';
load '.\Control_IMU\accel_pitch.txt';
load '.\Control_IMU\accel_roll.txt';
absx = load ('.\A IMU\A absanglex.txt');
absx = absx(2100:15279,1);
load '.\A push_recovery\FilterAccelLateral.txt';
load '.\A push_recovery\FilterAccelSagittal.txt';
load '.\A push_recovery\COG\AgEnc_FKCOG.txt';
load '.\A push_recovery\COG\AngleX.txt';
load '.\A push_recovery\COG\AngleY.txt';
load '.\A push_recovery\COG\Estimated_COG_lateral.txt';
load '.\A push_recovery\COG\Estimated_COG_sagittal.txt';
load '.\A push_recovery\COG\force_data_ZMP.txt';
load '.\A push_recovery\COG\ZMP_lateral.txt';
load '.\A push_recovery\COG\ZMP_sagittal.txt';


time = 0.005:0.005:60;
time2 = 0.005:0.005:0.005*size(AgEnc_FKCOG);        %A_absanglex
time3 = 0.005:0.005:0.005*size(force_data_ZMP);      %A_finalanglex
time4 = 0.005:0.005:0.005*size(absx);  
% time4 = 0.004:0.004:0.004*size(A_finalanglex_org);  %A_finalanglex_org
% time5 = 0.004:0.004:0.004*size(IMUaccelx);          %IMUaccelx
% time6 = 0.004:0.004:0.004*size(IMUaccelx_org);      %IMUaccelx_org
% time7 = 0.005:0.005:0.005*size(AgEnc_FKCOG);        %AgEnc_FKCOG

figure(1)
subplot(4,1,1);
plot(time,  absangle_roll(:,1)*180/3.14,time, AngleX(:,1)*180/3.14)
title('angleX');
legend( 'absangleX', 'AngleX');

subplot(4,1,2);
plot(time,  absangle_pitch(:,1)*180/3.14,time, AngleY(:,1)*180/3.14)
title('angleY');
legend( 'absangleY', 'AngleY');

subplot(4,1,3);
plot(time, accel_roll(:,1), time, FilterAccelLateral(:,1) )
title('AccelLateral');
legend('accel Lateral', 'FilterAccelLateral');

subplot(4,1,4);
plot(time, accel_pitch(:,1), time, FilterAccelSagittal(:,1))
title('AccelSagittal');
legend('accel Sagittal', 'FilterAccelSagittal');


figure(2)
subplot(4,1,1);
plot(time, Estimated_COG_lateral(:,1)*100, time2, AgEnc_FKCOG(:,2) )
title('COG Lateral');
legend('Estimated COG Lateral', 'AgEnc FKCOG');

subplot(4,1,2);
plot(time, ZMP_lateral(:,1), time3, force_data_ZMP(:,2)  )
title('ZMP Lateral');
legend('ZMP Lateral', 'force data ZMP');

subplot(4,1,3);
plot(time, Estimated_COG_sagittal(:,1)*100, time2, AgEnc_FKCOG(:,1)  )
title('COG Sagittal');
legend('Estimated COG Sagittal', 'AgEnc FKCOG');

subplot(4,1,4);
plot(time, ZMP_sagittal(:,1), time3, force_data_ZMP(:,1)  )
title('ZMP Sagittal');
legend('ZMP Sagittal', 'force data ZMP');

