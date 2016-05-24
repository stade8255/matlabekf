%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Filename: result.m
% Authors: Sylvain Marleau, Vincent Zalzal
% Description:  This script shows filtered trajectories and compares the
%               common kalman filter with the UDU kalman filter.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
global sim

% Load data
load data.mat
trajectory_matlab


% figure(1);
% plot(trajectory(1,:), trajectory(3,:), '--', trajectory_measure(1,:), trajectory_measure(3,:), ':', trajectory_kalman(1,:), trajectory_kalman(3,:), '-');
% title('Real, Measured and Kalman Trajectories');
% axis([-2500 2500 0 400]);
% legend('Real', 'Measured', 'Kalman');
% xlabel('Dist ance (m)');
% ylabel('Altitude (m)');
% print -f1 -deps2c position

%%  simulation
if sim == 1
    % angle
    figure(1);
    subplot(3,1,1);
    plot(time, trueRotations(1,:)*180/pi, '--', time, measures(1,:)*180/pi, '-', time, trajectory_kalman(1,:)*180/pi, ':');
    title('anglex');
    legend('Real', 'Measured', 'Kalman');
    xlabel('time (sec)');
    ylabel('anglex (rad)');
    %print -f2 -deps2c anglex
    axis([0,100,-180,180])

    subplot(3,1,2);
    plot(time, trueRotations(2,:)*180/pi, '--', time, measures(2,:)*180/pi, '-', time, trajectory_kalman(2,:)*180/pi, ':');
    title('angley');
    legend('Real', 'Measured', 'Kalman');
    xlabel('time (sec)');
    ylabel('angley (rad)');
    %print -f3 -deps2c angley
    axis([0,100,-180,180])

    subplot(3,1,3);
    plot(time, trueRotations(3,:)*180/pi, '--', time, measures(3,:)*180/pi, '-', time, trajectory_kalman(3,:)*180/pi, ':');
    title('anglez');
    legend('Real', 'Measured', 'Kalman');
    xlabel('time (sec)');
    ylabel('anglez (rad)');
    %print -f3 -deps2c anglez
    axis([0,100,-180,180])

    % angular rate
    figure(2);
    subplot(3,1,1);
    plot(time, trueRotations(4,:), '--', time, measures(4,:), '-', time, trajectory_kalman(4,:), ':');
    title('wx');
    legend('Real', 'Measured', 'Kalman');
    xlabel('time (sec)');
    ylabel('wx (rad/s)');
    %print -f2 -deps2c anglex

    subplot(3,1,2);
    plot(time, trueRotations(5,:), '--', time, measures(5,:), '-', time, trajectory_kalman(5,:), ':');
    title('wy');
    legend('Real', 'Measured', 'Kalman');
    xlabel('time (sec)');
    ylabel('wy (rad/s)');
    %print -f3 -deps2c angley

    subplot(3,1,3);
    plot(time, trueRotations(6,:), '--', time, measures(6,:), '-', time, trajectory_kalman(6,:), ':');
    title('wz');
    legend('Real', 'Measured', 'Kalman');
    xlabel('time (sec)');
    ylabel('wz (rad/s)');
    %print -f3 -deps2c anglez

    % cogPosition zmp cogAcce
    figure(3);
    subplot(3,1,1);
    plot(time, trueTranslations(1,:), '--', time, tMeasures(1,:), '-', time, ttrajectory_kalman(1,:), ':');
    title('cog position');
    legend('Real', 'Measured', 'Kalman');
    xlabel('time (sec)');
    ylabel('position (m)');
    %print -f2 -deps2c anglex

    subplot(3,1,2);
    plot(time, trueTranslations(2,:), '--', time, tMeasures(2,:), '-', time, ttrajectory_kalman(2,:), ':');
    title('zmp');
    legend('Real', 'Measured', 'Kalman');
    xlabel('time (sec)');
    ylabel('zmp (m)');
    %print -f3 -deps2c angley

    subplot(3,1,3);
    plot(time, trueTranslations(3,:), '--', time, tMeasures(3,:), '-', time, ttrajectory_kalman(3,:), ':');
    title('cog Accelation');
    legend('Real', 'Measured', 'Kalman');
    xlabel('time (sec)');
    ylabel('accelation (m/s^2)');
    %print -f3 -deps2c anglez

%% experience
else
    % angle
    figure(1);
    subplot(2,1,1);
    plot( time, measures(1,:)*180/pi, '-', time, trajectory_kalman(1,:)*180/pi, ':');
    title('anglex');
    legend('Measured', 'Kalman');
    xlabel('time (sec)');
    ylabel('anglex (rad)');
    %print -f2 -deps2c anglex
    axis([0,60,-180,180])

    subplot(2,1,2);
    plot( time, measures(2,:)*180/pi, '-', time, trajectory_kalman(2,:)*180/pi, ':');
    title('angley');
    legend('Measured', 'Kalman');
    xlabel('time (sec)');
    ylabel('angley (rad)');
    %print -f3 -deps2c angley
    axis([0,60,-180,180])

%     subplot(3,1,3);
%     plot(time, trueRotations(3,:)*180/pi, '--', time, measures(3,:)*180/pi, '-', time, trajectory_kalman(3,:)*180/pi, ':');
%     title('anglez');
%     legend('Real', 'Measured', 'Kalman');
%     xlabel('time (sec)');
%     ylabel('anglez (rad)');
%     %print -f3 -deps2c anglez
%     axis([0,100,-180,180])

%     % angular rate
%     figure(2);
%     subplot(3,1,1);
%     plot(time, trueRotations(4,:), '--', time, measures(4,:), '-', time, trajectory_kalman(4,:), ':');
%     title('wx');
%     legend('Real', 'Measured', 'Kalman');
%     xlabel('time (sec)');
%     ylabel('wx (rad/s)');
%     %print -f2 -deps2c anglex
% 
%     subplot(3,1,2);
%     plot(time, trueRotations(5,:), '--', time, measures(5,:), '-', time, trajectory_kalman(5,:), ':');
%     title('wy');
%     legend('Real', 'Measured', 'Kalman');
%     xlabel('time (sec)');
%     ylabel('wy (rad/s)');
%     %print -f3 -deps2c angley
% 
%     subplot(3,1,3);
%     plot(time, trueRotations(6,:), '--', time, measures(6,:), '-', time, trajectory_kalman(6,:), ':');
%     title('wz');
%     legend('Real', 'Measured', 'Kalman');
%     xlabel('time (sec)');
%     ylabel('wz (rad/s)');
%     %print -f3 -deps2c anglez

    % cogPosition cogAcce
    figure(2);
    subplot(3,1,1);
    plot(time, tMeasures(1,:), '-', time, ttrajectory_kalman(1,:), ':');
    title('cog position');
    legend('Measured', 'Kalman');
    xlabel('time (sec)');
    ylabel('position (mm)');
    %print -f2 -deps2c anglex

    subplot(3,1,2);
    plot( time, ttrajectory_kalman(2,:), ':');
    title('velocity');
    legend('Kalman');
    xlabel('time (sec)');
    ylabel('velocity (mm/s)');
    %print -f3 -deps2c angley

    subplot(3,1,3);
    plot(time, tMeasures(2,:), '-', time, ttrajectory_kalman(3,:), ':');
    title('cog Accelation');
    legend( 'Measured', 'Kalman');
    xlabel('time (sec)');
    ylabel('accelation (mm/s^2)');
    %print -f3 -deps2c anglez
end
