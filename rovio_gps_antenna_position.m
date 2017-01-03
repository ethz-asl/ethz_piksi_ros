% This script computes GPS antenna position with respect to ViSensor IMU
% given GPS antenna position with respect to MAV IMU and MSF parameters.
% Usually GPS antenna is right above MAV IMU, therefore it is easier to measure
% the distance of the antenna to this IMU.

%% User's configurable parameters
% MAV calibration from msf_parameters_vision
q_ic_x = 0.80117676497;
q_ic_y = -0.00153579344205;
q_ic_z = 0.598419967364;
q_ic_w = 0.00264107385719;

p_ic_x = 0.121717437675;
p_ic_y = -0.0163993683837;
p_ic_z = -0.0534519930152;

% distance of the gps antenna to MAV IMU, in IMU frame
MavImu_r_MavImu_Gps_x = 0.0; % [m]
MavImu_r_MavImu_Gps_y = 0.0; % [m]
MavImu_r_MavImu_Gps_z = 0.11; % [m]


%% Transformation
% from sensor IMU (C frame in MSF) to MAV IMU (I frame in MSF)
q_I_C = [q_ic_w, q_ic_x, q_ic_y, q_ic_z];
I_p_I_C = [p_ic_x; p_ic_y; p_ic_z];

R_I_C = quat2rotm(q_I_C);

T_I_C = [R_I_C, I_p_I_C;
         0.0, 0.0, 0.0, 1.0];

% homogeneous coordinates
M_r_M_V = T_I_C * [MavImu_r_MavImu_Gps_x; 
                   MavImu_r_MavImu_Gps_y;
                   MavImu_r_MavImu_Gps_z;
                   1.0];
 
%% Print result
 format longg;
 display(['Rovio MrMV: ' num2str(M_r_M_V(1), '%.5f') ', ' num2str(M_r_M_V(2), '%.5f') ', ' num2str(M_r_M_V(3), '%.5f')]);

