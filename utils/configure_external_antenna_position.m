%% 
% This script helps you configuring rovio_filter.info file when using 
% external pose measurements, such as a GPS receiver.
% The output of this script is a quaterion (qVM) and 
% a position vector (MrMV), which must be copied into
% 'rovio_filter.info' file, in the section "PoseUpdate".
%
% Script outputs:
% qVM represents the orientation of the external pose measurement frame
% with respect to ViSensor IMU frame.
% MrMV represents the position of the origin of the external pose
% measurement frame with respect to the origin of the ViSensor IMU frame.
%
% Frames:
% Frame names in rovio: M (ViSensor IMU), V (external pose, for example 
%                       GPS), I (inertial)
% Frame names in msf: I (MAV IMU), C (ViSensor IMU)
% Custon frame names used in this script: MI (MAV IMU)
%

% ----------------------------------------------------------
%% user's parameters: fill this section with our parameters!
% ----------------------------------------------------------

% msf_parameters_vision from msf_parameters_vision.yaml file
% pose_sensor/init/q_ic
q_I_C_x = -0.58546635941;
q_I_C_y = 0.588750056004;
q_I_C_z = -0.389971319019;
q_I_C_w = 0.398151835231;

% pose_sensor/init/p_ic
I_p_I_C_x = 0.120477158968;
I_p_I_C_y = -0.00541789178186;
I_p_I_C_z = -0.109118925469;

% gps antenna position respect MAV IMU, for now taken manually with a meter
MI_p_MI_V_x = 0.0;
MI_p_MI_V_y = 0.0;
MI_p_MI_V_z = 0.11;

% ----------------------------------------------------------

%% compute transformation T_MI_M
T_MI_V = trvec2tform([MI_p_MI_V_x, MI_p_MI_V_y, MI_p_MI_V_z]);
% simple inverse of T_MI_V since there's no rotation
T_V_MI = trvec2tform(-[MI_p_MI_V_x, MI_p_MI_V_y, MI_p_MI_V_z]);
% q = [w x y z]
T_MI_M = trvec2tform([I_p_I_C_x, I_p_I_C_y, I_p_I_C_z]) * ...
         quat2tform([q_I_C_w, q_I_C_x, q_I_C_y, q_I_C_z]);

T_V_M = T_V_MI * T_MI_M;
q_V_M = tform2quat(T_V_M);
R_V_M = tform2rotm(T_V_M);
V_r_V_M = tform2trvec(T_V_M);

T_M_V = [R_V_M', -(R_V_M') * V_r_V_M';
         0.0, 0.0, 0.0, 1.0];
M_r_M_V = tform2trvec(T_M_V);
q_M_V = tform2quat(T_M_V);

%% print rovio settings for external pose section
% q = [w x y z]
fprintf(['\n ----- copy following values into section "PoseUpdate"' ...
        ' of rovio_filter.info ----- \n\n']);
    
fprintf(' qVM_x %.7f \n qVM_y %.7f \n qVM_z %.7f \n qVM_w %.7f \n', ...
        q_V_M(2), q_V_M(3), q_V_M(4), q_V_M(1));
fprintf(' MrMV_x %.7f \n MrMV_y %.7f \n MrMV_z %.7f \n', ...
    M_r_M_V(1), M_r_M_V(2), M_r_M_V(3));

fprintf(['\n-------------------------------------------------------' ... 
         '------------------------------\n']);

%% print TF deug broadcaster nodes
% creat TF broadcaster to visualize transformation in rviz
fprintf(['\n ----- use the following TF broadcaster to double ' ...
         ' check outputed transformation ----- \n\n']);
% map == mav_imu for testing
fprintf(['<node pkg="tf" type="static_transform_publisher"' ... 
         ' name="map_mav_imu_broadcaster"' ...
         ' args="0 0 0 0 0 0 1' ...
         ' map mav_imu 100" /> \n']);

% mav_imu to ViSensor imu
tf_node_string=['<node pkg="tf" type="static_transform_publisher"' ...
                ' name="mav_imu_vi_sensor_broadcaster"' ...
                ' args="%d %d %d %d %d %d %d' ...
                ' mav_imu vi_sensor_imu 100" /> \n'];
fprintf(tf_node_string, I_p_I_C_x, I_p_I_C_y, I_p_I_C_z, ...
        q_I_C_x, q_I_C_y, q_I_C_z, q_I_C_w);

% ViSensor imu to GPS antenna position (T_M_V)
tf_node_string=['<node pkg="tf" type="static_transform_publisher"' ...
                ' name="vi_sensor_gps_broadcaster"' ...
                ' args="%d %d %d %d %d %d %d' ...
                ' vi_sensor_imu gps_antenna 100" /> \n'];

px = M_r_M_V(1);
py = M_r_M_V(2);
pz = M_r_M_V(3);
qx = q_M_V(2);
qy = q_M_V(3);
qz = q_M_V(4);
qw = q_M_V(1);
fprintf(tf_node_string, px, py, pz, qx, qy, qz, qw);
fprintf(['\n-------------------------------------------------------' ... 
         '------------------------------\n']);