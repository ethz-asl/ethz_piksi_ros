% Plot GPS solutions 
clear 
% close all 
clc

%% Initialize.
dataset = '20181011-111007';
path = '~/datasets/GPS/gps_swift/';

iphone_name = ['iphone-' dataset '.csv'];
raw_name = ['position_log_' dataset '.csv'];
rover_raw = ['swift-gnss-' dataset '.sbp.json'];
rover_obs = [rover_raw(1:end-4) 'obs'];
rover_nav = [rover_raw(1:end-4) 'nav'];
rover_sbs = [rover_raw(1:end-4) 'sbs'];
base_obs = ['ETH2-' dataset '.18o'];

solutions = {'single','kinematic'};
%% Calculate and load data.
iphone = loadGPS([path iphone_name], 'iphone', [datetime(1900,0,0); datetime(2100,0,0)]);
raw = loadGPS([path raw_name], 'raw', [iphone{1}(1); iphone{1}(end)]);

% Convert (if necessairy) sbp file
if ~exist([path rover_obs], 'file')
    [status, cmdline] = system(['~/catkin_ws/src/siemens_tools/dependencies/3rd_party/rtklib/app/convbin/gcc/sbp2rinex ' ... 
      path rover_obs(1:end-3) 'json']);
end
          
% Calculate (if necessairy) and load RTK data
for i=1:length(solutions)
    filepath = [path solutions{i} '-' dataset '.txt'];
    if ~exist(filepath, 'file')
          disp(['Processing ' solutions{i}]);
          [status, cmdline] = system(['~/catkin_ws/src/siemens_tools/dependencies/3rd_party/rtklib/app/rnx2rtkp/gcc/rnx2rtkp ' ... 
              path rover_obs ' ' path rover_nav ' ' path rover_sbs ' ' path base_obs  ' -k ' solutions{i} '.conf -o ' filepath]);
    end
    eval([solutions{i} ' = loadGPS(filepath, ''rtk'', [iphone{1}(1); iphone{1}(end)]);']);
end

%% Plot.
wm = webmap('Open Street Map');
wmline(iphone{2}, iphone{3}, 'Color', 'b');
wmmarker(iphone{2}, iphone{3}, 'Color', 'b', 'AutoFit', true, 'IconScale', 0.5, 'Alpha', 1);
wmline(raw{2}, raw{3}, 'Color', 'r');
wmmarker(raw{2}, raw{3}, 'Color', 'r', 'AutoFit', true, 'IconScale', 0.5, 'Alpha', 1);
%wmline(single{2}, single{3}, 'Color', 'g');
%wmmarker(single{2}, single{3}, 'Color', 'g', 'AutoFit', true, 'IconScale', 0.1, 'Alpha', 0.2);
wmline(kinematic{2}, kinematic{3}, 'Color', 'c');
wmmarker(kinematic{2}, kinematic{3}, 'Color', 'c', 'AutoFit', true, 'IconScale', 0.5, 'Alpha', 1);

figure 
title('# tracked satellites');
scatter(seconds(iphone{1}-iphone{1}(1)), iphone{7}, 'b.', 'DisplayName', 'iPhone');
hold on 
scatter(seconds(raw{1}-raw{1}(1)), raw{7} + 0.1, 'r.', 'DisplayName', 'Piksi Raw');
scatter(seconds(single{1}-single{1}(1)), single{7} + 0.2, 'g.', 'DisplayName', 'Single');
scatter(seconds(kinematic{1}-kinematic{1}(1)), kinematic{7} + 0.3, 'c.', 'DisplayName', 'RTK Kinematic');
legend('show');