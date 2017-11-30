%% load cvs data extracted from rosbag
data = load('gps_helix_antenna');

%% plot carrier to nois ratio
[cn0, relative_time] = extract_cn0(data);
plot(relative_time, cn0);
ylabel('C/N0');
xlabel('Time [s]');
grid on;
