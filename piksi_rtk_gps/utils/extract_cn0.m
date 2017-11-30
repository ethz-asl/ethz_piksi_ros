function [cn0, relative_time] = extract_cn0(data)

% look for max number of satellites
[max_sat, max_sat_index] = max(data.num_sat);
sat_ids = str2num(data.sat{max_sat_index});

% collect carrier to noise ratio for every satellite
number_samples = length(data.cn0);
cn0 = zeros(number_samples, max_sat);
% cn0(:,1) will contain cn0 of satellite with id = sat_ids(1) 

for index_sample = 1 : number_samples
    % check which satellite ids we have in this sample
    sat_id_in_sample = str2num(data.sat{index_sample});
    % collect cn0 for every present satellite and put it in the right
    % column of cn0
    cn0_sample = str2num(data.cn0{index_sample});
    for index_id = 1 : max_sat
        id = sat_ids(index_id);
        % look if curret id is present in this sample
        position_id = find(sat_id_in_sample == id, 1); 
        if ~ isempty(position_id)
            cn0(index_sample, index_id) = cn0_sample(position_id);
        end
    end

end

relative_time = (data.rosbagTimestamp - data.rosbagTimestamp(1)) / 1e9;

end

