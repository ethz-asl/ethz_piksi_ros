function gps_data = loadGPS(path, type, start_end)
    GPS_CEST = 2*3600 - 19;
    switch type
        case 'iphone'
            fid = fopen(path, 'rt', 'n', 'UTF16LE');
            fread(fid, 2, '*uint8');   %adjust the 2 to fit the UTF encoding
            filecontent = fread(fid, [1 inf], '*char');
            fclose(fid);
            datacell = textscan(filecontent, '%d%s%f%f%f%f%f%f%f', 'Delimiter', '\t', 'HeaderLines', 2);
            ind = 1:min(length(datacell{1}),length(datacell{end}));
            t = datetime(datacell{2}(ind), 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSS');
            lat = datacell{5}(ind);
            long = datacell{4}(ind);
            alt = datacell{6}(ind);
            head = datacell{7}(ind);
            speed = datacell{8}(ind)/3.6;
            speed(speed > 50) = 0;
            sat = zeros(size(speed));
        case 'raw'
            fid = fopen(path, 'r');
            filecontent = fread(fid, [1 inf], '*char');
            fclose(fid);
            datacell = textscan(filecontent, '%s%s%f%f%f%f%f%f%f%f', 'Delimiter', ',', 'HeaderLines', 1);
            ind = 1:min(length(datacell{1}),length(datacell{end}));
            t = datetime(datacell{1}(ind), 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSS');
            lat = datacell{4}(ind);
            long = datacell{5}(ind);
            alt = datacell{6}(ind);
            head = zeros(size(alt));
            [x, y] = deg2utm(lat, long);
            speed = [0; sqrt(diff(x).^2 + diff(y).^2)./(seconds(diff(t)))];
            speed(speed > 50) = 0;
            sat = datacell{9}(ind);
        case 'rtk'
            fid = fopen(path, 'r');
            filecontent = fread(fid, [1 inf], '*char');
            fclose(fid);
            headerlines = size(strfind(filecontent, '%'),2);
            datacell = textscan(filecontent, '%s%s%f%f%f%f%f%f%f%f%f%f%f%f%f', 'Delimiter', ' ', 'MultipleDelimsAsOne', 1, 'HeaderLines', headerlines);
            ind = 1:min(length(datacell{1}),length(datacell{end}));
            t = datetime(strcat(datacell{1}(ind), {' '}, datacell{2}(ind)), 'InputFormat', 'yyyy/MM/dd HH:mm:ss.SSS') + GPS_CEST/(3600*24);
            lat = datacell{3}(ind);
            long = datacell{4}(ind);
            alt = datacell{5}(ind);
            head = zeros(size(alt));
            [x, y] = deg2utm(lat, long);
            speed = [0; sqrt(diff(x).^2 + diff(y).^2)./(seconds(diff(t)))];
            speed(speed > 50) = 0;
            sat = datacell{7}(ind);
    end
    gps_data = {t, lat, long, alt, head, speed, sat};
    for i=1:length(gps_data)
        gps_data{i} = gps_data{i}(gps_data{1} >= start_end(1) & gps_data{1} <= start_end(2));
    end
    ind = gps_data{2} > 0.0;
    for i=1:length(gps_data)
        gps_data{i} = gps_data{i}(ind);
    end
end