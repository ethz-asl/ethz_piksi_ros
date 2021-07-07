#!/usr/bin/python
import sys
import csv

import numpy as np

if __name__ == "__main__":
    file_dir = sys.argv[1]
    lat = []
    lon = []
    height = []

    with open(file_dir, "r") as f:
        reader = csv.reader(f, delimiter=" ", skipinitialspace=True)
        # Run through header
        in_header = True
        observation_time = None
        while in_header:
            header_line = next(reader)
            # Store which time is being used
            if len(header_line) > 1 and header_line[1] == "obs":
                observation_time = header_line[6]
            if header_line[0] != "%":
                in_header = False

        for row in reader:
            # Only append postion if in Fix mode!
            if row[5] == "1":
                lat.append(float(row[2]))
                lon.append(float(row[3]))
                height.append(float(row[4]))

    if len(lat) == 0:
        print(
            "Could not compute mean position. No solutions with fix mode found in file!"
        )
        sys.exit(1)

    # Compute mean:
    mean_lat = sum(lat) / len(lat)
    mean_lon = sum(lon) / len(lon)
    mean_height = sum(height) / len(height)

    print(
        f"Average base position:\nLatitude:\t{mean_lat}\nLongitude:\t{mean_lon}\nHeight:\t\t{mean_height}"
    )
