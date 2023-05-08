#!/bin/bash

# Dependencies:
#   - RINEX (ideally newest version for best results)
#   - SBP2RINEX (from swift repos)
#   - pos2bag python script
#   [[- GDAL with newest version PROJ (python)
#   - sqlite3
#      --> https://github.com/OSGeo/GDAL.git ; https://github.com/OSGeo/PROJ/tree/master ; https://gis.stackexchange.com/questions/317109/build-gdal-with-proj-version-6 ;
#     ./configure --with-python --with-proj
#     https://proj.org/install.html ]]]
#   - geotf_python , numpy_eigen, catkin_boost_python_buildtool

# Script needs as argument directory with rosbag, base station correction and rover correction as sbp binaries and corresponding configuration file
# Base station correction file needs to have the prefix: "base_"

# Script:
#  1. converts sbp binaries to rinex and then generates a PPK solution using the base and rover observation
#  2. generates KML file which can be uploaded to swisstopo maps [https://map.geo.admin.ch/] to check for results (markers are color coded based on solution type)
#     (Green - Fix ; Yellow - Float ; Blue - No DGPS)
#  4. Adds ppk solution to new rosbag (has appendix _with_obs). The ppk solution can be converted to a reference frame if one is specified in script

# Configuration info:
# - needs to be in xyz --> pos2bag converts solution to enu later on.

# Make script fail if one of the steps fail
set -e

# TODO: Check if all files exist in dir and naming is correct!!!!!

# Font colors
BLUE='\033[0;36m'
MAGENTA='\033[0;35m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# File and directory variables. Observation directory needs to be passed as first argument
OBSERVATION_DIR=${1%/}
SOLUTION_DIR=$OBSERVATION_DIR/ppk_solution
RNX_DIR=$SOLUTION_DIR/rnx_files
DATA_NAME=""
SOL_FILE_NAME=""
COMP_SOLUTION=true

# Create solution directories if they don't exist
if [[ ! -d $SOLUTION_DIR ]]; then
  mkdir $SOLUTION_DIR
fi
if [[ ! -d $RNX_DIR ]]; then
  mkdir $RNX_DIR
fi

# Extract name of dataset from directory name
DATA_NAME="$(basename $OBSERVATION_DIR)"
SOL_FILE_NAME=ppk_$DATA_NAME

# Check if solution was already computed
for csv_file in $SOLUTION_DIR/*.csv; do
  if [[ -f "${csv_file}" ]]; then
    echo -e "${MAGENTA}\nFound csv file in solution directory. \nRecompute solution? (previous files will be lost) [Y/n] ${NC}"
    read user_input
    if [[ $user_input == "n" || $user_input == "N" ]]; then
      COMP_SOLUTION=false
    else
      # whipe directory
      rm -r $SOLUTION_DIR
      mkdir $SOLUTION_DIR
      mkdir $RNX_DIR
    fi
  fi
done

# Convert from SBP to RINEX
if $COMP_SOLUTION; then
  echo -e "${BLUE}\n=== Converting SBP Binaries to RINEX === \n ${NC}"
  for binaries in "$OBSERVATION_DIR"/*$DATA_NAME.sbp; do
    sbp2rinex $binaries -d $RNX_DIR
  done

  echo -e "${MAGENTA}\nWould you like to use SwissPos files as reference? (If not an SBP file prefixed with 'base_' needs to be in directory) [Y/n] ${NC}"
  read user_input
  if [[ $user_input == "Y" || $user_input == "y" ]]; then
    LOOK_FOR_SWISSPOS_FILES=true
  else
    LOOK_FOR_SWISSPOS_FILES=false
  fi
  
  # Copy SwissPos to correct directory
  if $LOOK_FOR_SWISSPOS_FILES; then
      echo -e "${MAGENTA}\nFound SWISSPOS observation files. Using these to compute PPK Solution.\n ${NC}"
      cp $OBSERVATION_DIR/*.2*o $RNX_DIR/base_$DATA_NAME.obs
      cp $OBSERVATION_DIR/*.2*n $RNX_DIR/base_$DATA_NAME.nav
  fi

  echo -e "${BLUE}\n=== Creating PPK Solution === \n ${NC}"
  rnx2rtkp $RNX_DIR/$DATA_NAME.obs $RNX_DIR/$DATA_NAME.nav $RNX_DIR/base_$DATA_NAME.obs $RNX_DIR/base_$DATA_NAME.nav -k $OBSERVATION_DIR/*.conf -o $SOLUTION_DIR/$SOL_FILE_NAME.csv

  # CREATE KML AND CHANGE TO CORRECT MARKERS!
  echo -e "${BLUE}\n=== Creating KML File from PPK Solution === \n ${NC}"
  pos2kml $SOLUTION_DIR/$SOL_FILE_NAME.csv
  # Change solution to correct markers as changing the color does not work
  # Marker size
  perl -i -pe's{<scale>.*?</scale>}{++$n == 1 ? "<scale>0.5</scale>" : $&}ge' $SOLUTION_DIR/$SOL_FILE_NAME.kml
  perl -i -pe's{<scale>.*?</scale>}{++$n == 2 ? "<scale>0.4</scale>" : $&}ge' $SOLUTION_DIR/$SOL_FILE_NAME.kml
  perl -i -pe's{<scale>.*?</scale>}{++$n == 3 ? "<scale>0.4</scale>" : $&}ge' $SOLUTION_DIR/$SOL_FILE_NAME.kml
  # Make no DGPS (blue) bigger so that it stands out in map
  perl -i -pe's{<scale>.*?</scale>}{++$n == 4 ? "<scale>0.7</scale>" : $&}ge' $SOLUTION_DIR/$SOL_FILE_NAME.kml
  # Set correct marker type
  perl -i -pe's{pal2(.*?)png}{++$n == 2 ? "paddle/grn-blank-lv.png" : $&}ge' $SOLUTION_DIR/$SOL_FILE_NAME.kml # Fixed - Green
  perl -i -pe's{pal2(.*?)png}{++$n == 2 ? "paddle/ylw-blank-lv.png" : $&}ge' $SOLUTION_DIR/$SOL_FILE_NAME.kml # Float - Yellow
  perl -i -pe's{pal2(.*?)png}{++$n == 2 ? "paddle/blu-blank-lv.png" : $&}ge' $SOLUTION_DIR/$SOL_FILE_NAME.kml # No DGPS (single) - Blue
fi

echo -e "${MAGENTA}\nWould you like to add PPK Solution to ROSbag? [Y/n] ${NC}"
read user_input
if [[ $user_input == "Y" || $user_input == "y" ]]; then
  echo -e "${BLUE}\n=== Starting Python script to add ppk solution to ROS Bag === \n ${NC}"
  ROSBAG=$OBSERVATION_DIR/$DATA_NAME.bag
  python pos2bag.py $ROSBAG $SOLUTION_DIR/$SOL_FILE_NAME.csv $SOLUTION_DIR
fi

echo -e "${MAGENTA}\nWould you like to compute the mean position of the base station? [Y/n] ${NC}"
read user_input
if [[ $user_input == "Y" || $user_input == "y" ]]; then
  echo -e "${BLUE}\n=== Computing mean base position from values when in FIX mode === \n ${NC}"
  python compute_base_mean_position.py $SOLUTION_DIR/$SOL_FILE_NAME.csv
fi