# Data Processing

This page describes process of extracing the collected data from rosbag files using the provided scripts in the "data_processing" dir.

## Files
data_processing/
- condaenv.yml - The anaconda env file for using data processing scripts
- data_extraction
- -  extract_data.py - extracts multi sensor data from provided bagfiles
- -   updatedb.py - udpates the database with the extracted bagfile information
- model
- -  RawBagfile.py - Devfines the database structure for a bagfile
- -   setupdb.py - Utility script to connect to the database
- utils
- -  distance.py - utility script to calculate distance between gps coordinates
- -  Logger.py - Logging utility script
- -  md5hash.py - Utility to generate md5 hashes


## Sample usage

### extract script
1. setup ros env
```
$ source /opt/ros/kinetic/setup.bash
```

2. activate conda env for data extraction. (use the provided condaenv.yml file to create your conda environment for the first time)
```
$ conda activate wwe
```

The above two steps is mandatory for using any feature of the extract script

3. use the help param to see the script usage
```
$ python ./data_processing/data_extraction/extract_data.py --help
usage: extract_data.py [-h] [--encode ENCODE] [--hist] [-o OUTPUT] [-v]
                       rosbag datatype [datatype ...]

Extracts raw data from rosbag files

positional arguments:
  rosbag                Rosbag file to extract data
  datatype              Type of data to be extracted. supported option include
                        [all|info|images|caminfo|gps|lidar|imu]

optional arguments:
  -h, --help            show this help message and exit
  --encode ENCODE       [raw|jpeg] when provided with datatype=images, this
                        option extracts images in the corresponding format
  --hist                when provided with datatype=images, this option
                        generates image histograms
  -o OUTPUT, --output OUTPUT
                        Dir to dump extracted data
  -v, --verbose         enable verbose outputs
```

The script and extract images, caminfo, gps and lidar data individually by passing the corresponding arguments. You can pass the 'all' argument to extract all of the supported datatypes in one shot.

4. extracting images
```
$ python ./data_processing/data_extraction/extract_data.py ./2018-10-12_13-10-52.bag images --encode jpeg -v -o ./output/
```

This will create a dir named './output/extracted/' and dump all the extracted images in that dir.

Similarly you can use it for other datatypes.

5. extracting all info from rosbags:
```
$ python ./data_processing/data_extraction/extract_data.py ./2018-10-12_13-10-52.bag all -v -o /datastore01/wwe/raw/pilot/bangalore/2018-10-12/2018-10-12_13-10-52/
```

Along with extracting all the supported datatypes, the "all" option also does the following
- extracts the rosbag metadata into a output_path/info.json
- creates a output_path/analytics/timestamp.csv of all the sensor timestamps
- creates an mp4 video from the front left camera frames.

Extracting data with "all" option is required for the data web visualizer to work properly.
