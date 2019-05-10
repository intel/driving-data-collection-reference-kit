#!/usr/bin/env python

'''
  Python 2.7 script to extract data from rosbag files.
  This scripts supports extraction of images (jpeg or ppm), gps, imu and the rosbag info.
  Refer to help option for the usage details.

  Copyright (C)  2019 Intel Corporation
  SPDX-License-Identifier: BSD-3-Clause 

'''


import os # for file & path utils
import stat # for file permission handling

# image processing
import cv2
from matplotlib import pyplot as plt

# ros imports
import roslib
import rosbag
import rospy
import sensor_msgs.point_cloud2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # ros to CV conversions
import yaml

import pandas as pd # dataframes and csv
import json # json data handling
import xml.etree.ElementTree as ET # xml data handling

# from multiprocessing import Process # handle multiprocessing
import subprocess

from datetime import datetime # time management
from tqdm import tqdm # for progress bar
import humanfriendly # convert time and numbers to human friendly format

# if called as standlone script, add top dir to syspat to import our modules
if __name__ == "__main__":
    import sys
    repoPath = os.path.dirname(os.path.realpath(__file__)) + "/../"
    sys.path.append(repoPath)
# else, we presume the modules are organized and available
from utils import Logger
from utils import distance

__author__ = "Nitheesh K L"
__copyright__ = "Copyright (C) 2019, Intel Corporation"
__credits__ = ["Nitheesh K L"]
__license__ = "BSD-3-Clause"
__version__ = "1.0.0"
__maintainer__ = "Nitheesh K L"
__email__ = "nitheesh.k.l@intel.com"
__status__ = "Dev"

# time format for all data processing
TIME_FILE_NAME_FORMAT="%Y-%m-%d_%-H-%-M-%-S-%f"
# interested sensor topics for rosbags
topic_sensor_info = ['/gps/fix', '/imu/data', '/imu/mag', '/lidar/pointCloud']

'''
Util func to create dirs if not present
'''
def make_dirs(dir_path):
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)

'''
Generate list of interested topic for provided msg type
Params:
    msgType - ros msg type of interest
Returns:
    list of topics that contains msgType or emtpy list if none
'''
def getTopics(bag, msgType):
    interestedTopics = []
    for topic,topictouple in bag.get_type_and_topic_info()[1].iteritems():
        if topictouple.msg_type == msgType:
            interestedTopics.append(topic)

    return interestedTopics

'''
Extract images from provided rosbag
Params:
    bag - opened rosbag file handle
    extractPath - dir under which images will be extracted.
    img_type - file type for images. Raw (ppm) or Jpeg.
    hist - if true, generates histogram of images
'''
def extract_images(bag, extractPath, img_type="raw", hist=False):
    topic_raw_image = getTopics(bag, 'sensor_msgs/Image')
    Logger.debug(('intesrested image topics = %s' % topic_raw_image))
    jobs = []
    for topic in topic_raw_image:
        #p = Process(target=extract_topic_images, args=(bag, topic, extractPath, img_type, hist, ))
        #jobs.append(p)
        #p.start()
        extract_topic_images(bag, topic, extractPath, img_type, hist)

    #for p in jobs:
    #    p.join()

'''
Utility method to extract images from a specific topic
'''
def extract_topic_images(bag, topic, extractPath, img_type="raw", hist=False):

    if img_type == "raw":
        file_format = ".pgm"

    if img_type == "jpeg":
        file_format = ".jpeg"

    bridge = CvBridge()

    # crate dirs for topic
    make_dirs(extractPath + topic)

    frame = 0
    timestampFile = extractPath + topic + "/timestamp.csv"
    timestampsInSec, timestampsInString = [], []
    for tpic, msg, time in tqdm(bag.read_messages(topics=[topic]), unit="frame", desc="extracting images from " + topic):
        timestampSec = msg.header.stamp.to_time()
        dtime = datetime.fromtimestamp(timestampSec)
        timestampString = dtime.strftime(TIME_FILE_NAME_FORMAT)
        timestampsInSec.append(timestampSec)
        timestampsInString.append(timestampString)
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            image_file = extractPath + topic + "/" + str(frame).zfill(7) + file_format

            if not os.path.exists(image_file):
                #Logger.debug('extracted ' + image_file)
                cv2.imwrite(image_file, cv_image)
            else:
                Logger.warn("file exists. skipping write: " + image_file)

            if hist:
                hist_file = extractPath + topic + "/" + str(frame).zfill(7) + "_hist.png"
                gry_img = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
                h = plt.hist(gry_img.ravel(), 256, [0,256], color="black")
                plt.savefig(hist_file)
        frame += 1

    df = pd.DataFrame(columns=['sec', 'datetime'])
    df['sec'] = timestampsInSec
    df['datetime'] = timestampsInString
    df.index.name='frame'
    df.to_csv(timestampFile, float_format='%.7f')

'''
Extract rosbag info as json file
Params:
    bag - opened rosbag file handle
    base_path - dir under which the info.json file will be extracted
'''
def extract_bag_info(bag, base_path):
    info_file = base_path + '/info.json'
    baginfo = yaml.load(bag._get_yaml_info())

    with open(info_file, 'w+') as f:
        f.write('%s' % json.dumps(baginfo))
        f.close()

    Logger.debug(json.dumps(baginfo))

'''
Extracts camera info msgs
Params:
    bag - opened rosbag file handle
    base_path - dir under which the extracted files will be stored
'''
def extract_cam_info(bag, base_path):
    extract_sensor_info(bag, 'sensor_msgs/CameraInfo', base_path)

'''
Extracts imu info from rosbag and stores each frame as text file
Params:
    bag - opened rosbag file handle
    base_path - dir under which the data.json file will be extracted
'''
def extract_imu_info(bag, base_path):
    extract_sensor_info(bag, 'sensor_msgs/Imu', base_path)

'''
Extracts gps info into json and kml files
Params:
    bag - opened rosbag file handle
    extractPath - dir under which the data.json file will be extracted
'''
def extract_gps_info(bag, extractPath):
    topic_sensor_info = getTopics(bag, 'sensor_msgs/NavSatFix')
    Logger.debug(('interested gps topics = %s' % topic_sensor_info))

    if not topic_sensor_info:
        Logger.warn('no topics available to extract data!')
        return

    # create dir for each topic
    for topic in topic_sensor_info:
        make_dirs(extractPath + topic)

    sensordata = {}
    sensordata["data"] = []
    json_dump_file = extractPath + topic + "/data.json"
    kml_file = extractPath + topic + "/data.kml"
    timestampFile = extractPath + topic + "/timestamp.csv"
    timestampsInSec, timestampsInString = [], []
    # read gps data
    for topic in topic_sensor_info:
        for tpic, msg, time in tqdm(bag.read_messages(topics=topic_sensor_info), unit=" frame", desc="extracting " + topic):
            timestampSec = msg.header.stamp.to_time()
            dtime = datetime.fromtimestamp(timestampSec)
            timestampString = dtime.strftime(TIME_FILE_NAME_FORMAT)
            timestampsInSec.append(timestampSec)
            timestampsInString.append(timestampString)
            sensor_info_file = extractPath + topic + "/" + dtime.strftime(TIME_FILE_NAME_FORMAT) + ".txt"
            data = {}
            data['header'] = {'seq': msg.header.seq, 'stamp': {'secs': msg.header.stamp.secs, 'nsecs': msg.header.stamp.nsecs}}
            data['status'] = {'status': msg.status.status, 'service': msg.status.service}
            data['latitude'] = msg.latitude
            data['longitude'] = msg.longitude
            data['altitude'] = msg.altitude
            data['position_covariance'] = [x for x in msg.position_covariance]
            data['position_covariance_type'] = msg.position_covariance_type
            sensordata["data"].append(data)
            #if not os.path.exists(sensor_info_file):
            #    debugPrint('extracted ' + sensor_info_file kml_document+ "\n")
            #    f = open(sensor_info_file, 'w+')
            #    f.write('%s\n' % msg)
            #    f.close()
            #else:
            #    print "file exists - " + sensor_info_file
        df = pd.DataFrame(columns=['sec', 'datetime'])
        df['sec'] = timestampsInSec
        df['datetime'] = timestampsInString
        df.index.name='frame'
        df.to_csv(timestampFile, float_format='%.7f')

        baginfo = yaml.load(bag._get_yaml_info())
        # Create KML file
        kml_root = ET.Element('kml')
        kml_root.set('xmlns', 'http://www.opengis.net/kml/2.2')
        kml_document = ET.SubElement(kml_root,'Document')
        kml_document_name = ET.SubElement(kml_document,'name')
        kml_document_name.text = baginfo['path'].split('/')[-1]
        kml_document_desc = ET.SubElement(kml_document,'description')
        kml_document_desc.text = "size=" + str(baginfo['size']) + ", duration=" + str(baginfo['duration'])
        style_start = ET.SubElement(kml_document,'Style')
        style_start.set('id', 'startmark')
        style_start_iconstyle = ET.SubElement(style_start, 'IconStyle')
        style_start_iconstyle_icon = ET.SubElement(style_start_iconstyle, 'Icon')
        style_start_iconstyle_icon_href = ET.SubElement(style_start_iconstyle_icon, 'href')
        style_start_iconstyle_icon_href.text = "http://maps.google.com/mapfiles/kml/paddle/grn-circle-lv.png"
        style_stop = ET.SubElement(kml_document,'Style')
        style_stop.set('id', 'stopmark')
        style_stop_iconstyle = ET.SubElement(style_stop, 'IconStyle')
        style_stop_iconstyle_icon = ET.SubElement(style_stop_iconstyle, 'Icon')
        style_stop_iconstyle_icon_href = ET.SubElement(style_stop_iconstyle_icon, 'href')
        style_stop_iconstyle_icon_href.text = "http://maps.google.com/mapfiles/kml/paddle/red-circle-lv.png"
        style_line = ET.SubElement(kml_document,'Style')
        style_line.set('id', 'line')
        style_line_linestyle = ET.SubElement(style_line, 'LineStyle')
        style_line_linestyle_color = ET.SubElement(style_line_linestyle, 'color')
        style_line_linestyle_color.text = "FF0000"
        style_line_linestyle_width = ET.SubElement(style_line_linestyle, 'width')
        style_line_linestyle_width.text = "4"

        coordinates = "\n"
        for data in sensordata["data"]:
            coordinates = coordinates + str(data['longitude']) + "," + str(data['latitude']) + "," + str(data['altitude']) + "\n"

        kml_document_placemark_start = ET.SubElement(kml_document,'Placemark')
        placemark_name = ET.SubElement(kml_document_placemark_start,'name')
        placemark_name.text = "Start - " + baginfo['path'].split('/')[-1]
        placemark_desc = ET.SubElement(kml_document_placemark_start,'description')
        placemark_desc.text = "size=" + humanfriendly.format_size(baginfo['size']) + ", duration=" + humanfriendly.format_timespan(baginfo['duration'])
        placemark_styleurl = ET.SubElement(kml_document_placemark_start, 'styleUrl')
        placemark_styleurl.text = "#startmark"
        placemark_starting_point = ET.SubElement(kml_document_placemark_start, 'Point')
        placemark_starting_point_coordinates = ET.SubElement(placemark_starting_point, 'coordinates')
        placemark_starting_point_coordinates.text = str(sensordata['data'][0]['longitude']) + "," + str(sensordata['data'][0]['latitude']) + "," + str(sensordata['data'][0]['altitude'])

        kml_document_placemark_end = ET.SubElement(kml_document,'Placemark')
        placemark_name = ET.SubElement(kml_document_placemark_end,'name')
        placemark_name.text = "Stop - " + baginfo['path'].split('/')[-1]
        placemark_desc = ET.SubElement(kml_document_placemark_end,'description')
        placemark_desc.text = "size=" + humanfriendly.format_size(baginfo['size']) + ", duration=" + humanfriendly.format_timespan(baginfo['duration'])
        placemark_styleurl = ET.SubElement(kml_document_placemark_end, 'styleUrl')
        placemark_styleurl.text = "#stopmark"
        placemark_ending_point = ET.SubElement(kml_document_placemark_end, 'Point')
        placemark_ending_point_coordinates = ET.SubElement(placemark_ending_point, 'coordinates')
        placemark_ending_point_coordinates.text = str(sensordata['data'][-1]['longitude']) + "," + str(sensordata['data'][-1]['latitude']) + "," + str(sensordata['data'][-1]['altitude'])

        kml_document_placemark_line = ET.SubElement(kml_document,'Placemark')
        placemark_name = ET.SubElement(kml_document_placemark_line,'name')
        placemark_name.text = baginfo['path'].split('/')[-1]
        placemark_desc = ET.SubElement(kml_document_placemark_line,'description')
        placemark_desc.text = "size=" + humanfriendly.format_size(baginfo['size']) + ", duration=" + humanfriendly.format_timespan(baginfo['duration'])
        placemark_styleurl = ET.SubElement(kml_document_placemark_line, 'styleUrl')
        placemark_styleurl.text = "#line"
        placemark_linestring = ET.SubElement(kml_document_placemark_line, 'LineString')
        placemark_linestring_coordinates = ET.SubElement(placemark_linestring, 'coordinates')
        placemark_linestring_coordinates.text = coordinates

        with open(kml_file, "w+") as f:
            f.write(ET.tostring(kml_root, encoding='UTF-8', method='xml'))

        # write to json file
        with open(json_dump_file, "w+") as f:
            f.write(json.dumps(sensordata))
            Logger.debug("dumped binary data to " + json_dump_file)

        # Calculate distance covered from the extracted data.json file
        # and save the distance in the bagfiles info.json
        with open(json_dump_file) as f:
            gpsData = json.load(f)
        if gpsData == None or gpsData["data"] == None:
            Logger.error("No gps data found in " + json_dump_file)
            return
        data = gpsData["data"]
        dist = distance.calcDistance(data)
        baginfo = None
        # info.json is stored one dir above
        infofile = extractPath + '/../info.json'
        if not os.path.exists(infofile):
            Logger.warn("info file not found - " + infofile)
            return
        with open(infofile) as f:
            baginfo = json.load(open(infofile))
        if baginfo is None:
            Logger.error("No data found in " + infofile)
            return
        Logger.debug("distance = " + str(dist) + " km")
        if "distance" in baginfo:
            dist = dist + baginfo["distance"]["dist"]
        baginfo["distance"] = {"unit":"km", "dist":dist}
        Logger.debug("updating info")
        with open(infofile, "w") as f:
            json.dump(baginfo, f)

'''
Extracts lidar points into json files
Params:
    bag - opened rosbag file handle
    extractPath - dir under which the extracted data will be stored
'''
def extract_lidar_points(bag, extractPath):
    topics = getTopics(bag, 'sensor_msgs/PointCloud2')
    Logger.debug(('interested lidar topics = %s \n' % topics))

    for topic in topics:
        extractDir = extractPath + '/' + topic
        make_dirs(extractDir)
        frame = 0
        timestampFile = extractPath + topic + "/timestamp.csv"
        timestampsInSec, timestampsInString = [], []
        for tpic, msg, time in tqdm(bag.read_messages(topics=[topic]), unit=" frame", desc="extracting " + topic):
            timestampSec = msg.header.stamp.to_time()
            dtime = datetime.fromtimestamp(timestampSec)
            timestampString = dtime.strftime(TIME_FILE_NAME_FORMAT)
            timestampsInSec.append(timestampSec)
            timestampsInString.append(timestampString)
            data = []
            for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
                data.append({
                    'x': point[0], # x coordinate
                    'y': point[1], # y coordinate
                    'z': point[2], # z coordinate
                    'i': point[3], # intensity
                    'r': point[4]  # ring
                })
            frameFile = extractDir + "/" + str(frame).zfill(7) + ".json"
            with open(frameFile, 'w') as f:
                f.write(json.dumps(data))
            frame += 1
        # write timestamps of this topic
        df = pd.DataFrame(columns=['sec', 'datetime'])
        df['sec'] = timestampsInSec
        df['datetime'] = timestampsInString
        df.index.name='frame'
        df.to_csv(timestampFile, float_format='%.7f')

'''
Extracts sensor msgs from rosbag as json data
Params:
    bag - opened rosbag file handle
    extractPath - dir under which the extracted data will be stored
'''
def extract_sensor_info(bag, topic, extractPath):
    topic_sensor_info = getTopics(bag, topic)
    Logger.debug(('interested topics = %s \n' % topic_sensor_info))

    if not topic_sensor_info:
        Logger.warn('no topics available to extract data!')
        return

    # create dir for each topic
    for topic in topic_sensor_info:
        make_dirs(extractPath + topic)

        msgs = []
        json_dump_file = extractPath + topic + "/data.json"
        timestampFile = extractPath + topic + "/timestamp.csv"
        timestampsInSec, timestampsInString = [], []
        for tpic, msg, time in tqdm(bag.read_messages(topics=[topic]), unit=" frame", desc="extracting " + topic):
            timestampSec = msg.header.stamp.to_time()
            dtime = datetime.fromtimestamp(timestampSec)
            timestampString = dtime.strftime(TIME_FILE_NAME_FORMAT)
            timestampsInSec.append(timestampSec)
            timestampsInString.append(timestampString)
            sensor_info_file = extractPath + topic + "/" + dtime.strftime(TIME_FILE_NAME_FORMAT) + ".txt"
            data = {}
            msgs.append(msg)
            if not os.path.exists(sensor_info_file):
                Logger.debug('extracted ' + sensor_info_file)
                #f = open(sensor_info_file, 'w+')
                #f.write('%s\n' % msg)
                #f.close()
            else:
                Logger.warn("file exists. skipping write: " + sensor_info_file)
        df = pd.DataFrame(columns=['sec', 'datetime'])
        df['sec'] = timestampsInSec
        df['datetime'] = timestampsInString
        df.index.name='frame'
        df.to_csv(timestampFile, float_format='%.7f')

        sensordata = {}
        sensordata["data"] = msgs
        with open(json_dump_file, "w+") as f:
            f.write('%s' % sensordata)
            Logger.debug("dumped binary data to " + json_dump_file)

'''
Combines all the images in a given dir into a video using ffmpeg
Params:
    imagesDir - directory containing images
    output    - name of the video file to be created
    fps       - fps of the video to be created. Default=24
'''
def makeVideo(imagesDir, output, fps=24):
    command = [ 'ffmpeg', # use system installed ffmepg binary
	    '-f', 'image2',
	    '-r', str(fps), # frames per second
	    '-i', imagesDir +  '/%07d.jpeg', # images are stored as <7digits>.jpeg
	    '-an', # Tells FFMPEG not to expect any audio
	    '-y', # (optional) overwrite output file if it exists
	    '-stats', # (optional) show stats
	    '-loglevel', 'error',  # (optional) disable long info prints
	    output ]
    subprocess.call(command)

def extract_videos(basepath):
    Logger.debug("Generating videos")
    extractDir = basepath + "/extracted"
    infoFile = basepath + "/info.json"
    if not os.path.exists(infoFile):
        Logger.error("info file not found! Exiting.")
        return

    with open(infoFile) as f:
        info = json.load(f)

    # generate videos
    videoDir = basepath + '/videos'
    for topic in info['topics']:
        topicName = topic['topic']
        msgType = topic['type']
        # create videos from only the left lens/camera
        if msgType == 'sensor_msgs/Image' and 'left' in topicName:
            imagesDir = extractDir + topicName
            if not os.path.exists(imagesDir):
                Logger.error("Doesn't exist - " + imagesDir)
                continue
            make_dirs(videoDir)
            filename = topicName.replace('image_raw', '').replace('left', 'Left').replace('/', '') + '.mp4'
            videoFile = videoDir + '/' + filename
            Logger.debug('creating ' + videoFile)
            makeVideo(imagesDir, output=videoFile, fps=10)

    # Annotations dir for storing video comments
    annotDir = basepath + '/annotations'
    make_dirs(annotDir)
    os.chmod(annotDir, stat.S_IRWXU | stat.S_IRWXG | stat.S_ISGID | stat.S_IRWXO)

'''
Extracts all required data from the bag files, generates corresponding
timestamp files and videos
Params:
    bag - opened bagfile to be processed
    basepath - directory to place the extracted data
'''
def extract_all(bag, basepath):
    extractDir = basepath + "/extracted"

    # extract bag info
    extract_bag_info(bag, basepath)
    # extract gps
    extract_gps_info(bag, extractDir)
    # extract lidar
    extract_lidar_points(bag, extractDir)
    # extract images
    extract_images(bag, extractDir, img_type="jpeg", hist=False)

    with open(basepath + "/info.json") as f:
        info = json.load(f)

    # collate timestamps csv
    Logger.debug('creating timestamps.csv')
    # we are interested only in images, lidar, gps and imu data types
    interestedTypes = ['sensor_msgs/Image', 'sensor_msgs/PointCloud2', 'sensor_msgs/NavSatFix', 'sensor_msgs/Imu']
    dataFrames = []
    topicNames = []
    for topic in info['topics']:
        topicName = topic['topic']
        msgType = topic['type']
        timestampFile = extractDir + topicName + "/timestamp.csv"
        if msgType in interestedTypes and os.path.exists(timestampFile):
            Logger.debug('reading timestamps from ' + timestampFile)
            df = pd.read_csv(timestampFile, index_col=['frame'],usecols=['frame', 'sec'])
            df.columns = [topicName]
            dataFrames.append(df)
            topicNames.append(topicName)
    allFrames = pd.concat(dataFrames, ignore_index=True, axis=1)
    allFrames.columns = topicNames
    make_dirs(basepath + '/analytics')
    allFrames.to_csv(basepath + '/analytics/timestamps.csv', float_format='%.7f')
    Logger.debug('timestamps written to ' + basepath + '/analytics/timestamps.csv')

    # generate videos
    videoDir = basepath + '/videos'
    for topic in info['topics']:
        topicName = topic['topic']
        msgType = topic['type']
        # create videos from only the left lens/camera
        if msgType == 'sensor_msgs/Image' and 'left' in topicName:
            imagesDir = extractDir + topicName
            make_dirs(videoDir)
            filename = topicName.replace('image_raw', '').replace('left', 'Left').replace('/', '') + '.mp4'
            videoFile = videoDir + '/' + filename
            Logger.debug('creating ' + videoFile)
            makeVideo(imagesDir, output=videoFile, fps=10)

    # Annotations dir for storing video comments
    annotDir = basepath + '/annotations'
    make_dirs(annotDir)
    os.chmod(annotDir, stat.S_IRWXU | stat.S_IRWXG | stat.S_ISGID | stat.S_IRWXO)

def main(argv):
    parser = argparse.ArgumentParser(description="Extracts raw data from rosbag files")
    parser.add_argument('rosbag', help='Rosbag file to extract data')
    parser.add_argument('datatype', nargs='+', help='Type of data to be extracted. supported option include [all|info|images|caminfo|gps]')
    parser.add_argument('--encode', help='[raw|jpeg] when provided with datatype=images, this option extracts images in the corresponding format')
    parser.add_argument('--hist', action='store_true', help='when provided with datatype=images, this option generates image histograms')
    parser.add_argument('-o', '--output', help='Dir to dump extracted data')
    parser.add_argument('-v', '--verbose', action='store_true', help='enable verbose outputs')
    args = parser.parse_args()

    # Initialized logger
    Logger.init(level=Logger.LEVEL_INFO, name="extract_data")
    if args.verbose:
        Logger.setLevel(Logger.LEVEL_DEBUG)

    inputfile = args.rosbag
    # set ouput dir if provided else extract data in current dir
    if args.output is not None:
        basepath = args.output
    else:
        basepath = "./"
    # all sensor data is extracted into <base_path>/extracted dir"
    outputdir = basepath + "/extracted"

    Logger.debug('processing ' + inputfile)
    Logger.debug('extracting to ' + outputdir)

    # create output dir it not existing for dumping data
    if not os.path.exists(outputdir):
        os.makedirs(outputdir)
    # check if bagfile exists
    if not os.path.exists(inputfile):
        Logger.error("File not found: " + inputfile)
        return

    bag  = None
    # extract specified datatypes
    for datatype in args.datatype:
        if datatype == 'videos':
            extract_videos(basepath)
        else:
            # open bagfile
            if bag is None:
                bag = rosbag.Bag(inputfile)
            if datatype == 'images':
                hist = False
                if args.hist:
                    hist = True
                if args.encode is not None and args.encode == "raw":
                    extract_images(bag, outputdir, img_type="raw", hist=hist)
                elif args.encode is not None and args.encode == "jpeg":
                    extract_images(bag, outputdir, img_type="jpeg", hist=hist)
                else:
                    extract_images(bag, outputdir)
            elif datatype == 'caminfo':
                extract_cam_info(bag, outputdir)
            elif datatype == 'imu':
                extract_imu_info(bag, outputdir)
            elif datatype == 'gps':
                extract_gps_info(bag, outputdir)
            elif datatype == 'info':
                extract_bag_info(bag, basepath)
            elif datatype == 'lidar':
                extract_lidar_points(bag, outputdir)
            elif datatype == 'all':
                extract_all(bag, basepath)
    # close bag and exit
    if bag is not None:
        bag.close()

if __name__ == "__main__":
    import argparse
    main(sys.argv)

