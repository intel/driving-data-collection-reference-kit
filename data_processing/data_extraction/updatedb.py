#!/usr/bin/env python

'''
Updates db will all info.json files found in the provided dir path.
This is gnerally used to update db periodically from all info.json files available in dataset
Params:
    dataset_path - dir containing rosbag file to search for corresponding info.json files

Copyright (C)  2019 Intel Corporation
SPDX-License-Identifier: BSD-3-Clause 

'''

# file handling
import os
import fnmatch
import json # handle json data
from mongoengine import * # handle mongodb
import humanfriendly # convert time and number to human readable
from tqdm import tqdm # progress bar

# if called as standlone script, add top dir to syspat to import our modules
if __name__ == "__main__":
    import sys
    repoPath = os.path.dirname(os.path.realpath(__file__)) + "/../"
    sys.path.append(repoPath)
# else, we presume the modules are organized and available
from model.RawBagfile import RawBagfile # MongoDb model for rosbag
from model.setupdb import setupdb # to setup mongodb connection
from utils.md5hash import genMd5 # util to generate MD5 hash
from utils import Logger # Logging utils

__author__ = "Nitheesh K L"
__copyright__ = "Copyright (C) 2019, Intel Corporation"
__credits__ = ["Nitheesh K L"]
__license__ = "BSD-3-Clause"
__version__ = "1.0.0"
__maintainer__ = "Nitheesh K L"
__email__ = "nitheesh.k.l@intel.com"
__status__ = "Dev"

def update_all_info(dataset_path):
    # find all info.json files in the given path
    Logger.debug('fetching info.json files in ' + dataset_path)
    matches = []
    for root,dirnames,filenames in os.walk(dataset_path):
        for filename in fnmatch.filter(filenames, '*.bag'):
            matches.append(os.path.join(root, filename))
            Logger.debug("")
            Logger.debug(os.path.join(root, filename))

    Logger.debug("")
    Logger.debug("found " + str(len(matches)) + " info.json files")

    # connect to wwe database
    Logger.debug("connecting to database...")
    setupdb()

    # update db from each info.json
    for bagfile in tqdm(matches, unit="file", desc="updating db"):
        infofile = bagfile.replace("rosbags", "raw", 1)
        infofile = infofile.replace(".bag", "/info.json", 1)
        if not os.path.exists(infofile):
            Logger.warn("info file not found - " + infofile)
            continue
        baginfo = json.load(open(infofile))
        # split infofile path. ex: wwe/raw/intel/<...>/2018_01_11/2018-01-11_15-40-44/info.json
        pathinfo = infofile.split("/")
        vendor = pathinfo[1]
        cdate = pathinfo[-3]
        size = humanfriendly.format_size(baginfo['size'])
        duration = humanfriendly.format_timespan(baginfo['duration'])
        name = baginfo['path'].split("/")[-1]
        name = name[:10] + "_" + name[11:]
        dist = "0"
        if 'distance' in baginfo and 'dist' in baginfo['distance']:
            dist = "{0:.5f} ".format(baginfo['distance']['dist']) + baginfo['distance']['unit']
            Logger.debug("adding distance: " + dist)

        # add relevant locations as necessary
        # TODO: pickup locations from a config file instead of manual checks
        if "bangalore" in infofile:
            loc = "bangalore"
        elif "telangana" in infofile:
            loc = "hyderabad"
        elif "hyderabad" in infofile:
            loc = "hyderabad"
        else:
            loc = "unknown"

        rawbagfile = RawBagfile(
                key = genMd5(infofile),
                vendor = vendor, 
                hsize = size, 
                hduration = duration, 
                filename = name, 
                location = loc, 
                capturedate = cdate, 
                path = infofile,
                distance = dist,
                info = baginfo
                )
        duplicateFound = False
        # for existing entries
        for bagfile in RawBagfile.objects(key=rawbagfile.key):
            Logger.warn("found entry with duplicate key - " + bagfile.key)
            duplicateFound = True
            bagfile.delete()
        for bagfile in RawBagfile.objects(filename=rawbagfile.filename):
            Logger.warn("found entry with duplicate filename - " + bagfile.filename)
            duplicateFound = True
            bagfile.delete()

        # save new info if no duplicates
        #if not duplicateFound:
        #    debugPrint("udpating db with new info...")
        rawbagfile.save()

'''
Update db with info from the provided info.json file
Params:
    infofile - info.json file whose data has to be inserted into db
'''
def update_single_info(infofile):
    Logger.debug('reading info from ' + infofile)
    with open(infofile, 'r') as f:
        baginfo = json.load(f)

    # connect to wwe database
    Logger.debug("connecting to database...")
    setupdb()

    # split infofile path. ex: raw_dataset/intel/bangalore/2018_01_11/2018-01-11_15-40-44/info.json
    pathinfo = infofile.split("/")
    vendor = pathinfo[1]
    cdate = pathinfo[-3]
    size = humanfriendly.format_size(baginfo['size'])
    duration = humanfriendly.format_timespan(baginfo['duration'])
    name = baginfo['path'].split("/")[-1]
    name = name[:10] + "_" + name[11:]
    dist = "0"
    if 'distance' in baginfo and 'dist' in baginfo['distance']:
        dist = "{0:.5f} ".format(baginfo['distance']['dist']) + baginfo['distance']['unit']
        Logger.debug("adding distance: " + dist)


    # add relevant locations as necessary
    # TODO: pickup locations from a config file instead of manual checks
    if "bangalore" in infofile:
        loc = "bangalore"
    elif "telangana" in infofile:
        loc = "hyderabad"
    elif "hyderabad" in infofile:
        loc = "hyderabad"
    else:
        loc = "unknown"

    rawbagfile = RawBagfile(
            key = genMd5(infofile),
            vendor = vendor,
            hsize = size,
            hduration = duration,
            filename = name,
            location = loc,
            capturedate = cdate,
            path = infofile,
            distance = dist,
            info = baginfo
            )
    duplicateFound = False
    # for existing entries
    for bagfile in RawBagfile.objects(key=rawbagfile.key):
        Logger.warn("found entry with duplicate key - " + bagfile.key)
        duplicateFound = True
        bagfile.delete()
    for bagfile in RawBagfile.objects(filename=rawbagfile.filename):
        Logger.warn("found entry with duplicate filename - " + bagfile.filename)
        duplicateFound = True
        bagfile.delete()

    # save new info if no duplicates
    #if not duplicateFound:
    #    debugPrint("udpating db with new info...")
    rawbagfile.save()

'''
Deletes all RawBagfile entries from DB
'''
def flushdb():
    # connecting to wwe database
    Logger.debug("connecting to database...")
    setupdb()

    Logger.debug("proceeding to delete entires...")
    for bagfile in RawBagfile.objects():
        Logger.debug("deleting " + bagfile.filename)
        bagfile.delete()

    Logger.debug("finished!")

'''
Find database entries of records having the provided filename
Params:
    name - Filename/bagname to search in db
'''
def findFilename(name):
    # connecting to wwe database
    Logger.debug("connecting to database...")
    setupdb()

    for bagfile in RawBagfile.objects(filename=name):
        print("filename = " + bagfile.filename)
        print("key = " + bagfile.key)
        print("vendor = " + bagfile.vendor)
        print("hsize = " + bagfile.hsize)
        print("hduration = " + bagfile.hduration)
        print("location = " + bagfile.location)
        print("capturedate = " + bagfile.capturedate)
        print("path = " + bagfile.path)
        print("distance = " + bagfile.distance)

def main(argv):
    parser = argparse.ArgumentParser(description="Update wwe db with rosbag info files", formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('action', help='\
action to perform.[all|single]\n\
all - to updated db with all info.json files found recursively in the given path.\n\
single - to update db with a single info json file.\n\
flush - removes all entries from the db.\n\
find - find and print details of the entry in db containing the provided filename.')
    parser.add_argument('-d', '--dataset_path', help='path to wwe raw dataset dir to search for info.json files')
    parser.add_argument('-i', '--info', help='info.json file whose content has to be added to the existing db')
    parser.add_argument('-f', '--filename', help='filename to search in db')
    parser.add_argument('-v', '--verbose', action='store_true', help='enable verbose outputs')
    args = parser.parse_args()

    # Initialized logger
    Logger.init(level=Logger.LEVEL_INFO, name="updateDB")
    if args.verbose:
        Logger.setLevel(Logger.LEVEL_DEBUG)

    if args.action == 'all':
        if args.dataset_path is not None:
            update_all_info(args.dataset_path)
        else:
            Logger.error("Dataset path not provided!")
            return
    elif args.action == 'single':
        if args.info is not None:
            update_single_info(args.info)
        else:
            Logger.error("info.json file not provided to update db!")
            return
    elif args.action == "flush":
        flushdb()
    elif args.action == "find":
        if args.filename is not None:
            findFilename(args.filename)
        else:
            Logger.error("filename not provided to search db")
            return
    else:
        Logger.error("unknown action - " + args.action)

if __name__ == "__main__":
    import argparse
    main(sys.argv)