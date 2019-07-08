#!/usr/bin/env python

'''
This script helps to calculate the distance between two gps coordinates.

Copyright (C)  2019 Intel Corporation
SPDX-License-Identifier: BSD-3-Clause 

'''

import os
import sys
import json
import math

if __name__ == "__main__":
    import sys
    repoPath = os.path.dirname(os.path.realpath(__file__)) + "/../"
    sys.path.append(repoPath)

from utils import Logger

__author__ = "Nitheesh K L"
__copyright__ = "Copyright (C) 2019, Intel Corporation"
__credits__ = ["Nitheesh K L, stackoverflow (collated from couple of answers)"]
__license__ = "BSD-3-Clause"
__version__ = "1.0.0"
__maintainer__ = "Nitheesh K L"
__email__ = "nitheesh.k.l@intel.com"
__status__ = "Dev"

def degreesToRadians(degrees):
  return degrees * math.pi / 180

def distanceInKmBetweenEarthCoordinates(lat1, lon1, lat2, lon2):
  earthRadiusKm = 6371

  dLat = degreesToRadians(lat2-lat1)
  dLon = degreesToRadians(lon2-lon1)

  lat1 = degreesToRadians(lat1)
  lat2 = degreesToRadians(lat2)

  a = math.sin(dLat/2) * math.sin(dLat/2) + math.sin(dLon/2) * math.sin(dLon/2) * math.cos(lat1) * math.cos(lat2)
  c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
  
  return earthRadiusKm * c

def calcDistance(data):
    coords = []
    for msg in data:
        lat = msg["latitude"]
        lng = msg["longitude"]
        coords.append({"lat":lat, "lng":lng})

    #Logger.debug("read " + str(len(coords)) + " coordinate points")
    
    distance = 0.0
    previous = coords[0]
    for current in coords:
        lat1 = previous["lat"]
        lng1 = previous["lng"]
        lat2 = current["lat"]
        lng2 = current["lng"]
        dst = distanceInKmBetweenEarthCoordinates(lat1, lng1, lat2, lng2)
        distance += dst
        previous = current

    return distance

def doAction(gpsFilePath):
    Logger.debug("reading gps data from " + gpsFilePath)
    gpsData = None
    with open(gpsFilePath) as f:
        gpsData = json.load(f)
    
    if gpsData == None or gpsData["data"] == None:
        Logger.error("No gps data found in " + gpsFilePath)
        return

    data = gpsData["data"]
    distance = calcDistance(data)

    Logger.debug("distance = " + str(distance) + " km")
    return distance

def main(argv):
    parser = argparse.ArgumentParser(description="Calculate distance in km for given gps file")
    parser.add_argument("filepath", help="path to gps json file")
    parser.add_argument("-v", "--verbose", action="store_true", help="enable verbose outputs")
    args = parser.parse_args()

    Logger.init(level=Logger.LEVEL_INFO, name="GpsDistance")
    if args.verbose:
        Logger.setLevel(Logger.LEVEL_DEBUG)

    if args.filepath is None:
        print("file not specified!")
        return

    if not os.path.exists(args.filepath):
        print("file doesn't exist - " + args.filepath)
        return

    doAction(args.filepath)

if __name__ == "__main__":
    import argparse
    main(sys.argv)