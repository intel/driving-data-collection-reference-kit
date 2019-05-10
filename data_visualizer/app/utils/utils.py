#!/usr/bin/env python

'''
Proivdes a set of utility methods to format and manipulate data to be shown 
on the web pages.

Copyright (C)  2019 Intel Corporation
SPDX-License-Identifier: BSD-3-Clause 

'''

import fnmatch
import os
import yaml
import csv
import json
from datetime import datetime
import json
from app.models import RawBagfile
import humanfriendly
import pandas as pd

__author__ = "Nitheesh K L"
__copyright__ = "Copyright (C) 2019, Intel Corporation"
__credits__ = ["Nitheesh K L"]
__license__ = "BSD-3-Clause"
__version__ = "1.0.0"
__maintainer__ = "Nitheesh K L"
__email__ = "nitheesh.k.l@intel.com"
__status__ = "Dev"

def findRosbags(path):
    matches = []
    for root, dirnames, filenames in os.walk(path):
        for filename in fnmatch.filter(filenames, '*.bag'):
            matches.append(os.path.join(root, filename))

    matches.sort(reverse=True)
    rosbags = {}
    for bagfile in matches:
        extracted_dir = os.path.splitext(bagfile)[0]
        info_file = extracted_dir + "/info.json"
        if os.path.exists(info_file):
            baginfo = json.load(open(info_file))
        else:
            baginfo = None
        rosbags[bagfile] = baginfo

    return rosbags


def getRosbagInfo(name=None):
    result = {}
    if name is not None:
        for bagfile in RawBagfile.objects(filename=name):
            result[bagfile.filename] = bagfile.info
    return result

def getDashboardInfo():
    def getPhase(filepath):
        if 'pilot' in filepath:
            return 'pilot'
        elif 'phase1' in filepath:
            return 'phase1'
        elif 'phase2' in filepath:
            return 'phase2'
        else:
            return 'unknown'

    raw = {}
    raw['phases'] = {}
    for bagfile in RawBagfile.objects:
        phase = getPhase(bagfile.path)
        if phase not in raw['phases']:
            p = {}
            p["locations"] = {}
            p["total_size"] = 0
            p["total_duration"] = 0
            p["total_distance"] = 0
            p["total_frames"] = 0
            raw["phases"][phase] = p
        if bagfile.location not in raw["phases"][phase]["locations"]:
            loc = {}
            loc["fileinfo"] = []
            loc["total_size"] = 0
            loc["total_duration"] = 0
            loc["total_distance"] = 0
            loc["total_frames"] = 0
            raw["phases"][phase]["locations"][bagfile.location] = loc
        frames = 0
        for item in bagfile.info["topics"]:
            if "frontNear/left/image_raw" in item["topic"]:
                frames = item["messages"]
                break
        raw["phases"][phase]["locations"][bagfile.location]["fileinfo"].append({
            'name': bagfile.filename,
            'size': bagfile.hsize,
            'duration': bagfile.hduration,
            'distance': bagfile.distance,
            'frames': frames
        })
        dst = 0
        if bagfile.distance != "":
            dst = float(bagfile.distance.split(" ")[0])
        # total for each location
        raw["phases"][phase]["locations"][bagfile.location]["total_size"]      += bagfile.info['size']
        raw["phases"][phase]["locations"][bagfile.location]["total_duration"]  += bagfile.info['duration']
        raw["phases"][phase]["locations"][bagfile.location]["total_distance"]  += dst
        raw["phases"][phase]["locations"][bagfile.location]["total_frames"]    += frames
        # total for each phase
        raw["phases"][phase]["total_size"]        += bagfile.info['size']
        raw["phases"][phase]["total_duration"]    += bagfile.info['duration']
        raw["phases"][phase]["total_distance"]    += dst
        raw["phases"][phase]["total_frames"]      += frames

    # totals for all data
    total_size = 0
    total_duration = 0
    total_distance = 0
    total_frames = 0

    for phase in raw["phases"]:
        for location in raw["phases"][phase]["locations"]:
            m, s = divmod(raw["phases"][phase]["locations"][location]["total_duration"], 60)
            h, m = divmod(m, 60)
            raw["phases"][phase]["locations"][location]["total_size"] = humanfriendly.format_size(
                raw["phases"][phase]["locations"][location]["total_size"])
            raw["phases"][phase]["locations"][location]["total_duration"] = str(
                int(h)) + "hrs " + str(int(m)) + " mins " + str(int(s)) + " sec"
            raw["phases"][phase]["locations"][location]["total_distance"] = humanfriendly.format_number(
                raw["phases"][phase]["locations"][location]["total_distance"])
            raw["phases"][phase]["locations"][location]["total_frames"] = humanfriendly.format_number(
                raw["phases"][phase]["locations"][location]["total_frames"])
        total_size += raw["phases"][phase]["total_size"]
        total_duration += raw["phases"][phase]["total_duration"]
        total_distance += raw["phases"][phase]["total_distance"]
        total_frames += raw["phases"][phase]["total_frames"]
        m, s = divmod(raw["phases"][phase]["total_duration"], 60)
        h, m = divmod(m, 60)
        raw["phases"][phase]["total_duration"] = str(int(h)) + "hrs " + str(int(m)) + " mins " + str(int(s)) + " sec"
        raw["phases"][phase]["total_size"] = humanfriendly.format_size(raw["phases"][phase]["total_size"])
        raw["phases"][phase]["total_distance"] = humanfriendly.format_number(raw["phases"][phase]["total_distance"])
        raw["phases"][phase]["total_frames"] = humanfriendly.format_number(raw["phases"][phase]["total_frames"])
    bagfiles_info = {}
    bagfiles_info['raw'] = raw
    m, s = divmod(total_duration, 60)
    h, m = divmod(m, 60)
    bagfiles_info['total_size'] = humanfriendly.format_size(total_size)
    bagfiles_info['total_duration'] = str(h) + " hours"
    bagfiles_info['total_distance'] = humanfriendly.format_number(
        total_distance)
    bagfiles_info['total_frames'] = humanfriendly.format_number(total_frames)
    return bagfiles_info
