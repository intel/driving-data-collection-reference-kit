#!/usr/bin/env python

'''
MongoDb document/model of RawBagfile.
Each of this RawBagfile entry corresponds to an individual rosbag file
and holds its metadata.

Copyright (C)  2019 Intel Corporation
SPDX-License-Identifier: BSD-3-Clause 

'''

from mongoengine import *

__author__ = "Nitheesh K L"
__copyright__ = "Copyright (C) 2019, Intel Corporation"
__credits__ = ["Nitheesh K L"]
__license__ = "BSD-3-Clause"
__version__ = "1.0.0"
__maintainer__ = "Nitheesh K L"
__email__ = "nitheesh.k.l@intel.com"
__status__ = "Dev"

class RawBagfile(Document):
    # unique key. In our case a unique MD5 hash generated for the bagfile
    key = StringField(required=True)
    # vendo who recorded the bagfile
    vendor = StringField(max_length=50, required=True)
    # human readable string of the bag's size
    hsize = StringField(max_length=50)
    # human readable string of the bag's duration
    hduration = StringField(max_length=50)
    # name of the bagfile.
    filename = StringField(required=True)
    # location where data was captured.
    location = StringField(required=True)
    # date of capture of the bagfile
    capturedate = StringField(max_length=10, required=True)
    # file path on disk of the bagfile
    path = StringField()
    # rosbag info as json structure
    info = DictField(required=True)
    # distance travled for data recording
    distance = StringField()