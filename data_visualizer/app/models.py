#!/usr/bin/env python

'''
MongoDb document/model of RawBagfile.
Each of this RawBagfile entry corresponds to an individual rosbag file
and holds its metadata. This model reflects exactly the same model used and 
uploaded by the extraction scripts.

Copyright (C)  2019 Intel Corporation
SPDX-License-Identifier: BSD-3-Clause 

'''

from flask_mongoengine import MongoEngine

__author__ = "Nitheesh K L"
__copyright__ = "Copyright (C) 2019, Intel Corporation"
__credits__ = ["Nitheesh K L"]
__license__ = "BSD-3-Clause"
__version__ = "1.0.0"
__maintainer__ = "Nitheesh K L"
__email__ = "nitheesh.k.l@intel.com"
__status__ = "Dev"

db = MongoEngine()

class RawBagfile(db.Document):
    key = db.StringField(required=True)
    vendor = db.StringField(max_length=50, required=True)
    hsize = db.StringField(max_length=50)
    hduration = db.StringField(max_length=50)
    filename = db.StringField(required=True)
    location = db.StringField(required=True)
    capturedate = db.StringField(max_length=10, required=True)
    path = db.StringField()
    info = db.DictField(required=True)
    distance = db.StringField()
