#!/usr/bin/env python

'''
This script is part of the falsk app to renders the requested web urls.

Copyright (C)  2019 Intel Corporation
SPDX-License-Identifier: BSD-3-Clause 

'''

from flask import render_template, request, jsonify, send_file, Response, make_response
from app import app
from utils import utils
import os
import io
import json

__author__ = "Nitheesh K L"
__copyright__ = "Copyright (C) 2019, Intel Corporation"
__credits__ = ["Nitheesh K L"]
__license__ = "BSD-3-Clause"
__version__ = "1.0.0"
__maintainer__ = "Nitheesh K L"
__email__ = "nitheesh.k.l@intel.com"
__status__ = "Dev"

@app.route('/')
@app.route('/index')
@app.route('/index.html')
def index():
    dashboard_info = utils.getDashboardInfo()
    return render_template('index.html', dashboard_info=dashboard_info)