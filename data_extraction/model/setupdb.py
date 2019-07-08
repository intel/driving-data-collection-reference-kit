#!/usr/bin/env python

'''
Utility script to setup mongodb connection to wwe database

Copyright (C)  2019 Intel Corporation
SPDX-License-Identifier: BSD-3-Clause

'''

from mongoengine import connect

__author__ = "Nitheesh K L"
__copyright__ = "Copyright (C) 2019, Intel Corporation"
__credits__ = ["Nitheesh K L"]
__license__ = "BSD-3-Clause"
__version__ = "1.0.0"
__maintainer__ = "Nitheesh K L"
__email__ = "nitheesh.k.l@intel.com"
__status__ = "Dev"

def setupdb():
    # connects to the localhost with default port.
    connect("wwe")

if __name__ == "__main__":
    setupdb()
