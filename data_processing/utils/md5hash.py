#!/usr/bin/env python

'''
Provides utility methods to generate md5 hash

Copyright (C)  2019 Intel Corporation
SPDX-License-Identifier: BSD-3-Clause

'''

import os
import sys
import argparse
import hashlib

__author__ = "Nitheesh K L"
__copyright__ = "Copyright (C) 2019, Intel Corporation"
__credits__ = ["Nitheesh K L"]
__license__ = "BSD-3-Clause"
__version__ = "1.0.0"
__maintainer__ = "Nitheesh K L"
__email__ = "nitheesh.k.l@intel.com"
__status__ = "Dev"

VERBOSE = False

def debugPrint(msg, inline=False):
    global VERBOSE
    if VERBOSE:
        if inline:
            msg = "\r" + msg
        else:
            msg = msg + "\n"
        sys.stdout.write(msg)

def genMd5(filename, blocksize=65536):
    debugPrint("initializing md5 hasher...")
    debugPrint("using block-size=" + str(blocksize))
    hasher = hashlib.md5()
    total = 0
    with open(filename, 'rb') as afile:
	buf = afile.read(blocksize)
	while len(buf) > 0:
            total = total + len(buf)
            debugPrint("read " + str(total) + " bytes", inline=True)
	    hasher.update(buf)
	    buf = afile.read(blocksize)
       
        hash_digest = hasher.hexdigest()
        debugPrint("\n" + hash_digest)
    return hash_digest

def main(argv):
    parser = argparse.ArgumentParser(description="Generates md5 hash for the given file")
    parser.add_argument("filepath", help="path to file")
    parser.add_argument("-b", "--bsize", help="block size for reading file")
    parser.add_argument("-v", "--verbose", action="store_true", help="enable verbose outputs")
    args = parser.parse_args()

    global VERBOSE
    VERBOSE = args.verbose

    if args.filepath is None:
        print("file not specified!")
        return

    if not os.path.exists(args.filepath):
        print("file doesn't exist - " + args.filepath)
        return

    if args.bsize is None:
        print genMd5(args.filepath)
    else:
        print genMd5(args.filepath, blocksize=int(args.bsize))

if __name__ == "__main__":
    main(sys.argv)