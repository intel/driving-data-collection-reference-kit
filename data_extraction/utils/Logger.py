#!/usr/bin/env python

'''
This script provides logging utility that can be used by other scripts to log 
data with four different log levels - info, error, warn, debug

Copyright (C)  2019 Intel Corporation
SPDX-License-Identifier: BSD-3-Clause 

'''

import logging

__author__ = "Nitheesh K L"
__copyright__ = "Copyright (C) 2019, Intel Corporation"
__credits__ = ["Nitheesh K L"]
__license__ = "BSD-3-Clause"
__version__ = "1.0.0"
__maintainer__ = "Nitheesh K L"
__email__ = "nitheesh.k.l@intel.com"
__status__ = "Dev"

logger = None
handler = None

LEVEL_INFO = "info"
LEVEL_DEBUG = "debug"
LEVEL_WARN = "warn"
LEVEL_ERROR = "error"

def debug(msg):
    if logger is not None:
        logger.debug(msg)

def error(msg):
    if logger is not None:
        logger.error(msg)

def warn(msg):
    if logger is not None:
        logger.warn(msg)

def info(msg):
    if logger is not None:
        logger.info(msg)

def init(logfile=None, level="info", name=__name__):
    global logger
    global handler
    logger = logging.getLogger(name)
    if "error" in level.lower():
        logger.setLevel(logging.ERROR)
    elif "warn" in level.lower():
        logger.setLevel(logging.WARNING)
    elif "debug" in level.lower():
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.INFO)

    if logfile is not None:
        handler = logging.FileHandler(logfile)
        handler.setLevel(logger.level)
    else:
        handler = logging.StreamHandler()
        handler.setLevel(logger.level)

    formatter = logging.Formatter('%(asctime)s - %(name)s-[%(levelname)s]: %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)

    return logger

def setLevel(level):
    global logger
    global handler
    if "error" in level.lower():
        logger.setLevel(logging.ERROR)
    elif "warn" in level.lower():
        logger.setLevel(logging.WARNING)
    elif "debug" in level.lower():
        logger.setLevel(logging.DEBUG)
    else:
        logger.setLevel(logging.INFO)

    handler.setLevel(logger.level)

    return logger

def main(argv):
    parser = argparse.ArgumentParser(description="Logger utility")
    parser.add_argument("level", help="log levels: [info|warn|debug|error]")
    parser.add_argument("message", help="message to be logged")
    parser.add_argument("-f", "--file", help="file to be used for logging. Default is log to console")
    args = parser.parse_args()

    if args.level is None:
        print("log level is required")
        return

    if args.message is None:
        print("No message to log")
        return

    init(logfile=args.file, level=args.level)

    if "error" in args.level:
        error(args.message)
    elif "warn" in args.level:
        warn(args.message)
    elif "debug" in args.level:
        debug(args.message)
    elif "info" in args.level:
        info(args.message)
    else:
        print("error: unkown log level - " + args.level)

if __name__ == "__main__":
    import sys
    import argparse
    main(sys.argv)