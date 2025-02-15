# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.



#
# Integrated tests
#
# Purpose:  This script calls any scripts in the tools folder to make sure they execute without error
#           that they complete properly.
# Author:   Hanspeter Schaub
# Creation Date:  May 26, 2020
#


import shutil
import subprocess
import sys
from pathlib import Path

import pytest


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")


# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("toolCase", [
    'fswAuto/autosetter/autosetter.py'
    , 'fswAuto/autowrapper/autowrapper.py'
])

def test_toolsFolder(show_plots, toolCase):

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    pathFile = Path(__file__).parents[2] / "externalTools" / toolCase
    shellCmd = [sys.executable, str(pathFile)]
    try:
        subprocess.check_call(shellCmd)

        # remove output folders if they exist
        pathOutFolder = pathFile.parent / "outputFiles"
        if pathOutFolder.exists():
            shutil.rmtree(pathOutFolder)

    except subprocess.CalledProcessError:
        testFailCount = testFailCount + 1
        testMessages.append(toolCase + " failed to run.")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found

    assert testFailCount < 1, testMessages

