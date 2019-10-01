#!/usr/bin/env python
import os
import time
import sys
import re
import matplotlib.pyplot as plt
import glob

cmd0 = "mpiexec.openmpi --allow-run-as-root -np 6 python ppo_ag_general_testV2.py --scenario 3 3 3 3 3 3 --rosports 11712 11712 11712 11712 11712 11712 --robotIds 0 1 2 3 4 5 --fileId 9"
cmd1 = "mpiexec.openmpi --allow-run-as-root -np 6 python ppo_ag_general_ttV2.py --scenario 3 3 3 3 3 3 --rosports 11713 11713 11713 11713 11713 11713 --robotIds 0 1 2 3 4 5 --fileId 77777"


os.system("gnome-terminal -e 'bash -c \"%s\" '"%cmd)
