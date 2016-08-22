#!/usr/bin/env python

# System modules
import os

def trex_path(tail_path):
  # Check if TREX path is defined. If not, set from reelative
  if os.environ.has_key('TREX') == 0:
    os.environ['TREX'] = os.path.abspath(os.path.join(__file__,"../","../","../"))

  trex_home = os.environ['TREX']
  
  # Compose path
  return os.path.join(trex_home,tail_path)
