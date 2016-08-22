#!/usr/bin/env python

# System modules
import unittest

# TREX modules
from TREX.core.assembly import Assembly,Token
from TREX.core.conflict import Conflict

class Timeline():
  # Timeline constants
  INTERNAL, EXTERNAL = range(2)

  def __init__(self,key,name,mode):
    self.key = key
    self.name = name
    self.mode = mode

    self.tokens = []

class DbCore():
  def __init__(self,tick=0,reactor_name=""):
    # Create storage structures
    self.clear()

    # Initialize properties
    self.tick = tick
    self.reactor_name = reactor_name
    self.conflict = None

  def clear(self):
    # Clear timelines
    self.int_timelines = {}
    self.ext_timelines = {}

    # Clear assembly
    self.assembly = Assembly()

# Test utilities
def construct_test_db_core():
  # Create assembly
  from TREX.core.assembly import construct_test_assembly
  assembly = construct_test_assembly()

  # Create db_core
  db_core = DbCore(0,"test")

  # Store assembly
  db_core.assembly = assembly

  # Create timelines
  tl1 = Timeline(0,"tl1",Timeline.INTERNAL)
  tl2 = Timeline(0,"tl2",Timeline.EXTERNAL)

  db_core.int_timelines["tl1"] = tl1
  db_core.ext_timelines["tl2"] = tl2

  # Create tokens
  tl1.tokens = [assembly.tokens[0]]
  tl2.tokens = [assembly.tokens[1]]

  return assembly
  
# Unit tests
class TestDbCoreStructures(unittest.TestCase):
  def test_construct(self):
    construct_test_db_core()

if __name__ == '__main__':
  unittest.main()
