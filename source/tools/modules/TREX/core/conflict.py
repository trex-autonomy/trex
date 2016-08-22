#!/usr/bin/env python

# System modules
import unittest

# TREX modules
from TREX.core.assembly import Assembly,Token

class Conflict():
  def __init__(self, summary="TREX Conflict", analysis="Don't steal her eggs, or else you'll be in one."):
    self.summary = summary
    self.analysis = analysis
