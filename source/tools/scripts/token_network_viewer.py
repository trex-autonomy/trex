#!/usr/bin/env python

# System modules
import sys,os
import unittest
import signal
import threading, time
import gtk

# TREX modules
from TREX.core import *
from TREX.widgets_gtk import *

def main():

  # Process cli arguments
  if len(sys.argv) > 1:
    log_path = sys.argv[1]
  else:
    log_path = "."

  # Window configuration for a window that should not be allowed to be closed
  WINDOW_FUNCTIONS = gtk.gdk.FUNC_MOVE | gtk.gdk.FUNC_RESIZE | gtk.gdk.FUNC_MINIMIZE | gtk.gdk.FUNC_MAXIMIZE;

  # Initialize gtk multithread support
  gtk.gdk.threads_init()

  # Create db reader window
  db_reader_window = DbReaderWindow(log_path=log_path)
  db_reader_window.w.connect("destroy",gtk.main_quit)
  db_reader_window.assembly_ticks_only_check.set_active(True)
  db_reader_window.assembly_ticks_only = True

  # Create token network graph generator
  token_network = TokenNetwork()
  db_reader_window.register_listener(token_network.set_db_cores)

  # Create token network window
  token_network_window = TokenNetworkWindow(token_network)
  token_network_window.register_listener(PropertyWindowFactory)
  token_network_window.window.set_functions(WINDOW_FUNCTIONS)

  # Create token network filter window
  token_network_filter = TokenNetworkFilter(token_network)
  token_network_filter_window = TokenNetworkFilterWindow(token_network_filter)
  token_network_filter_window.w.window.set_functions(WINDOW_FUNCTIONS)

  db_reader_window.w.present()

  # Set up signal handler
  signal.signal(signal.SIGINT, signal.SIG_DFL) 

  gtk.main()

if __name__ == '__main__':
  main()
