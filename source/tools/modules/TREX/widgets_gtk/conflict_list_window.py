#!/usr/bin/env python

# System modules
import sys
import os,stat
import shutil

import time
import threading

import gtk
import gtk.glade

import unittest

# TREX modules
import TREX.util
from TREX.widgets_gtk import trex_glade_path
from TREX.core.db_core import DbCore
from TREX.core.conflict import Conflict

##############################################################################
# ConflictListWindow
#   This window provides a user with controls to step to any tick and re-run
#   the data loader. It also does continuous polling of the file system in
#   order to grab data for visualization as it is written to disk.
##############################################################################

class ConflictListWindow():
  def __init__(self,conflict = None):
    # Store db reader
    self.conflict = conflict

    # Create glade window
    tree = gtk.glade.XML(trex_glade_path("conflict_list_window.glade"))
    self.w = tree.get_widget("conflict_list_window")
    self.w.set_title("Conflicts")

    # Add references to all widgets
    for w in tree.get_widget_prefix('_'):
      name = w.get_name()[1:]
      # Make sure we don't clobber existing attributes
      assert not hasattr(self, name)
      setattr(self, name, w)

    # Create liststore model
    self.create_model()

    # Create tree view
    self.conflict_view.set_model(self.store)
    self.conflict_view.connect('row-activated', self.on_activate)
    self.create_columns(self.conflict_view)

    self.w.show()

  ############################################################################
  # Construction utilities
  ############################################################################

  # Initialize filter list
  def create_model(self):
    self.store = gtk.ListStore(str, str, str)

  # Create list view columns
  def create_columns(self, treeView):
    # Tick column
    rendererText = gtk.CellRendererText()
    column = gtk.TreeViewColumn("Tick", rendererText, text=0)
    column.set_sort_column_id(0)    
    treeView.append_column(column)

    # Step column
    rendererText = gtk.CellRendererText()
    column = gtk.TreeViewColumn("Step", rendererText, text=1)
    column.set_sort_column_id(1)    
    treeView.append_column(column)

    # Reactor column
    rendererText = gtk.CellRendererText()
    column = gtk.TreeViewColumn("Reactor", rendererText, text=2)
    column.set_sort_column_id(2)    
    treeView.append_column(column)

  ############################################################################
  # Conflict management
  ############################################################################

  def set_db_cores(self,db_cores,reactor_name):
    # Clear conflicts
    self.store.clear()

    # Re-populate
    for reactor,db_core in db_cores.items():
      for conflict in db_core.conflicts:
	self.store.append([ str(conflict[0]), str(conflict[1]), reactor])

      if db_core.conflict:
	buf = self.analysis_view.get_buffer()

	buf.set_text(db_core.conflict.summary + "\n\n" + db_core.conflict.analysis)

  ############################################################################
  # Event Handlers
  ############################################################################

  def on_activate(self,widget,row,col):
    rowdata = self.store[str(row[0])]
    tick = (int(rowdata[0]),int(rowdata[1]))
    reactor = rowdata[2]
    self.activate_cb(tick,reactor)

  def register_activate_callback(self,cb):
    self.activate_cb = cb


# Testing utilities
class _GtkTester():
  def spawn_gtk_thread(self):
    # Spawn the window
    gtk_thread = threading.Thread(target=self.gtk_thread)
    gtk_thread.start()

  def gtk_thread(self):
    # Spawn a thread to run gtk in
    print "Spawning gtk thread..."
    self.conflict_window.w.connect("destroy",gtk.main_quit)
    gtk.main()

# Unit tests
class TestConflictListWindow(unittest.TestCase,_GtkTester):
  # Create the gtk thread and window structure
  def setUp(self):
    # Initialize GTK Python threading functionality
    gtk.gdk.threads_init()
    # Create a new db reader window
    self.conflict_window = ConflictListWindow()
    # Spawn the window
    self.spawn_gtk_thread()
    
  # Destroy window and kill gtk
  def tearDown(self):
    print "Killing The window..."
    self.conflict_window.w.destroy()
    time.sleep(5)

  # Test basic addition and removal of filters
  def test_push_network(self):
    from TREX.core.assembly import construct_test_assembly

    time.sleep(10)

if __name__ == '__main__':
  unittest.main()
