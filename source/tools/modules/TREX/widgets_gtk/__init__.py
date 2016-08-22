# System modules
import os

# Import utilities
import TREX.util

# Defile utility to get a widget gladefile path
def trex_glade_path(glade_file):
  return os.path.join(os.path.dirname(__file__),glade_file)

# Import gtk window classes
from TREX.widgets_gtk.db_reader_window import DbReaderWindow
from TREX.widgets_gtk.token_network_window import TokenNetworkWindow
from TREX.widgets_gtk.token_network_filter_window import TokenNetworkFilterWindow
from TREX.widgets_gtk.property_window import PropertyWindow,PropertyWindowFactory
from TREX.widgets_gtk.timeline_window import TimelineWindow
from TREX.widgets_gtk.conflict_list_window import ConflictListWindow
