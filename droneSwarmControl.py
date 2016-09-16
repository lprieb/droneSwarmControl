import serial
import xbee
import logging
import threading
from pymavlink import mavutil
import types
import dronekit
import tempfile
import socket
import threading
import time
import sys
import traceback
import serial.tools.list_ports
import argparse
import errno
from Queue import Queue, Empty
import gi
gi.require_version('Gtk', '3.0') # Checks that appropriate version is running
from gi.repository import Gtk, GLib, Gdk, Pango
import functools
import inspect
import math

'''
 #From http://downgra.de/2009/05/16/python-monkey-patching/
def wrap(orig_func):
    """ decorator to wrap an existing method of a class.
        e.g.

        @wrap(Post.write)
        def verbose_write(forig, self):
            print 'generating post: %s (from: %s)' % (self.title,
                                                      self.filename)
            return forig(self)

        the first parameter of the new function is the the original,
        overwritten function ('forig').
    """

    # har, some funky python magic NOW!
    @functools.wraps(orig_func)
    def outer(new_func):
        def wrapper(*args, **kwargs):
            return new_func(orig_func, *args, **kwargs)
        if inspect.ismethod(orig_func):
            setattr(orig_func.im_class, orig_func.__name__, wrapper)
        return wrapper
    return outer
'''
# Failed attempt to monkey patch error printing function. Left here for future reference


# Utility Functions obtained from http://stackoverflow.com/questions/31174295/getattr-and-setattr-on-nested-objects
def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)

sentinel = object()
def rgetattr(obj, attr, default=sentinel):
    if default is sentinel:
        _getattr = getattr
    else:
        def _getattr(obj, name):
            return getattr(obj, name, default)
    return functools.reduce(_getattr, [obj]+attr.split('.'))



class GuiManager(Gtk.Window):
    def __init__(self, nlinks, logger):
        super(GuiManager, self).__init__(title="Swarm System")
        self.nlinks = nlinks
        self.nlink_gui_boxes = list()
        self.logger = logger
        self.set_border_width(10)

        self.vbox1= Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        self.add(self.vbox1)

        self.hbox1= Gtk.Box(spacing=6)
        self.hbox1.set_vexpand(False)
        self.vbox1.pack_start(self.hbox1, True, True, 0)
        self.hbox1.set_hexpand(True)

        self.map = Gtk.Image.new_from_file("earthmap.jpg")

        self.scrolled = Gtk.ScrolledWindow()
        self.scrolled.set_hexpand(False)
        self.scrolled.set_vexpand(True)
        self.scrolled.set_size_request(500,-1)
        self.devices_vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        self.devices_vbox.set_homogeneous(False)
        self.scrolled.add(self.devices_vbox)

        self.hbox1.pack_start(self.map, False, False, 0)
        self.hbox1.pack_start(self.scrolled, False, False,0)
        self.map.set_hexpand(True)


        #List Boxes allow to create separted boxes inside a regular box
        self.global_command_list_box = Gtk.ListBox()
        self.global_command_list_box.set_selection_mode(Gtk.SelectionMode.NONE)

        list_row = Gtk.ListBoxRow()
        list_row.add(global_command_box(self.nlinks, self.logger))
        self.global_command_list_box.add(list_row)

        self.devices_vbox.pack_start(self.global_command_list_box, True, True, 0)

        # Log Box
        self.log_scroll_window = Gtk.ScrolledWindow()
        self.log_scroll_window.set_hexpand(False)
        self.log_scroll_window.set_vexpand(True)
        self.log_scroll_window.set_size_request(-1,200)
        self.log_textview = Gtk.TextView()
        self.log_textview.set_editable(False)
        self.log_textview.set_cursor_visible(False)
        self.log_textbuffer = self.log_textview.get_buffer() 
        # Functions for stream compatability
        self.log_textbuffer.write = lambda text: GLib.idle_add(self.writing_function_callback, text)
        self.log_textbuffer.flush = lambda : None

        self.log_scroll_window.add(self.log_textview)

        self.vbox1.pack_start(self.log_scroll_window, True, True, 0)

    def get_log_buffer(self):
        return self.log_textbuffer
    def add_nlink_box(self, nlink):
        if not self.find_nlink(nlink):
            self.logger.debug("Adding Device Box...")
            temp_nlink_box = nLinkBox(nlink, self.logger)
            self.devices_vbox.pack_start(temp_nlink_box, True, True, 0)
            self.nlink_gui_boxes.append(temp_nlink_box)
            self.show_all()

    def remove_nlink_box(self, nlink_name):
        for nlink_box in self.nlink_gui_boxes:
            if nlink_box.nlink.name == nlink_name:
                self.devices_vbox.remove(nlink_box)
                self.nlink_gui_boxes.remove(nlink_box)
                break
    def writing_function_callback(self, text):
        self.log_textbuffer.insert(self.log_textbuffer.get_end_iter(), text, -1)
        adj = self.log_scroll_window.get_vadjustment()
        adj.set_value(adj.get_upper() - adj.get_page_size())
        return False
    def find_nlink(self, nlink):
        for nlink_box in self.nlink_gui_boxes:
            if nlink_box.nlink.name == nlink.name:
                return True
        return False

# Contained class to handle actions to be performed by drones
class droneAction(object):
    def __init__(self, command, params = None):
        self.command = command
        self.params = params



class nLinkBox(Gtk.ListBox):
    def __init__(self, nlink, logger):
        super(nLinkBox, self).__init__()
        self.set_border_width(5)
        self.logger = logger
        self.nlink = nlink

        self.set_selection_mode(Gtk.SelectionMode.NONE)

        list_row = Gtk.ListBoxRow()
        self.frame = Gtk.Frame()
        self.vbox1 = Gtk.Box(spacing=6, orientation=Gtk.Orientation.VERTICAL)
        self.vbox1.override_background_color(Gtk.StateType.NORMAL, Gdk.RGBA(0.2745,0.718,0.89,1))
        self.hbox = Gtk.Box(spacing=6)
        #self.hbox.set_homogeneous(True)
        self.vbox2 = Gtk.Box(spacing=6, orientation=Gtk.Orientation.VERTICAL)
        self.vbox3 = Gtk.Box(spacing=6, orientation=Gtk.Orientation.VERTICAL)
        self.vbox4 = Gtk.Box(spacing=6, orientation=Gtk.Orientation.VERTICAL)

        # Initialize Labels
        self.latitude_label = Label(self.nlink, "Latitude", "location.global_relative_frame.lat")
        self.longitude_label = Label(self.nlink, "Longitude", "location.global_relative_frame.lon")
        self.altitude_label = Label(self.nlink, "Altitude", "location.global_relative_frame.alt")
        self.pitch_label = Label(self.nlink, "Pitch", "attitude.pitch")
        self.roll_label = Label(self.nlink, "Roll", "attitude.roll")
        self.yaw_label = Label(self.nlink, "Yaw", "attitude.yaw")
        self.velocity_label = Label(self.nlink, "Velocity","velocity")
        self.groundspeed_label = Label(self.nlink,"Groundspeed","groundspeed")
        self.airspeed_label = Label(self.nlink, "Airspeed", "airspeed")
        if not hasattr(self.nlink.drone.battery, "voltage"):
            self.nlink.drone.battery.voltage = None
        self.battery_voltage_label = Label(self.nlink,"Battery Voltage", "battery.voltage")
        self.battery_current_label = Label(self.nlink, "Battery Current", "battery.current")
        self.battery_level_label = Label(self.nlink, "Battery Level", "battery.level")
        self.last_heartbeat_label = Label(self.nlink,"Last Heartbeat","last_heartbeat")
        self.is_armable_label = Label(self.nlink,"Is Armable","is_armable")
        self.system_status_label = Label(self.nlink, "System Status", "system_status.state")
        self.mode_label = Label(self.nlink, "", "mode")
        self.armed_label = Label(self.nlink, "Armed", "armed")
        self.heading_label = Label(self.nlink, "Heading", "heading")


        self.vbox2.pack_start(self.latitude_label, False, False, 0)
        self.vbox2.pack_start(self.longitude_label, False, False, 0)
        self.vbox2.pack_start(self.altitude_label, False, False, 0)
        self.vbox2.pack_start(self.pitch_label, False, False, 0)
        self.vbox2.pack_start(self.yaw_label, False, False, 0)
        self.vbox2.pack_start(self.roll_label, False, False, 0)
        
        self.vbox3.pack_start(self.velocity_label, False, False, 0)
        self.vbox3.pack_start(self.groundspeed_label, False, False, 0)
        self.vbox3.pack_start(self.airspeed_label, False, False, 0)
        self.vbox3.pack_start(self.battery_voltage_label, False, False, 0)
        self.vbox3.pack_start(self.battery_current_label, False, False, 0)
        self.vbox3.pack_start(self.battery_level_label, False, False, 0)
        
        self.vbox4.pack_start(self.last_heartbeat_label, False, False, 0)
        self.vbox4.pack_start(self.is_armable_label, False, False, 0)
        self.vbox4.pack_start(self.system_status_label, False, False, 0)
        self.vbox4.pack_start(self.mode_label, False, False, 0)
        self.vbox4.pack_start(self.armed_label, False, False, 0)
        self.vbox4.pack_start(self.heading_label, False, False, 0)

        self.hbox.pack_start(self.vbox2, False, False, 0)
        self.hbox.pack_start(self.vbox3, False, False, 0)
        self.hbox.pack_start(self.vbox4, False, False, 0)

        self.vbox1.pack_start(self.hbox, False, False, 0)
        self.vbox1.pack_start(single_drone_command_box(self.nlink, self.logger), False, False, 0)

        self.frame.add(self.vbox1)
        list_row.add(self.frame)
        self.add(list_row)


class Label(Gtk.Label):
    def __init__(self, nlink, attribute_name, attribute_path):
        super(Label, self).__init__(xalign=0)
        self.nlink = nlink
        self.attribute_name = attribute_name
        self.attribute_path = attribute_path

        tmp_value = rgetattr(self.nlink.drone, attribute_path)

        if isinstance(tmp_value, float):
            tmp_value = "{0:0.3f}".format(tmp_value)

        if attribute_name is not "":
            self.set_text(attribute_name+ ": "+ str(tmp_value))
        else:
            self.set_text(str(tmp_value))

        # Initialize Listener

        self.nlink.drone.add_attribute_listener(self.attribute_path, self.attribute_callback)

    def attribute_callback(self, obj, attr_name, value):
        #Called by Drone manager
        GLib.idle_add(self.gui_callback, value)
        # Action put into GUI action puffer
        pass
    def gui_callback(self, value):
        #Called by GUI
        if isinstance(value, float):
            value = "{0:0.3f}".format(value)
            #print "Changed ", self.attribute_name," to ", value


        if self.attribute_name is not "":
            self.set_text(self.attribute_name+": "+str(value))
        else:
            self.set_text(str(value))
        # Returns False so action is only performed once
        return False


class command_box(Gtk.Box):
    def __init__(self, logger):
        super(command_box, self).__init__(spacing=6)
        self.commands_store = None
        self.combo = None
        self.frame_combo = Gtk.Frame()
        self.frame_button = Gtk.Frame()
        self.initialize_combo()
        self.logger = logger

        self.box = Gtk.Box(spacing=3)

        self.param_entry = param_entry_stack([])

        self.send_button = Gtk.Button.new_with_label("Send")
        self.send_button.connect("clicked", self.send)

        '''
        self.frame_combo.add(self.combo)
        self.frame_combo.set_shadow_type(Gtk.ShadowType.NONE)
        self.frame_combo.set_label("")'''

        self.frame_button.add(self.send_button)
        self.frame_button.set_shadow_type(Gtk.ShadowType.NONE)
        self.frame_button.set_label("")

        self.pack_start(self.combo, True, True, 0)
        self.pack_start(self.param_entry, False, False, 0)
        self.pack_start(self.frame_button, False, False, 0)

    def initialize_combo(self):
        self.commands_store = Gtk.ListStore(str)
        self.commands_store.append(["ARM VEHICLE"])
        self.commands_store.append(["SET MODE"])
        self.commands_store.append(["TAKE OFF"])
        self.commands_store.append(["GO TO (Global Relative)"])
        self.commands_store.append(["GO TO (Home Relative)"])
        self.commands_store.append(["GO TO (Current Relative)"])
        self.commands_store.append(["MOVE AT (Velocity/Duration)"])
        self.commands_store.append(["RETURN TO LAUNCH"])
        self.commands_store.append(["LAND"])
        self.combo = Gtk.ComboBox.new_with_model(self.commands_store)
        #self.combo.set_size_request(5,-1)
        renderer_text = Gtk.CellRendererText()
        renderer_text.set_property("align-set", True)
        renderer_text.set_property("alignment", Pango.Alignment.CENTER)
        renderer_text.set_property("xalign", 0.5)
        renderer_text.set_property("wrap-mode",Pango.WrapMode.WORD)
        renderer_text.set_property("wrap-width", 110)
        self.combo.pack_start(renderer_text, True)
        self.combo.add_attribute(renderer_text, "text", 0)
        self.combo.connect("changed",self.set_entries)

    

    def send(self, button):
        tree_iter = self.combo.get_active_iter()

        if tree_iter is not None:
            model = self.combo.get_model()
            command = model[tree_iter][0]
            print "Sending command ", command
    def set_entries(self, combo):
        tree_iter = combo.get_active_iter()

        if tree_iter is not None:
            model = self.combo.get_model()
            command = model[tree_iter][0]
            if command == "ARM VEHICLE":
                self.param_entry.set_entries_by_name([])
            if command == "SET MODE":
                self.param_entry.set_entries_by_name(["Mode",])
            if command == "TAKE OFF":
                self.param_entry.set_entries_by_name(["Altitude"])
            if command == "GO TO (Global Relative)":
                self.param_entry.set_entries_by_name(["Latitude", "Longitude","Altitude"])
            if command == "GO TO (Home Relative)":
                self.param_entry.set_entries_by_name(["North", "East", "Down"])
            if command == "GO TO (Current Relative)":
                self.param_entry.set_entries_by_name(["North","East","Down"])
            if command == "MOVE AT (Velocity/Duration)":
                self.param_entry.set_entries_by_name(["Vx","Vy","Vz","Duration"])
            if command == "RETURN TO LAUNCH":
                self.param_entry.set_entries_by_name([])
            if command == "LAND":
                self.param_entry.set_entries_by_name([])

class param_entry_stack(Gtk.Stack):
    def __init__(self, names = None):
        super(param_entry_stack, self).__init__()

        self.empty_entry = Gtk.Entry()
        self.empty_entry.set_editable(False)
        self.empty_entry.set_can_focus(False)
        self.empty_entry.set_name("grayEntry")
        self.empty_entry_frame = Gtk.Frame()
        self.empty_entry_frame.set_shadow_type(Gtk.ShadowType.NONE)
        self.empty_entry_frame.set_label("")
        self.empty_entry_frame.add(self.empty_entry)
        self.add_named(self.empty_entry_frame,"Stack-0")
        self.num_entries = 0 # This value holds the current number of entries being displayed

        self.entry_frames = [[self.empty_entry_frame,]]

        for index_1 in range(1,5):
            temp_list = []
            temp_box = Gtk.Box()
            for index_2 in range(index_1):
                temp_entry = Gtk.Entry()
                temp_entry.set_width_chars(30/index_1)
                temp_entry.set_text("") # Clear text buffer
                temp_entry.set_editable(True)
                temp_frame = Gtk.Frame()
                temp_frame.add(temp_entry)
                temp_frame.set_shadow_type(Gtk.ShadowType.NONE)
                if index_1 != 1:
                    temp_box.pack_start(temp_frame, False, False, 0)
                temp_list.append(temp_frame)
            box_name = "Stack-"+str(index_1)
            if index_1 == 1:
                self.add_named(temp_frame, box_name) # add entry directly without box
            else:
                self.add_named(temp_box, box_name)
            self.entry_frames.append(temp_list)

        if names is not None:
            self.set_entries_by_name(names)
    def set_entries_by_name(self, names):
        print "names are", names
        self.num_entries = len(names)
        child_name = "Stack-"+str(self.num_entries)
        temp_box = self.get_child_by_name(child_name)
        temp_box.show_now()
        self.set_visible_child(temp_box)
        index = 0
        if len(names) is not 0:
            for frame in self.entry_frames[self.num_entries]:
                name = names[index]
                if name is not None:
                    frame.set_label(name)
                index += 1
    def get_current_params(self):
        params = []
        for frame in self.entry_frames[self.num_entries]:
            entry = frame.get_child()
            params.append(entry.get_text())

        return params

class global_command_box(command_box):
    def __init__(self, nlinks, logger):
        super(global_command_box, self).__init__(logger)
        self.nlinks = nlinks
    def send(self, button):
        tree_iter = self.combo.get_active_iter()
        if tree_iter is not None:
            model = self.combo.get_model()
            command = model[tree_iter][0]
            params = self.param_entry.get_current_params()

        action = droneAction(command, params)
        for nlink in self.nlinks:
            nlink.action_buffer.put(action)

class single_drone_command_box(command_box):
    def __init__(self, nlink, logger):
        super(single_drone_command_box, self).__init__(logger)
        self.nlink = nlink
    def send(self, button):
        tree_iter = self.combo.get_active_iter()
        if tree_iter is not None:
            model = self.combo.get_model()
            command = model[tree_iter][0]
            params = self.param_entry.get_current_params()

            action = droneAction(command, params)
            self.nlink.action_buffer.put(action)



class xbeeManager(threading.Thread):
    def __init__(self, local_xbee, queue, networkLinks, logger):
        super(xbeeManager,self).__init__()
        self.xbee = local_xbee
        self.to_close = False
        self.logger = logger
        self.queue = queue
        self.networkLinks = networkLinks
        print "Xbee Manager started"
    def run(self):
        # Reset all other radios in the network
        #self.xbee.send("at",command="NR", parameter='\x01')
        #time.sleep(2)
        start = time.time()
        while not self.to_close:
            try:
                mes_received = self.xbee.wait_read_frame()
                #self.logger.debug("Received message")
                #if time.time() - start> 2.0:
                    #print "Received message"
                    #start = time.time()

                #self.logger.debug("Received message")
                mes_thread = threading.Thread(target = self.sortMessages, args=(mes_received,))
                mes_thread.start()
                
            #except xbee.TimeoutException:
                #pass
            except Exception as e:
                traceback_stack = traceback.extract_stack()
                traceback_list = traceback.format_list(traceback_stack)
                if not self.to_close:
                    err_str = ""
                    err_str +="Traceback:"
                    for line in traceback_list:
                        err_str+= line
                    err_str += "\nError:"
                    err_str += e.__str__()
                    self.logger.error(err_str)
                    print(e)

    
    def sortMessages(self, mes_received):
        if mes_received['id'] == 'at_response':
            if mes_received['command'] == 'ND':
                source_addr= mes_received['paramter']['source_addr']
                source_addr_long = mes_received['parameter']['source_addr_long']
                found_link = self.findLink(source_addr_long)        
                if found_link is not None:
                    pass
                else: 
                    request = {"request":"new_link", "remote_addr_long":source_addr_long, "remote_addr":source_addr}
                    self.queue.put(request)
                
        elif mes_received['id'] == "rx":
                source_addr = mes_received['source_addr']
                source_addr_long = mes_received['source_addr_long']
                #self.logger.debug("In sort messages source_addr is "+source_addr)
                #self.logger.debug("In sort messages source_addr_long is "+source_addr_long)
                found_link = self.findLink(source_addr_long)
                if found_link is not None:
                    found_link.send_to_control(mes_received['rf_data'])
                else: 
                    request = {"request":"new_link", "remote_addr_long":source_addr_long, "remote_addr":source_addr}
                    self.queue.put(request)
                    found_link = self.findLink(source_addr_long)
                    if found_link is not None:
                        found_link.send_to_control(mes_received['rf_data'])

    def getLinks(self):
        return self.networkLinks
    def findLink(self, remote_addr_long):
        found_link = None
        for link in self.networkLinks:
            if link.hasAddress(remote_addr_long):
                found_link = link
        return found_link

    def close(self):
        print "Closing Xbee Manager..."
        self.to_close = True

            
class networkLink(threading.Thread):
    def __init__(self, local_xbee, remote_addr_long, lock, logger, droneNum, queue, gui_manager, remote_addr="FFFE", active_dead_ports=set(), network_address="127.0.0.1"):
        super(networkLink, self).__init__()
        self.network_address = network_address
        self.remote_addr_long = remote_addr_long
        self.remote_addr = remote_addr
        self.local_xbee = local_xbee
        self.droneNum = droneNum
        self.gui_manager = gui_manager
        self.logger = logger.getLogger("Drone-"+str(droneNum))
        self.listening = False
        self.port = None
        self.lock = lock
        self.active = True
        self.queue = queue
        self.name = "Drone-"+str(droneNum)
        self.action_buffer = Queue() 

        if self.remote_addr_long == None:
            raise self.ArgumentError("Long Address must be set") 

        self.send_thread = threading.Thread(target=self.send_to_drone)
        self.to_close = False
        self.active_dead_ports = active_dead_ports
        self.connection_initialized = False
        self.connection_established = False
        # Have to be performed in separate threads because they both start blocking operations
        # until they communicate with each other
        self._init_drone_thread = threading.Thread(target=self._init_drone)
        self._init_socket_thread = threading.Thread(target=self._init_socket)
        self.actions_thread = threading.Thread(target=self.manage_actions)
        self._init_drone_thread.setDaemon(True)
        self._init_socket_thread.start()
        self._init_drone_thread.start()
        self.send_thread.start()
        self.actions_thread.start()
        '''
        self.logger.debug("In init link self.remote_addr is:"+self.remote_addr)
        self.logger.debug("In init link self.remote_addr_long is:"+self.remote_addr_long)'''
    
    def _init_drone(self):
        # thread that gets stuck
        request = dict()
        while not self.listening and not self.to_close: # Waiting for server to be ready
            time.sleep(0.01)
        try:
            ip = "tcp:"+self.network_address+":"+str(self.port)
            self.logger.info("Connecting with Vehicle...")
            self.drone = dronekit.connect(ip, wait_ready=False, rate=1, status_printer=self.printStationMessages, heartbeat_timeout=90)
            '''
            self.cmds = self.drone.commands
            self.cmds.download()
            self.logger.info("Downloading parameters from Vehicle...")
            self.cmds.wait_ready(timeout=300)'''
            self.connection_established = True
            GLib.idle_add(self.gui_manager.add_nlink_box, self)
            self.logger.info("Connection with Drone Established")
            self._connection_check_thread = threading.Thread(target=self._check_connection)
            self._connection_check_thread.start()

        except socket.error as e:
            self.logger.error("Connection failed")
            traceback.print_exc(file=sys.stderr)
            self.logger.error("Server Error:{}".format(e))
            # Build Request
            request['request'] = "remove_drone"
            request['remote_addr_long'] = self.remote_addr_long
            self.queue.put(request)
            self.close()
        except dronekit.APIException as e:
            self.logger.error("Could not Establish connection with drone. Removing drone" + str(self.droneNum)+" from list")
            traceback.print_exc(file=sys.stderr)
            self.logger.error("Error: {}".format(e))
            request = dict()
            request['request'] = "remove_drone"
            request['remote_addr_long'] = self.remote_addr_long
            self.queue.put(request)
            self.close()
        
    def _init_socket(self):
        for port in range(1000,10000):
            if port not in self.active_dead_ports:
                try:
                    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.socket.settimeout(5.0)
                    self.socket.bind((self.network_address, port))
                    self.port = port
                    self.socket.listen(0)
                    self.listening = True
                    break

                except socket.error as e:
                    self.active_dead_ports.add(port)
                    print 
                    tries += 1
                    if tries > 10:
                        raise Exception("Could not create network link")
                        self.to_close = True
                        break
                    if self.to_close:
                        break
        if self.port is not None:
            self.active_dead_ports.add(self.port)
        self.conn_socket, address = self.socket.accept()
        self.logger.debug("Connection from address "+str(address)+" accepted")
        self.connection_initialized = True
        '''
        while not self.to_close:
            self.conn_socket, address = self.socket.accept()'''


    def run(self):
        pass

    def send_to_drone(self):
        data = ""
        start = time.time()
        while not self.to_close:
            if self.connection_initialized:
                try:
                    if time.time() - start > 2.0:
                        #self.logger.debug("Receiving data from groundstation")
                        start = time.time()
                    data = self.conn_socket.recv(4096)
                except socket.error as e:
                    if not self.to_close:
                        if e.errno == errno.WSAECONNRESET:
                            if self.connection_initialized:
                                self.logger.error("Connection with Drone was lost")
                                # Connection closed by drone
                                # Close inactive links
                                self.connection_initialized = False
                                self.conn_socket.close()
                                if hasattr(self, "drone"):
                                    if self.drone is not None:
                                        self.drone._handler.master.close()
                                        self.drone.close()
                                # Reconnect to Drone
                                self.logger.info("Reconnecting with Drone")
                                self._init_drone_thread = threading.Thread(target=self._init_drone)
                                self._init_socket_thread = threading.Thread(target=self._init_socket)
                                self._init_drone_thread.setDaemon(True)
                                self._init_socket_thread.start()
                                self._init_drone_thread.start()

                        '''
                        print >> sys.stderr, "Traceback:"
                        traceback.print_exc(file=sys.stderr)
                        print >> sys.stderr, "Error:"
                        print >> sys.stderr, e
                        print >> sys.stderr, "String name of error"
                        print >> sys.stderr, errno.errorcode[e.errno]'''
                try:
                    with self.lock:
                        if data != "":
                            #self.logger.debug("Sending xbee message")
                            if not self.to_close:
                                self.local_xbee.send('tx', dest_addr=self.remote_addr, dest_addr_long=self.remote_addr_long, data=data)
                except serial.SerialException as e:
                    if not self.to_close:
                        print >> sys.stderr, "Traceback:"
                        traceback.print_exc(file=sys.stderr)
                        print >> sys.stderr, "Error:"
                        print >> sys.stderr, e

    def send_to_control(self, message):
        if self.connection_initialized and not self.to_close:
            try:
                self.conn_socket.send(message)
                #self.logger.debug("sending Data to groundstation")
            except socket.error as e:
                if not self.to_close:
                    if e.errno == errno.WSAECONNRESET:
                        if self.connection_initialized:
                            self.logger.error("Connection with Drone was lost")
                            # Connection closed by drone

                            # Close inactive links
                            self.connection_initialized = False
                            self.conn_socket.close()
                            if hasattr(self, "drone"):
                                if self.drone is not None:
                                    self.drone.close()
                            # Reconnect to Drone
                            self.logger.info("Reconnecting with Drone...")
                            self._init_drone_thread = threading.Thread(target=self._init_drone)
                            self._init_socket_thread = threading.Thread(target=self._init_socket)
                            self._init_drone_thread.setDaemon(True)
                            self._init_socket_thread.start()
                            self._init_drone_thread.start()
    def close(self):
        print "Closing Network Link..."
        self.to_close = True
        self.socket.close()
        self.conn_socket.close()
        self.active = False
        GLib.idle_add(self.gui_manager.remove_nlink_box, self.name)
        if hasattr(self, "drone"):
            if self.drone is not None:
                self.drone.close()
    def setAddrLong(self, remote_addr):
        self.remote_addr = remote_addr
    def setAddr(self, remote_addr):
        self.remote_addr = remote_addr
    def hasAddress(self, address):
        '''
        self.logger.debug("The type of address pass is "+str(type(address)))
        self.logger.debug("And its value is: "+str(address))
        self.logger.debug("The type of the local long address is "+str(type(self.remote_addr_long)))
        self.logger.debug("And its value is: "+str(self.remote_addr_long))
        self.logger.debug("The type of local short address is "+str(type(self.remote_addr)))
        self.logger.debug("And its value is: "+self.remote_addr)'''

        if self.remote_addr == address or self.remote_addr_long == address:
            #self.logger.debug("Returning True")
            return True
        else:
            #self.logger.debug("Returning False")
            return False
    def printStationMessages(self, text):
        self.logger.info(text)
    class ArgumentError(Exception):
        def __init__(self, mess):
            self.message(mess)
        def __str__(self):
            return self.message
    def hasName(self, name):
        if self.name==name:
            return True
        else:
            return False
    def perform_action(self, action, params):
        if action == "ARM VEHICLE":
            if not self.drone.mode.name == "GUIDED":
                self.logger.info("Changing vehicle mode to GUIDED")
                self.drone.mode = dronekit.VehicleMode("GUIDED")
                counter = 0
                while not self.drone.mode.name == 'GUIDED' and counter < 5:
                    time.sleep(1)
                    counter+=1
                if not self.drone.mode.name == 'GUIDED':
                    self.logger.error("Could not change mode to GUIDED")
                    return
                
            if not self.drone.is_armable:
                self.logger.error("Vehicle is not armable. Please try again later")
                return
            else:
                self.drone.armed = True
                counter = 0
                self.logger.info("Arming...")

        if action == "SET MODE":
            counter = 0
            mode = params[0]
            self.logger.info("Changing mode to"+str(mode))
            if self.drone.mode.name is not mode:
                self.mode = dronekit.VehicleMode(mode)
            counter += 0
            while not self.drone.mode.name == mode and counter < 10 and not self.to_close:
                time.sleep(1)
                counter += 1
            if self.drone.mode.name is not mode:
                self.logger.info("Failed to change mode to "+mode)

        if action == "TAKE OFF":
            target_altitude = 0.0
            if not self.drone.mode.name == "GUIDED":
                self.logger.error("Vehicle must be in GUIDED mode to take off")
            else:
                try:
                    target_altitude = float(params[0])
                except ValueError:
                    self.logger.error("Altitude parameter must be an int or a float")
                    return
                self.drone.simple_takeoff(target_altitude)
                self.logger.info("Vehicle is taking off to an altitude of "+str(target_altitude))
            start = time.time()
            last_alt = self.drone.location.global_relative_frame.alt
            while not self.to_close:
                current_alt = self.drone.location.global_relative_frame.alt
                if current_alt > last_alt:
                    last_alt = current_alt
                    start = time.time()
                if last_alt>= target_altitude*0.95:
                    self.logger.info("Vehicle reached target altitude")
                    break
                if time.time() - start > 10:
                    self.logger.error("Drone could not reach target altitude")
                    break

        if action == "GO TO (Global Relative)":
            if not self.drone.mode.name == "GUIDED":
                self.logger.error("Vehicle must be in GUIDED mode to perfom Go To action")
            else:
                lat = params[0]
                lon = params[1]
                alt = params[2]
                if lat == "":
                    lat = self.drone.global_relative_frame.lat
                else:
                    try:
                        lat = float(lat)
                    except ValueError:
                        self.logger.error("Latitude, Longitude, and Altitude must be ints or floats")
                        return
                if lon == "":
                    lon = self.drone.global_relative_frame.lon
                else:
                    try:
                        lon = float(lon)
                    except ValueError:
                        self.logger.error("Latitude, Longitude, and Altitude must be ints or floats")
                        return
                if alt == "":
                    lat = self.drone.global_relative_frame.lat
                else:
                    try:
                        alt = float(alt)
                    except ValueError:
                        self.logger.error("Latitude, Longitude, and Altitude must be ints or floats")
                        return
                    
                dest_location = dronekit.LocationGlobalRelative(-34.364114, 149.166022, 30)
                self.drone.simple_goto(dest_location)
                self.logger.info("Sending GO TO command...")

        if action == "GO TO (Home Relative)":
            north = params[0]
            east = params[1]
            down = params[2]

            if north == "":
                north = self.drone.location.local_frame.north
            else:
                try:
                    north = float(north)
                except ValueError:
                    self.logger.error("North, East, Down must be ints or floats")
                    return
            if east == "":
                east = self.drone.local_frame.east
            else:
                try:
                    east = float(east)
                except ValueError:
                    self.logger.error("North, East, Down must be ints or floats")
                    return
            if down == "":
                down = self.drone.location.local_frame.down
            else:
                try:
                    down = float(down)
                except ValueError:
                    self.logger.error("North, East, Down must be ints or floats")
                    return

            self.logger.info("Sending GO TO command...")
            dest_location = dronekit.LocationLocal(north, east, down)
            self.drone.simple_goto(dest_location)

        if action == "GO TO (Current Relative)":
            north = params[0]
            east = params[1]
            down = params[2]

            if north == "":
                north = self.drone.local_frame.north
            else:
                try:
                    north = float(north)
                except ValueError:
                    self.logger.error("North, East, Down must be ints or floats")
                    return
            if east == "":
                east = self.drone.local_frame.east
            else:
                try:
                    east = float(east)
                except ValueError:
                    self.logger.error("North, East, Down must be ints or floats")
                    return
            if down == "":
                down = self.drone.local_frame.down
            else:
                try:
                    down = float(down)
                except ValueError:
                    self.logger.error("North, East, Down must be ints or floats")
                    return

            self.logger.info("Sending GO TO command...")
            dest_location = self.get_location_metres(self.drone.location.global_relative_frame, north, east, down)
            self.drone.simple_goto(dest_location)


        if action == "MOVE AT (Velocity/Duration)":
            velocity_x = params[0]
            velocity_y = params[1]
            velocity_z = params[2]
            duration = params[3]

            try:
                velocity_x = float(velocity_x)
                velocity_y = float(velocity_y)
                velocity_z = float(velocity_z)
                duration = int(duration)
            except ValueError:
                self.logger.error("Velocities must be floats. Duration must be an int.")
                return

            # Function from http://python.dronekit.io/guide/copter/guided_mode.html
            msg = self.drone.message_factory.set_position_target_local_ned_encode(
                0,       # time_boot_ms (not used)
                0, 0,    # target system, target component
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
                0b0000111111000111, # type_mask (only speeds enabled)
                0, 0, 0, # x, y, z positions (not used)
                velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
                0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


            # send command to vehicle on 1 Hz cycle
            for x in range(0,duration):
                self.drone.send_mavlink(msg)
                time.sleep(1)
                self.logger.info("Moving vehicle...")

        if action == "RETURN TO LAUNCH":
            self.drone.mode = dronekit.VehicleMode("RETURN TO LAUNCH")

            if self.drone.mode.name is not "RETURN TO LAUNCH":
                self.mode = dronekit.VehicleMode(mode)
            counter += 0
            while not self.drone.mode.name == mode and counter < 5 and not self.to_close:
                time.sleep(1)
                counter += 1
            if self.drone.mode.name is not "RETURN TO LAUNCH":
                self.logger.info("Failed to set mode to RETURN TO LAUNCH")

        if action == "LAND":
            self.drone.mode = dronekit.VehicleMode("LAND")

            if self.drone.mode.name is not "LAND":
                self.mode = dronekit.VehicleMode(mode)
            counter += 0
            while not self.drone.mode.name == mode and counter < 5 and not self.to_close:
                time.sleep(1)
                counter += 1
            if self.drone.mode.name is not "LAND":
                self.logger.info("Failed to set mode to LAND")
    def manage_actions(self):
        while not self.to_close:
            if not self.action_buffer.empty():
                current_action = self.action_buffer.get()
                command = current_action.command
                params = current_action.params
                self.perform_action(command, params)
            else:
                time.sleep(1)
    def _check_connection(self):
        self.logger.debug("Connection Check Initialized")
        while not self.connection_established:
            time.sleep(5)

        while not self.to_close:
            time.sleep(5)
            self.logger.debug("Checking connection")
            if hasattr(self, "drone"):
                if not self.drone._handler._alive:
                    self.connection_established = False
                    self.logger.error("Connection with Drone was lost(2)")
                    # Connection closed by drone

                    # Close inactive links
                    self.connection_initialized = False
                    self.conn_socket.close()
                    self.drone._handler.master.close()
                    self.drone.close()
                    # Reconnect to Drone
                    self.logger.info("Reconnecting with Drone...")
                    self._init_drone_thread = threading.Thread(target=self._init_drone)
                    self._init_socket_thread = threading.Thread(target=self._init_socket)
                    self._init_drone_thread.setDaemon(True)
                    self._init_socket_thread.start()
                    self._init_drone_thread.start()



    # The function below was modified from http://python.dronekit.io/guide/copter/guided_mode.html
    def get_location_metres(self, original_location, dNorth, dEast, dalt):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned LocationGlobal has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.

        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius=6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        newalt = original_location.alt + dalt
        if type(original_location) is dronekit.LocationGlobal:
            targetlocation=dronekit.LocationGlobal(newlat, newlon, newalt)
        elif type(original_location) is dronekit.LocationGlobalRelative:
            targetlocation=dronekit.LocationGlobalRelative(newlat, newlon, newalt)
        else:
            raise Exception("Invalid Location object passed")

        return targetlocation

        
class NetworkManager():
    def __init__(self, device, logger, gui_manager, nlinks,baud=57600, network_address="127.0.0.1"):
        self.baud = baud
        self.logger = logger
        self.gui_manager = gui_manager
        self.requestsQueue = Queue()
        self.device = device
        self.to_close = False
        self.networkLinks = nlinks
        try:
            self.serial = serial.Serial(device, baud, write_timeout=10, timeout=5, xonxoff=False, rtscts=False)
        except serial.SerialException:
            error_str =("Could not open serial device\n")
            error_str +=("\tDevices Available:\n")
            if sys.platform == "win32":
                for device in serial.tools.list_ports.comports():
                    error_str += ("\t"+str(device)+"\n")
            self.logger.error(error_str)
            self.close()
            #sys.exit(1)
        if not self.to_close:
            self.xbee = xbee.ZigBee(self.serial)
            self.xbeeManagerInstance = xbeeManager(self.xbee, logger = self.logger, queue=self.requestsQueue, networkLinks=self.networkLinks)
            self.request_thread = threading.Thread(target=self.process_requests)
            self.cleaning_thread = threading.Thread(target=self.clean_up)
            self.active_dead_ports = set()
            self.network_address = network_address
            self.xbee_addresses = set()
            self.lock = threading.Lock()
            self.droneNum = 0 # Holds current active number of drones
            print "Network Manager started"
    def start(self):
        if not self.to_close:
            self.xbeeManagerInstance.start()
            self.request_thread.start()
            self.cleaning_thread.start()
        else:
            self.logger.error("Could not run Network Manager")
    def process_requests(self):
         while not self.to_close:
            try:
                request = self.requestsQueue.get(timeout=1)
                self.logger.debug("Got new request:{}".format(request))
                if request['request'] == "new_link":
                    remote_addr_long = request['remote_addr_long']
                    if remote_addr_long not in self.xbee_addresses:
                        self.logger.info("New drone detected...")
                        self.xbee_addresses.add(remote_addr_long)
                        remote_addr = request['remote_addr']
                        self.droneNum += 1
                        temp_nlink = networkLink(active_dead_ports=self.active_dead_ports, logger=self.logger, network_address=self.network_address, 
                            queue=self.requestsQueue, droneNum = self.droneNum, local_xbee=self.xbee, remote_addr_long=remote_addr_long, 
                            remote_addr=remote_addr, lock=self.lock, gui_manager=self.gui_manager)
                        temp_nlink.start()
                        self.networkLinks.append(temp_nlink)
                        #self.logger.debug("Vehicle with address "+str(remote_addr)+" added.")
                elif request['request'] == 'remove_drone' :
                    remote_addr_long = request['remote_addr_long']
                    self.xbee_addresses.discard(remote_addr_long)
                    link = self.findLink(remote_addr_long)
                    if link is not None:
                        #self.logger.debug("Removing vehicle with address "+str(link.remote_addr_long))
                        link.close()
                        self.networkLinks.remove(link)
                    else:
                        pass
                        #self.logger.debug("Could not find link with address "+str(remote_addr_long))
                    #self.logger.debug("Vehicle with address "+str(remote_addr_long)+" has been removed.")

            except Empty:
                pass
    def clean_up(self):
        while not self.to_close:
            time.sleep(1.0)
            for link in self.networkLinks:
                if not link.active:
                    self.networkLinks.remove(link)


    def num_vehicles(self):
        return len(networkLinks)
    def close(self, *args, **kwargs):

        print "Closing Network Manager..."
        self.to_close = True
        try:
            for link in self.networkLinks:
                try:
                    link.close()
                except Exception as e:
                    self.logger.debug(e)
        except Exception as e:
            self.logger.debug(e)
        try:
            self.xbeeManagerInstance.close()
        except Exception as e:
            self.logger.debug(e)
        try:
            self.serial.close()
        except Exception as e:
            self.logger.debug(e)
    def findLink(self, remote_addr_long):
        self.logger.debug("findLink called")
        self.logger.debug("list of links is:")
        self.logger.debug(str(self.networkLinks))
        found_link = None
        for link in self.networkLinks:
            if link.hasAddress(remote_addr_long):
                self.logger.debug("Link has been found")
                found_link = link
        return found_link

if __name__ == "__main__":
    LOGLEVEL = logging.INFO

    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--device", metavar="DEVICE", dest="device", default="/dev/ttyUSB0", help="Serial device to receive communication from (default=/dev/ttyUSB0)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose mode")
    parser.add_argument("-l", "--logfile", help="Logfile to store logging messages (default=stdout)")

    args = parser.parse_args()

    if args.verbose:
        LOGLEVEL = logging.DEBUG

    logging.basicConfig(
        format='[%(asctime)s][%(levelname)s][%(threadName)s][%(name)s]:%(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
        level=LOGLEVEL
        )

    log_formatter = logging.Formatter(
        fmt  = '[%(asctime)s][%(levelname)s][%(threadName)s][%(name)s]:%(message)s',
        datefmt = '%Y-%m-%d %H:%M:%S'
    )

    logger = logging.getLogger()


    '''
    @wrap(dronekit.util.errprinter)
    def add_err_to_logger(forig, self, text):
        logger.error(text)

    import dronekit.__init__
    '''
    #Attempt to create monkey patch to have library error printer function print to my logger
    

    nlinks = list()

    provider = Gtk.CssProvider()
    provider.load_from_data('#grayEntry { background: gray; }')
    Gtk.StyleContext.add_provider_for_screen(Gdk.Screen.get_default(), provider,
    Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)

    try:
        gManager = GuiManager(nlinks, logger)
        gManager.connect("delete-event", Gtk.main_quit)
        gManager.show_all()
        log_stream = gManager.get_log_buffer()
        window_logger_ch = logging.StreamHandler(log_stream)
        window_logger_ch.setFormatter(log_formatter)
        window_logger_ch.setLevel(LOGLEVEL)
        logger.addHandler(window_logger_ch)
        nm = NetworkManager(args.device, logging, gManager, nlinks)
        gManager.connect("delete-event", nm.close)
        nm.start()

        Gtk.main()
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        #traceback.print_exc(file=sys.stderr)
        Gtk.main_quit()
        nm.close()
