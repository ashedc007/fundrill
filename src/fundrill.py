# Fundrill PCB utility Copyright(c) 2026  Tinashe Gwena 

import sys
import math
import serial
import time
import cv2 as cv
import numpy as np
import threading

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, Gdk, GLib, GdkPixbuf

import math
import cairo
class CNCcam:
    arrayofclosest = []
    drillcoords = []
    temp_drillcoords = []
    correctcoords = []
    num_coords = 0
    drill_size_definitions = []
    largest_drill_size = 0
    
    framecount = 0
    real_closest_exists = False
    real_closest_circle = (320, 240)
    real_closest_radius = 20
    
    real_closest_distance = 20
    current_coordinate_index = 0
    current_coordinate_X = 0
    current_coordinate_Y = 0
    current_drill_diameter = 0.5
    cnc_step_size  = 1
    
    auto_finding = False
    next_find_ready = False
    auto_try_count = 0
    auto_try_firsttime = True
    
    furthest_allowed_closest = 65
    clb_mmpix = 91
    
    cancnc = True
    
    check_complete = False
    
    timestart = 0
    
    displayHander = 0
    
    min_coordsX = 0
    min_coordsY = 0
    max_coordsX = 0
    max_coordsY = 0
    
    adj_min_coordsX = 0
    adj_min_coordsY = 0
    
    scale_coordsX = 0
    scale_coordsY = 0
    scale_coordsXY = 0
    
    
    margin_coordsX = 0
    margin_coordsY = 0
    minimum_margin = 10
    
    circles = None
    
    camera_busy = False
    ret = 0
    src = 0
    
    reverse_X_axis = False
    def __init__(self,argv):
        default_file = 'outdrill_correct.drl'
        
        self.filename = sys.argv[1] if len(sys.argv) > 1 else default_file
        print("file is : " + self.filename) 
        
        for number in range(1,11):
            #print(number)
            self.arrayofclosest.append(dict(
            x=320,
            y=240,
            radius=5,
            active=False,
            occurs=0
            ))
        
        
        if self.cancnc:
            # Open grbl serial port
            self.cnc = serial.Serial('/dev/ttyUSB0',115200)
            
            # Wake up grbl
            newlines = "\r\n\r\n"
            self.cnc.write(newlines.encode())
            time.sleep(2)   # Wait for grbl to initialize
            self.cnc.flushInput()  # Flush startup text in serial input
            
            newInit = "$X\n"
            self.cnc.write(newInit.encode())
            grbl_out = self.cnc.readline() # Wait for grbl response with carriage return
            #print (' : ' + grbl_out.strip())
            print(grbl_out)
            self.cnc.flushInput()
            
            self.cnc_set_home()
            
        
        #camera = cv.VideoCapture(0)
        self.camera = cv.VideoCapture(0)   
        print("hello")
        
    
    def start_timer(self):
        self.timestart = time.perf_counter()
        
    
    def time_elapsed(self):
        return time.perf_counter() - self.timestart
    
    def cnc_set_home(self):
        if self.cancnc:
            cnczero = "G10 P0 L20 X0 Y0 Z0" + "\n"
            
            self.cnc.write(cnczero.encode()) # Send g-code block to grbl
            grbl_out = self.cnc.readline() # Wait for grbl response with carriage return
            
            print(grbl_out)
            
            self.current_coordinate_X = 0
            self.current_coordinate_Y = 0
        
    
    def cnc_goto(self, xpos, ypos):
        self.circles = None
        self.real_closest_exists = False
        
        command_string = ""
        command_string += "G00 " + "X" + ("%0.4f" % (xpos )) + "Y" + ("%0.4f" % (ypos)) + "\n"
        print(command_string)
        
        if self.cancnc:
            self.cnc.write(command_string.encode()) # Send g-code block to grbl
            grbl_out = self.cnc.readline() # Wait for grbl response with carriage return
            #print (' : ' + grbl_out.strip())
            print(grbl_out)
        
        
        self.current_coordinate_X = xpos
        self.current_coordinate_Y = ypos
    
    def cnc_get_status(self):
        phys_coord_start = 0
        
        command_string = ""
        command_string += "?"  + "\n"
        #print(command_string)
        
        success = False
        if self.cancnc:
            num_retries = 0
            #    success = False
            while num_retries < 10 and not success:
                self.cnc.write(command_string.encode()) # Send g-code block to grbl
                grbl_out = self.cnc.readline() # Wait for grbl response with carriage return
                #print (' : ' + grbl_out.strip())
                ##print(grbl_out)
                
                if len(grbl_out) > 10:
                    #grbl_out_bytes = grbl_out
                    phys_coord_start = grbl_out.find(b'Idle') 
                    
                    if phys_coord_start != -1:
                        print("CNC Idle")
                        success = True
                        #num_retries  += 1
                    
                
                if not success:
                    num_retries  += 1
                    print("CNC status retries " + str(num_retries))
                    time.sleep(0.5)
                
            
        
        return success
    
    def cnc_get_position(self):
        phys_coord_start = 0
        
        command_string = ""
        command_string += "?"  + "\n"
        print(command_string)
        
        if self.cancnc:
            num_retries = 0
            success = False
            while num_retries < 5 and not success:
                self.cnc.write(command_string.encode()) # Send g-code block to grbl
                grbl_out = self.cnc.readline() # Wait for grbl response with carriage return
                #print (' : ' + grbl_out.strip())
                print(grbl_out)
                
                #grbl_out_bytes = grbl_out
                phys_coord_start = grbl_out.find(b'MPos') 
                first_comma = grbl_out.find(b',', phys_coord_start)
                second_comma = grbl_out.find(b',', first_comma + 1)
                if phys_coord_start != -1:
                    first_comma = grbl_out.find(b',', phys_coord_start + 5)
                    second_comma = grbl_out.find(b',', first_comma + 1)
                    print(grbl_out[(phys_coord_start + 5):first_comma]) 
                    print(grbl_out[(first_comma + 1):second_comma])
                    success = True
                    #num_retries  += 1
                else:
                    num_retries  += 1
                    print("retries " + str(num_retries))
                    time.sleep(0.5)
                
            
        
    
    def check_closest(self, circle_diameter=0.5):
        self.check_complete = False
        self.framecount = 0
        closest_occurrence_thresh = False
        some_hope = False
        num_retries = 0
        default_file = "test.txt"
        self.start_timer()
        while self.camera.isOpened()  and not self.check_complete:
            # Loads an image
            #src = cv.imread(cv.samples.findFile(filename), cv.IMREAD_COLOR)
            num_img_tries = 0
            img_success = False
            
            
            while not img_success and num_img_tries < 5:
                self.camera_busy = True
                ret, src = self.camera.read()
                self.ret = ret
                self.src = src
                self.camera_busy = False
                # Check if image is loaded fine
                if src is None:
                    print ('Error opening image!')
                    print ('Usage hough_circle.py [image_name -- default ' + default_file + '] \n')
                    num_img_tries += 1
                    #return -1
                else:
                    img_success = True
                
                
            
            
            if not img_success:
                return -1
            
            
            gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
            
            gray = cv.medianBlur(gray, 5)
            
            rows = gray.shape[0]
            
            minimum_radius = int((circle_diameter/2)*self.clb_mmpix * 0.7)
            maximum_radius = int((circle_diameter/2)*self.clb_mmpix * 1.3)
            
            #print("circle_parms " + str(circle_diameter) + " " + str(minimum_radius) + " " + str(maximum_radius))
            
            self.circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 8,
            param1=100, param2=22,
            minRadius=minimum_radius, maxRadius=maximum_radius)
            if self.circles is not None:
                self.circles = np.uint16(np.around(self.circles))
                nearest_distance = 400
                closest_found = False
                for i in self.circles[0,:]:
                    
                    #  distance = math.sqrt((320 - i[0])**2 + (240 - i[1])**2)
                    distance = self.calculate_distance(320, 240, i[0], i[1])
                    
                    if distance < nearest_distance and distance < self.furthest_allowed_closest:
                        closest_circle = (i[0], i[1])
                        closest_radius = i[2]
                        closest_found = True
                        nearest_distance = distance
                        #print("closest found " + str(nearest_distance) + " " + str(i[0]) + "," + str(i[1]))
                        some_hope = True
                        self.start_timer()
                    
                    
                
                if closest_found:
                    #print(frame_count)
                    #cv.circle(src, closest_circle, closest_radius, (0, 200, 0), 3)
                    self.framecount += 1
                    
                    if self.framecount < 45 and not closest_occurrence_thresh:
                        closest_match_found = False
                        for a_closest in self.arrayofclosest:
                            #for i in range(0,5) {
                            #print(a_closest)
                            if a_closest['active'] and not closest_match_found:
                                #this_distance = math.sqrt((closest_circle[0] - a_closest['x'])**2 + (closest_circle[1] - a_closest['y'])**2)
                                this_distance = self.calculate_distance(a_closest['x'], a_closest['y'], closest_circle[0], closest_circle[1])
                                if this_distance < 5:
                                    a_closest['occurs'] += 1
                                    #a_closest['active'] = True
                                    if a_closest['occurs'] > 5:
                                        closest_occurrence_thresh = True
                                    
                                    closest_match_found = True
                                    #print("close match found "  + " " + str(a_closest['occurs']))
                                
                            
                        
                        
                        if not closest_match_found:
                            for a_closest in self.arrayofclosest:
                                #for i in range(0,5) {
                                if not a_closest['active']:
                                    a_closest['x'] = closest_circle[0]
                                    a_closest['y'] = closest_circle[1]
                                    a_closest['radius'] = closest_radius
                                    a_closest['occurs'] = 1
                                    a_closest['active'] = True
                                    break
                                
                            
                        
                    
                    else:
                        #closest_circle = (i[0], i[1])
                        #closest_radius = i[2]
                        self.real_closest_exists = False
                        most_occurs = 0
                        last_distance = 250
                        
                        for a_closest in self.arrayofclosest:
                            #print(a_closest)
                            if a_closest['active']:
                                #this_distance = math.sqrt((320 - a_closest['x'])**2 + (240 - a_closest['y'])**2)
                                this_distance = self.calculate_distance(320, 240, a_closest['x'], a_closest['y'])
                                if a_closest['occurs'] > 5 and this_distance < last_distance:
                                    self.real_closest_circle = (a_closest['x'], a_closest['y'])
                                    self.real_closest_radius = a_closest['radius']
                                    self.real_closest_exists = True
                                    last_distance = this_distance
                                    self.real_closest_distance = this_distance
                                    next_find_ready = True
                                    print("real closest exists")
                                
                            
                            a_closest['occurs'] = 0
                            a_closest['active'] = False
                            
                        
                        
                        if not self.real_closest_exists and some_hope and num_retries < 5:
                            print("some hope retry")
                            self.framecount = 0
                            closest_occurrence_thresh = False
                            some_hope = False
                            num_retries += 1
                            
                        else:
                            self.check_complete = True
                        
                    
                    
                
            
            if(self.time_elapsed() > 3):
                self.check_complete = True
                print("time out")
            
        
    
    def calculate_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def camera_display(self):
        default_file = "test.txt"
        cam_image_valid = False
        if self.camera.isOpened():
            # Loads an image
            #src = cv.imread(cv.samples.findFile(filename), cv.IMREAD_COLOR)
            ret, src = self.camera.read()
            # Check if image is loaded fine
            if src is None:
                print ('Error opening image 1!')
                print ('Usage hough_circle.py [image_name -- default ' + default_file + '] \n')
                return -1
            
            
            
            src = cv.cvtColor(src, cv.COLOR_BGR2RGB)
            
            cv.line(src, (320, 220), (320, 260), (0,200,0), 1)
            cv.line(src, (300, 240), (340, 240), (0,200,0), 1)
            cv.circle(src, (320, 240), self.furthest_allowed_closest, (200, 0, 0), 1)
            
            
            pb = GdkPixbuf.Pixbuf.new_from_data(src.tostring(),
            GdkPixbuf.Colorspace.RGB,
            False,
            8,
            src.shape[1],
            src.shape[0],
            src.shape[2]*src.shape[1])
            self.displayHandler.camimage.set_from_pixbuf(pb.copy())
        
        
        self.displayHandler.camimage.show()
    
    def load_drills(self, filename):
        with open(filename, 'r') as reader:
            # Read & print the entire file
            #print(reader.read())
            begin_reading_coords = True
            current_drill_id = 0
            self.largest_drill_size = 0
            line = reader.readline()
            while line != '':  # The EOF char is an empty string
                if line[0] == 'X':
                    ycoord = line.find('Y')
                    if ycoord != -1:
                        #print(line, end='')
                        temp_Xcoord = float(line[1:ycoord])
                        if self.reverse_X_axis:
                            temp_Xcoord = 120 - temp_Xcoord
                        
                        temp_Ycoord = float(line[(ycoord+1):])
                        #print("x",line[1:ycoord],"y",line[(ycoord+1):])
                        self.temp_drillcoords.append([temp_Xcoord, temp_Ycoord, False, current_drill_id])
                        if begin_reading_coords:
                            self.min_coordsX = temp_Xcoord
                            self.min_coordsY = temp_Ycoord
                            self.max_coordsX = temp_Xcoord
                            self.max_coordsY = temp_Ycoord
                            begin_reading_coords = False
                        else:
                            self.min_coordsX = temp_Xcoord if temp_Xcoord < self.min_coordsX else self.min_coordsX
                            self.min_coordsY = temp_Ycoord if temp_Ycoord < self.min_coordsY else self.min_coordsY
                            
                            self.max_coordsX = temp_Xcoord if temp_Xcoord > self.max_coordsX else self.max_coordsX
                            self.max_coordsY = temp_Ycoord if temp_Ycoord > self.max_coordsY else self.max_coordsY
                            
                            
                        
                    
                elif line[0] == 'T':
                    drill_size_marker = line.find('C')
                    
                    if drill_size_marker != -1:
                        new_drill_size_id = int(line[1:drill_size_marker])
                        new_drill_diameter = float(line[(drill_size_marker+1):])
                        self.drill_size_definitions.append(dict( id=new_drill_size_id,
                        size=new_drill_diameter
                        ))
                        self.largest_drill_size = new_drill_diameter if new_drill_diameter > self.largest_drill_size else self.largest_drill_size
                        print("new drill " + str(new_drill_diameter) + " " + str(new_drill_diameter))
                        
                    else:
                        #find current size and set mode to that
                        drill_to_search_id = int(line[1:])
                        current_drill_id = next((ii for ii, xx in enumerate(self.drill_size_definitions) if xx["id"] == drill_to_search_id), None)
                        if current_drill_id is None:
                            current_drill_id = 0
                        
                        print("drill id " + str(current_drill_id))
                    
                    
                
                line = reader.readline()
            
        
        
        print("minx: " + str(self.min_coordsX) + " maxx: " + str(self.max_coordsX))
        
        
        self.num_coords = len(self.temp_drillcoords)
        print("number of coords", self.num_coords)
        
        closest_drill = 0
        current_drill = 0
        current_drill_X = 0
        current_drill_Y = 0
        
        temp_drill_offsetX = 0
        temp_drill_offsetY = 0
        
        real_min_coordsX = 0
        real_min_coordsY = 0
        begin_reading_coords = True
        
        #closest_distance = math.sqrt((self.temp_drillcoords[0][0])**2 + (self.temp_drillcoords[0][1])**2)
        closest_distance = self.calculate_distance(0, 0, self.temp_drillcoords[0][0], self.temp_drillcoords[0][1])
        if self.min_coordsX > 10 or self.min_coordsY > 10:
            temp_drill_offsetX = self.min_coordsX
            temp_drill_offsetY = self.min_coordsY
            
        
        current_drill_id = 0
        #print("current drill id " + str(current_drill_id))
        
        #i, color in enumerate(colors)
        for current_drill_id, a_drill_spec in enumerate(self.drill_size_definitions):
            print("current drill id " + str(current_drill_id))
            a_closest_found = True
            while a_closest_found:
                closest_distance = 1000
                a_closest_found = False
                
                #print(len(self.drillcoords))
                for a_temp_drillcoord in self.temp_drillcoords:
                    #print(i)
                    if not a_temp_drillcoord[2] and a_temp_drillcoord[3] == current_drill_id:
                        #distance = math.sqrt((current_drill_X - self.temp_drillcoords[i][0])**2 + (current_drill_Y - self.temp_drillcoords[i][1])**2)
                        distance = self.calculate_distance(current_drill_X, current_drill_Y, a_temp_drillcoord[0], a_temp_drillcoord[1])
                        if distance < closest_distance:
                            #closest_drill = i
                            the_closest_drill = a_temp_drillcoord
                            closest_distance = distance
                            a_closest_found = True
                            #print("a closest found")
                        
                    
                
                
                if a_closest_found:
                    
                    temp_screen_Xcoord = the_closest_drill[0] - self.min_coordsX
                    temp_screen_Ycoord = the_closest_drill[1] - self.min_coordsY
                    
                    temp_drill_Xcoord = the_closest_drill[0] - temp_drill_offsetX
                    temp_drill_Ycoord = the_closest_drill[1] - temp_drill_offsetY
                    
                    self.drillcoords.append([temp_screen_Xcoord, temp_screen_Ycoord,temp_drill_Xcoord, temp_drill_Ycoord, False, False, current_drill_id])
                    #  self.drillcoords.append([self.temp_drillcoords[closest_drill][0] - self.min_coordsX, self.temp_drillcoords[closest_drill][1] - self.min_coordsY,self.temp_drillcoords[closest_drill][0] - self.min_coordsX, self.temp_drillcoords[closest_drill][1] - self.min_coordsY, False, False])
                    the_closest_drill[2] = True
                    
                    #print("closest: " + str(temp_screen_Xcoord) + "," + str(temp_screen_Ycoord) +"|" + str(temp_drill_Xcoord) + "," + str(temp_drill_Ycoord))
                    if begin_reading_coords:
                        real_min_coordsX = temp_screen_Xcoord
                        real_min_coordsY = temp_screen_Ycoord
                        
                        begin_reading_coords = False
                    else:
                        real_min_coordsX = temp_screen_Xcoord if temp_screen_Xcoord < real_min_coordsX else real_min_coordsX
                        real_min_coordsY = temp_screen_Ycoord if temp_screen_Ycoord < real_min_coordsY else real_min_coordsY
                        
                    
                    
                    current_drill_X = the_closest_drill[0]
                    current_drill_Y = the_closest_drill[1]
                    # print(str(self.temp_drillcoords[closest_drill][0] - self.min_coordsX) + "," + str(self.temp_drillcoords[closest_drill][1] - self.min_coordsY))
                 #else {
                #current_drill_id += 1
                #print("current drill id " + str(current_drill_id))
            
        
        
        self.adj_min_coordsX = real_min_coordsX
        self.adj_min_coordsY = real_min_coordsY
        
    
    def closecnc(self):
        if cancnc:
            cnc.close()
        
        reader.close()
        return 0
    

class Handler:
    mycnccam = 0
    camimage = 0
    darea = 0
    
    tgl_cnc_sethome = 0
    btn_cnc_sethome = 0
    btn_cnc_setmm = 0
    txt_mm = 0
    tgl_cnc_calibrate = 0
    lbl_calib_level = 0
    
    btn_cnc_up = 0
    btn_cnc_left = 0
    btn_cnc_right = 0
    btn_cnc_down = 0
    lbl_cnc_mm = 0
    
    btn_cnc_home = 0
    btn_drill_verify = 0
    tgl_manual_verify = 0
    tgl_drill_verify = 0
    btn_drill_check = 0
    
    calibrate_stage = 1
    #calib_anchor = [-1, 0, 0, False] * 3
    calib_anchor = []
    
    drill_to_verify = 0
    verification_complete = False
    
    minimum_margin = 0
    scale_coordsX = 0
    scale_coordsY = 0
    scale_coordsXY = 0
    minimum_margin = 0
    
    home_coord = dict(
    x=0,
    y=0,
    radius=5,
    active=False,
    hole_id=0
    )
    
    def on_btn_for_misc_clicked(self, button):
        print("writing data ...")
        
        X_correction =  1.45
        Y_correction =  -4.496
        
        current_size_id = -1
        
        feed_rate = 70
        spindle_speed = 200
        mark_depth = 0.2
        drill_depth = 1.9
        
        for a_correctcoord in self.mycnccam.drillcoords:
            
            if a_correctcoord[4]:
                if a_correctcoord[6] != current_size_id:
                    print("T" + str(self.mycnccam.drill_size_definitions[a_correctcoord[6]]['id']))
                    current_size_id = a_correctcoord[6]
                
                ##print("X" + ("%0.4f" % (a_correctcoord[2] - 1.45)) + "Y" + ("%0.4f" % (a_correctcoord[3] +4.496)))
            
            #arrayofclosest.append(closestdict)
        
        
        print("... done")
        
        
        #----------------------- we write check file ------------------------
        checkfile = open("output_check.gcode", "w")
        
        checkfile.write("G21\n")
        checkfile.write("G90\n")
        checkfile.write("G94\n")
        checkfile.write("F65.00\n")
        checkfile.write("G00 Z2.5000\n")
        #checkfile.write("M03\n")
        checkfile.write("G4 P1\n")
        
        for a_correctcoord in self.mycnccam.drillcoords:
            
            if a_correctcoord[4]:
                
                checkfile.write("G00 X" + ("%0.4f" % (a_correctcoord[2] - X_correction)) + "Y" + ("%0.4f" % (a_correctcoord[3] - Y_correction)) + "\n")
                checkfile.write("G01 Z0.0000\n")
                checkfile.write("G01 Z0\n")
                checkfile.write("M5\n")
                checkfile.write("M6\n")
                checkfile.write("M0\n")
                checkfile.write("G00 Z2.5000\n")
            
            #arrayofclosest.append(closestdict)
        
        checkfile.write("G00 Z2.5000\n")
        checkfile.write("G00 X0.0000Y0.0000\n")
        checkfile.write("M05\n")
        
        checkfile.close()
        
        #----------------------- we write marking file ------------------------
        markfile = open("output_mark.gcode", "w")
        
        markfile.write("G21\n")
        markfile.write("G90\n")
        markfile.write("G94\n")
        markfile.write("F" + ("%0.2f" % (feed_rate)) + "\n")
        markfile.write("G00 Z2.5000\n")
        markfile.write("M03 S" + str(spindle_speed) + "\n")
        markfile.write("G4 P1\n")
        
        for a_correctcoord in self.mycnccam.drillcoords:
            
            if a_correctcoord[4]:
                
                markfile.write("G00 X" + ("%0.4f" % (a_correctcoord[2] - X_correction)) + "Y" + ("%0.4f" % (a_correctcoord[3] - Y_correction)) + "\n")
                markfile.write("G00 Z1.0000\n")
                markfile.write("G01 Z" + ("%0.4f" % (-1*mark_depth)) + "\n")
                markfile.write("G01 Z0\n")
                
                markfile.write("G00 Z2.5000\n")
            
            #arrayofclosest.append(closestdict)
        
        markfile.write("G00 Z2.5000\n")
        markfile.write("G00 X0.0000Y0.0000\n")
        markfile.write("M05\n")
        
        markfile.close()
        
        #----------------------- we write drilling file ------------------------
        drillfile = open("output_drill.gcode", "w")
        
        drillfile.write("G21\n")
        drillfile.write("G90\n")
        drillfile.write("G94\n")
        drillfile.write("F" + ("%0.2f" % (feed_rate)) + "\n")
        drillfile.write("G00 Z2.5000\n")
        drillfile.write("M03 S" + str(spindle_speed) + "\n")
        drillfile.write("G4 P1\n")
        
        for a_correctcoord in self.mycnccam.drillcoords:
            
            if a_correctcoord[4]:
                if a_correctcoord[6] != current_size_id:
                    # set toolchange height here normally 4mm
                    drillfile.write("G00 Z20.0000\n")
                    drillfile.write("T" + str(self.mycnccam.drill_size_definitions[a_correctcoord[6]]['id']) + "\n")
                    # spindle stop
                    drillfile.write("M5\n")
                    # tool change
                    drillfile.write("M6\n")
                    drillfile.write("(MSG, Change to tool dia=" + ("%0.4f" % (self.mycnccam.drill_size_definitions[a_correctcoord[6]]['size']) + ")\n"))
                    # program stop
                    drillfile.write("M0\n")
                    drillfile.write("G00 Z2.0000\n")
                    # program stop to double check height
                    drillfile.write("M0\n")
                    drillfile.write("M03 S" + str(spindle_speed) + "\n")
                    drillfile.write("G4 P1\n")
                    
                    current_size_id = a_correctcoord[6]
                
                
                drillfile.write("G00 X" + ("%0.4f" % (a_correctcoord[2] - X_correction)) + "Y" + ("%0.4f" % (a_correctcoord[3] - Y_correction)) + "\n")
                drillfile.write("G00 Z1.0000\n")
                drillfile.write("G01 Z" + ("%0.4f" % (-1*drill_depth)) + "\n")
                drillfile.write("G01 Z0\n")
                
                drillfile.write("G00 Z2.5000\n")
            
            #arrayofclosest.append(closestdict)
        
        drillfile.write("G00 Z2.5000\n")
        drillfile.write("G00 X0.0000Y0.0000\n")
        drillfile.write("M05\n")
        
        drillfile.close()
        
    
    def init_margins(self):
        self.minimum_margin = self.mycnccam.largest_drill_size * 10 + 10
        
        print("min margin " + str(self.minimum_margin))
        
        self.scale_coordsX = (1000 - (self.minimum_margin*2))/(self.mycnccam.max_coordsX - self.mycnccam.min_coordsX)
        self.scale_coordsY = (700 - (self.minimum_margin*2))/(self.mycnccam.max_coordsY - self.mycnccam.min_coordsY)
        
        #self.scale_coordsXY = self.scale_coordsX if self.scale_coordsX < self.scale_coordsY else self.scale_coordsY  
        
        if self.scale_coordsX < self.scale_coordsY:
            self.scale_coordsXY = self.scale_coordsX
            self.margin_coordsX = self.minimum_margin
            self.margin_coordsY = (700 - (self.mycnccam.max_coordsY - self.mycnccam.min_coordsY) * self.scale_coordsXY)/2
            
        else:
            self.scale_coordsXY = self.scale_coordsY
            self.margin_coordsX = (1000 - (self.mycnccam.max_coordsX - self.mycnccam.min_coordsX) * self.scale_coordsXY)/2
            self.margin_coordsY = self.minimum_margin
            
        
        
        print("minx: " + str(self.mycnccam.adj_min_coordsX) + " maxx: " + str(self.mycnccam.max_coordsX) + " scale: " + str(self.scale_coordsXY))
        
        #self.scale_coordsXY = 10
        
    
    def __init__(self):
        for i in range(0,3):
            print("init calib" + str(i))
            self.calib_anchor.append([-1, 0, 0, False])
        
        
    
    def on_darea_draw(self, widget, event):
        
        myGdkWindow = widget.get_window()
        cr = myGdkWindow.cairo_create()
        #cr = cairo.create()
        
        cr.set_source_rgb(0.8, 0.8, 0.8)
        cr.rectangle(0, 0, 1000, 700)
        cr.fill()
        
        cr.set_source_rgb(0.7, 0.7, 0.2)
        cr.arc(10, 10, 5, 0, 2*math.pi)
        cr.fill()
        
        cr.set_source_rgb(0.7, 0.7, 0.2)
        cr.arc(990, 690, 5, 0, 2*math.pi)
        cr.fill()
        
        cr.set_line_width(1)
        cr.set_source_rgb(0.7, 0.2, 0.0)
        
        drill_id = self.mycnccam.current_coordinate_index
        drill_location = self.get_drill_on_screen(drill_id)
        
        xpos = drill_location['x']
        ypos = drill_location['y']
        drill_diameter = drill_location['diameter'] 
        
        cr.arc(xpos, ypos, 5, 0, 2*math.pi)
        cr.fill()
        
        if self.tgl_drill_verify.get_active():
            
            drill_id = self.drill_to_verify
            drill_location = self.get_drill_on_screen(drill_id)
            
            xpos = drill_location['x']
            ypos = drill_location['y']
            drill_diameter = drill_location['diameter'] + 1 
            
            
            #cr.rectangle(xpos - 6, ypos - 6, 12, 12)
            cr.rectangle(xpos - drill_diameter, ypos - drill_diameter, drill_diameter*2, drill_diameter*2)
            cr.fill()
        
        
        for an_anchor in self.calib_anchor:
            if an_anchor[0] != -1:
                
                drill_id = an_anchor[0]
                drill_location = self.get_drill_on_screen(drill_id)
                
                xpos = drill_location['x']
                ypos = drill_location['y']
                drill_diameter = drill_location['diameter'] + 1
                
                if an_anchor[3]:
                    cr.rectangle(xpos - drill_diameter, ypos - drill_diameter, drill_diameter*2, drill_diameter*2)
                    cr.stroke()
                    
                else:
                    cr.arc(xpos, ypos, 5, 0, 2*math.pi)
                    cr.fill() 
                
            
            
        
        
        #len(self.drillcoords)
        
        #for i in range(self.mycnccam.num_coords){
        for i in range(len(self.mycnccam.drillcoords)):
            
            drill_id = i
            drill_location = self.get_drill_on_screen(drill_id)
            
            xpos = drill_location['x']
            ypos = drill_location['y']
            drill_diameter = drill_location['diameter']
            
            
            if self.mycnccam.drillcoords[i][4]:
                if i == self.mycnccam.current_coordinate_index:
                    cr.set_source_rgb(0.2, 0.23, 0.9)
                else:
                    cr.set_source_rgb(0.4, 0.9, 0.4)
                
                cr.arc(xpos, ypos, drill_diameter, 0, 2*math.pi)
                cr.fill()
                
            
            
            #cr.set_line_width(1)
            cr.set_source_rgb(0.7, 0.2, 0.0)
            #print("drawing " + str(i) + " x: " + str(xpos)  + " y: " + str(ypos))
            
            #cr.translate(xpos, ypos)
            cr.arc(xpos, ypos, drill_diameter, 0, 2*math.pi)
            cr.stroke()
            
        
    
    def on_camimage_draw(self, widget, event):
        default_file = "test.txt"
        camera_image_valid = True
        num_retries = 0
        
        
        
        if self.mycnccam.camera.isOpened():
            if not self.mycnccam.camera_busy:
                camera_image_valid = False
                while not camera_image_valid and num_retries < 5:
                    
                    # Loads an image
                    #src = cv.imread(cv.samples.findFile(filename), cv.IMREAD_COLOR)
                    ret, src = self.mycnccam.camera.read()
                    # Check if image is loaded fine
                    if src is None:
                        print ('Error opening image 2!')
                        #print ('Usage hough_circle.py [image_name -- default ' + default_file + '] \n')
                        # return -1
                        
                        num_retries = num_retries + 1
                        print('retry => ' + str(num_retries) + ' \n')
                        time.sleep(0.25)
                    else:
                        camera_image_valid = True
                        #print('Found camera image!')
                    
                
            else:
                ret =  self.mycnccam.ret
                src = self.mycnccam.src
                camera_image_valid = True
                
            
            
            if(camera_image_valid):
                src = cv.cvtColor(src, cv.COLOR_BGR2RGB)
                
                if self.mycnccam.circles is not None:
                    
                    self.mycnccam.circles = np.uint16(np.around(self.mycnccam.circles))
                    #nearest_distance = 400
                    #closest_found = False
                    for i in self.mycnccam.circles[0,:]:
                        
                        center = (i[0], i[1])
                        # circle center
                        cv.circle(src, center, 1, (0, 100, 100), 3)
                        # circle outline
                        radius = i[2]
                        cv.circle(src, center, radius, (255, 0, 255), 3)
                    
                    
                
                
                if self.mycnccam.real_closest_exists:
                    cv.circle(src, self.mycnccam.real_closest_circle, self.mycnccam.real_closest_radius, (0, 200, 0), 3)
                
                
                if  not self.tgl_manual_verify.get_active() and not self.tgl_cnc_calibrate.get_active():
                    cv.line(src, (320, 220), (320, 260), (0,200,0), 1)
                    cv.line(src, (300, 240), (340, 240), (0,200,0), 1)
                
                cv.circle(src, (320, 240), self.mycnccam.furthest_allowed_closest, (200, 0, 0), 1)
                
                pb = GdkPixbuf.Pixbuf.new_from_data(src.tostring(),
                GdkPixbuf.Colorspace.RGB,
                False,
                8,
                src.shape[1],
                src.shape[0],
                src.shape[2]*src.shape[1])
                self.camimage.set_from_pixbuf(pb.copy())
                
            
            
            self.camimage.show()
            
        
        time.sleep(0.1)
        #time.sleep(0.25)
    
    def onDestroy(self, *args):
        
        Gtk.main_quit()
    
    def onButtonPressed(self, button):
        print("Starting check ...")
        #self.mycnccam.start_timer()
        self.mycnccam.check_closest()
        
        print("... done")
    
    def find_drill(self, xpos, ypos):
        return_drill = -1
        
        last_distance = 25
        
        for i in range(self.mycnccam.num_coords):
            
            xpos_real = (self.mycnccam.drillcoords[i][0] - self.mycnccam.adj_min_coordsX) * self.scale_coordsXY + self.margin_coordsX
            ypos_real = 700 - ((self.mycnccam.drillcoords[i][1]  - self.mycnccam.adj_min_coordsY) * self.scale_coordsXY  + self.margin_coordsY) 
            drill_diameter = self.mycnccam.drill_size_definitions[self.mycnccam.drillcoords[i][6]]['size'] * 10
            
            distance = math.sqrt((xpos - xpos_real)**2 + (ypos - ypos_real)**2)
            
            #print("real positions " + str(xpos_real) + "," + str(ypos_real) + " | " + str(distance))
            
            if distance < drill_diameter and distance < last_distance:
                return_drill = i
                last_distance = distance
            
        
        
        return return_drill
    
    def get_drill_on_screen(self, drill_id):
        xpos = (self.mycnccam.drillcoords[drill_id][0] - self.mycnccam.adj_min_coordsX) * self.scale_coordsXY + self.margin_coordsX
        ypos = 700 - ((self.mycnccam.drillcoords[drill_id][1]  - self.mycnccam.adj_min_coordsY) * self.scale_coordsXY + self.margin_coordsY) 
        drill_diameter = int(self.mycnccam.drill_size_definitions[self.mycnccam.drillcoords[drill_id][6]]['size'] * 10)
        
        
        return  dict(x=xpos, y=ypos, diameter=drill_diameter)
        
    
    def add_filters(self, dialog):
        
        filter_drill = Gtk.FileFilter()
        filter_drill.set_name("Drill files")
        filter_drill.add_pattern("*.drl")
        dialog.add_filter(filter_drill)
        
        filter_any = Gtk.FileFilter()
        filter_any.set_name("Any files")
        filter_any.add_pattern("*")
        dialog.add_filter(filter_any)
        
    
    def on_darea_button_press_event(self, widget, event):
        
        
        print("click "  + " x: " + str(event.x)  + " y: " + str(event.y))
        
        drill_id = self.find_drill(event.x, event.y)
        drill_xpos = self.mycnccam.drillcoords[drill_id][2]
        drill_ypos = self.mycnccam.drillcoords[drill_id][3]
        
        print("drill -> " + str(drill_id)  + " x: " + str(drill_xpos)  + " y: " + str(drill_ypos))
        
        if drill_id != -1:
            if  self.tgl_cnc_calibrate.get_active():
                if self.calibrate_stage == 1:
                    if self.home_coord['active']:
                        self.mycnccam.cnc_goto( drill_xpos - self.home_coord['x'], drill_ypos - self.home_coord['y'])
                        self.temp_first_drillX = self.home_coord['x']
                        self.temp_first_drillY = self.home_coord['y']
                    else:
                        self.mycnccam.cnc_goto( drill_xpos, drill_ypos)
                        self.temp_first_drillX = self.mycnccam.current_coordinate_X
                        self.temp_first_drillY = self.mycnccam.current_coordinate_Y
                    
                    self.calib_anchor[0][0] = drill_id
                    
                
                elif self.calibrate_stage ==  2:
                    self.calib_anchor[1][0] = drill_id
                    if self.home_coord['active']:
                        self.mycnccam.cnc_goto( drill_xpos - self.home_coord['x'], drill_ypos - self.home_coord['y'])
                    else:
                        drill_xpos += self.calib_displaceX
                        drill_ypos += self.calib_displaceY
                        self.mycnccam.cnc_goto( drill_xpos, drill_ypos)
                    
                
                elif self.calibrate_stage ==  3:
                    self.calib_anchor[2][0] = drill_id
                    if self.home_coord['active']:
                        self.mycnccam.cnc_goto( drill_xpos - self.home_coord['x'], drill_ypos - self.home_coord['y'])
                    else:
                        drill_xpos += self.calib_displaceX
                        drill_ypos += self.calib_displaceY
                        self.mycnccam.cnc_goto(drill_xpos, drill_ypos)
                    
                
                elif self.calibrate_stage ==  4:
                    
                    self.mycnccam.cnc_goto( drill_xpos, drill_ypos)
                
                
            elif self.tgl_cnc_autohome.get_active():
                self.mycnccam.current_coordinate_index = drill_id
                self.current_drill_diameter = self.mycnccam.drillcoords[drill_id][6]
                #self.mycnccam.cnc_goto( drill_xpos, drill_ypos)
            else:
                self.mycnccam.current_coordinate_index = drill_id
                self.current_drill_diameter = self.mycnccam.drillcoords[drill_id][6]
                self.mycnccam.cnc_goto( drill_xpos, drill_ypos)
            
            #self.mycnccam.check_closest()
        
        self.mycnccam.camera_display()
        widget.queue_draw()
    
    def on_camevents_button_press_event(self, widget, event):
        print("click "  + " x: " + str(event.x)  + " y: " + str(event.y))
        #mouse at center ends up being at 332, 253
        cam_peturbX = ((320 - (event.x - 12))/self.mycnccam.clb_mmpix)
        cam_peturbY = (((event.y - 13) - 240)/self.mycnccam.clb_mmpix)
        
        print("x,y ", "%0.3f" % ((320 - event.x)/self.mycnccam.clb_mmpix), "%0.3f" % ((event.y - 240)/self.mycnccam.clb_mmpix))
        
        
        if  self.tgl_cnc_calibrate.get_active():
            if self.calibrate_stage < 4:
                self.calib_anchor[self.calibrate_stage - 1][1] = self.mycnccam.current_coordinate_X - cam_peturbX
                self.calib_anchor[self.calibrate_stage - 1][2] = self.mycnccam.current_coordinate_Y - cam_peturbY
                self.calib_anchor[self.calibrate_stage - 1][3] = True
                
                ##self.mycnccam.cnc_goto( self.mycnccam.current_coordinate_X - cam_peturbX, self.mycnccam.current_coordinate_Y - cam_peturbY)
                drill_id = self.calib_anchor[self.calibrate_stage - 1][0] 
                drill_size = self.mycnccam.drill_size_definitions[self.mycnccam.drillcoords[drill_id][6]]['size']
                
                print("XXpos" + str(self.mycnccam.current_coordinate_X - cam_peturbX))
                
                print("YYpos" + str(self.mycnccam.current_coordinate_Y - cam_peturbY))
                
                cam_coords = self.verify_drill_also(self.mycnccam.current_coordinate_X - cam_peturbX, self.mycnccam.current_coordinate_Y - cam_peturbY, drill_size)
                
                
                if cam_coords['valid']:
                    self.calib_anchor[self.calibrate_stage - 1][1] = cam_coords['x']
                    self.calib_anchor[self.calibrate_stage - 1][2] = cam_coords['y']
                    
                
                
            
            
            if self.calibrate_stage == 1:
                self.calib_displaceX = (self.mycnccam.current_coordinate_X - cam_peturbX) - self.temp_first_drillX
                self.calib_displaceY = (self.mycnccam.current_coordinate_Y - cam_peturbY) - self.temp_first_drillY
                self.calibrate_stage = 2
                self.lbl_calib_level.set_text(str(self.calibrate_stage))
            
            elif self.calibrate_stage == 2:
                self.calibrate_stage = 3
                self.lbl_calib_level.set_text(str(self.calibrate_stage))
            
            elif self.calibrate_stage == 3:
                self.calibrate_stage = 4
                #	self.tgl_cnc_calibrate.set_active(False)
                self.lbl_calib_level.set_text("offset")
                for i in range(0,3):
                    print("calibs: " + str(i) + " " + str(self.calib_anchor[i][0]) + " : " + str(self.calib_anchor[i][1]) + " : "+ str(self.calib_anchor[i][2]) + " : "+ str(self.calib_anchor[i][3]) )
                
                
                first_anchor = self.calib_anchor[0][0]
                second_anchor = self.calib_anchor[1][0]
                third_anchor = self.calib_anchor[2][0]
                
                T2 = [[self.calib_anchor[0][1],self.calib_anchor[1][1], self.calib_anchor[2][1]],
                [self.calib_anchor[0][2],self.calib_anchor[1][2],self.calib_anchor[2][2]]]
                T1 = [[self.mycnccam.drillcoords[first_anchor][2],self.mycnccam.drillcoords[second_anchor][2],self.mycnccam.drillcoords[third_anchor][2]],
                [self.mycnccam.drillcoords[first_anchor][3],self.mycnccam.drillcoords[second_anchor][3],self.mycnccam.drillcoords[third_anchor][3]],
                [1,1,1]]
                
                
                M = np.dot(T2,np.linalg.inv(T1))
                
                print(M)
                
                #y = np.linalg.inv(x) 
                
                #for i in range(self.mycnccam.num_coords){
                for a_drillcoord in self.mycnccam.drillcoords:
                    incoordmatrix = [[a_drillcoord[2]],
                    [a_drillcoord[3]],
                    [1]]
                    
                    outcoordmatrix = np.dot(M,incoordmatrix)
                    a_drillcoord[2] = outcoordmatrix[0][0]
                    a_drillcoord[3] = outcoordmatrix[1][0]
                    # print("orig: " + str(self.mycnccam.drillcoords[i][2]) + "," + str(self.mycnccam.drillcoords[i][3]))
                    # print("new: " + str(outcoordmatrix[0][0]) + "," + str(outcoordmatrix[1][0]))
                
                
                drill_xpos = self.mycnccam.drillcoords[third_anchor][2]
                drill_ypos = self.mycnccam.drillcoords[third_anchor][3]
                self.mycnccam.cnc_goto( drill_xpos, drill_ypos)
                
                #let's try ending one stage under instead
                self.calibrate_stage = 1
                self.tgl_cnc_calibrate.set_active(False)
                self.lbl_calib_level.set_text("done")
                
            elif self.calibrate_stage == 4:
                self.calibrate_stage = 1
                self.tgl_cnc_calibrate.set_active(False)
                self.lbl_calib_level.set_text("done")
                
                cam_peturbX = ((320 - (event.x - 12))/self.mycnccam.clb_mmpix)
                cam_peturbY = (((event.y - 13) - 240)/self.mycnccam.clb_mmpix)
                
                for a_drillcoord in self.mycnccam.drillcoords:
                    a_drillcoord[2] -= cam_peturbX
                    a_drillcoord[3] -= cam_peturbY
                    
                
                
            
            
        elif  self.tgl_cnc_autohome.get_active():
            
            self.mycnccam.cnc_set_home()
            #self.mycnccam.cnc_goto( cam_peturbX, cam_peturbY)
            drill_id = self.mycnccam.current_coordinate_index
            drill_size = self.mycnccam.drill_size_definitions[self.mycnccam.drillcoords[drill_id][6]]['size']
            
            self.mycnccam.cnc_goto(0 - cam_peturbX, 0 - cam_peturbY )
            cam_coords = self.verify_drill_also(0 - cam_peturbX, 0 - cam_peturbY, drill_size)
            
            if cam_coords['valid']:
                #self.calib_anchor[self.calibrate_stage - 1][1] = cam_coords['x']
                #self.calib_anchor[self.calibrate_stage - 1][2] = cam_coords['y']
                print("home confirmed")
                self.mycnccam.cnc_goto(cam_coords['x'], cam_coords['y'] )
                self.home_coord['x'] = self.mycnccam.drillcoords[drill_id][2]
                self.home_coord['y'] = self.mycnccam.drillcoords[drill_id][3]
                self.home_coord['active'] = True
                self.home_coord['radius'] = drill_size
                
            else:
                print("home failed")
            
            
            self.mycnccam.cnc_set_home() 
            self.tgl_cnc_autohome.set_active(False)   
            
            
        elif  self.tgl_manual_verify.get_active():
            current_drill_id = self.mycnccam.current_coordinate_index
            self.mycnccam.drillcoords[current_drill_id][2] -= cam_peturbX
            self.mycnccam.drillcoords[current_drill_id][3] -= cam_peturbY
            self.mycnccam.drillcoords[current_drill_id][4] = True
            print("manually verified " + str(current_drill_id))
            
            drill_xpos = self.mycnccam.drillcoords[current_drill_id][2]
            drill_ypos = self.mycnccam.drillcoords[current_drill_id][3]
            self.mycnccam.cnc_goto( drill_xpos, drill_ypos)
        
        
        self.darea.queue_draw()
        widget.queue_draw()
    
    def verf_thread(self):
        while not self.verification_complete and self.tgl_drill_verify.get_active():
            
            self.verify_drill(self.drill_to_verify)
            
            self.drill_to_verify += 1
            if self.drill_to_verify >= self.mycnccam.num_coords:
                self.verification_complete = True
                self.tgl_drill_verify.set_active(False)
            
            self.darea.queue_draw()
        
    
    def on_mnu_item_open_activate(self, widget):
        print("A File|Open menu item was selected.")
        
        dialog = Gtk.FileChooserDialog("Please choose a file", None,
        Gtk.FileChooserAction.OPEN,
        (Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL,
        Gtk.STOCK_OPEN, Gtk.ResponseType.OK))
        
        self.add_filters(dialog)
        
        response = dialog.run()
        if response == Gtk.ResponseType.OK:
            print("Open clicked")
            print("File selected: " + dialog.get_filename())
            path = dialog.get_filename()
            self.mycnccam.load_drills(path)
            self.init_margins()
            dialog.destroy()
        
        elif response == Gtk.ResponseType.CANCEL:
            print("Cancel clicked")
            dialog.destroy()
        
        
        widget.queue_draw()
        self.darea.queue_draw()
    
    def verify_drill(self, drill_id):
        drill_xpos = self.mycnccam.drillcoords[drill_id][2]
        drill_ypos = self.mycnccam.drillcoords[drill_id][3]
        drill_diameter = self.mycnccam.drill_size_definitions[self.mycnccam.drillcoords[drill_id][6]]['size']
        
        self.mycnccam.cnc_get_status()
        self.mycnccam.camera_display()
        self.mycnccam.cnc_goto( drill_xpos, drill_ypos)
        
        while not self.mycnccam.cnc_get_status():
            time.sleep(0.1)
        
        
        self.mycnccam.check_closest(drill_diameter)
        
        time.sleep(0.5)
        
        self.mycnccam.drillcoords[drill_id][5] = True
        
        #num_retries = 0
        
        if self.mycnccam.real_closest_exists:
            #correctcoords.append(( (drillcoords[correction_index][0] - (320 - real_closest_circle[0])/clb_mmpix), drillcoords[correction_index][1] + (240 - real_closest_circle[1])/clb_mmpix)) 
            self.mycnccam.drillcoords[drill_id][2] = drill_xpos - (320 - self.mycnccam.real_closest_circle[0])/self.mycnccam.clb_mmpix 
            self.mycnccam.drillcoords[drill_id][3] = drill_ypos + (240 - self.mycnccam.real_closest_circle[1])/self.mycnccam.clb_mmpix
            self.mycnccam.drillcoords[drill_id][4] = True
        else:
            self.mycnccam.drillcoords[drill_id][4] = False
        
    
    def verify_drill_also(self, xcoord, ycoord, drill_size):
        ret_coords = dict()
        ret_coords['valid'] = False
        
        drill_xpos = xcoord
        drill_ypos = ycoord
        drill_diameter = drill_size
        #    drill_diameter = self.mycnccam.drill_size_definitions[self.mycnccam.drillcoords[drill_id][6]]['size']
        drrill_diameter = drill_size
        self.mycnccam.cnc_get_status()
        self.mycnccam.camera_display()
        self.mycnccam.cnc_goto( drill_xpos, drill_ypos)
        time.sleep(0.3)
        
        self.mycnccam.cnc_get_status()
        self.mycnccam.check_closest(drill_diameter)
        
        time.sleep(0.1)
        
        if self.mycnccam.real_closest_exists:
            #correctcoords.append(( (drillcoords[correction_index][0] - (320 - real_closest_circle[0])/clb_mmpix), drillcoords[correction_index][1] + (240 - real_closest_circle[1])/clb_mmpix)) 
            
            ret_coords['x'] = drill_xpos - (320 - self.mycnccam.real_closest_circle[0])/self.mycnccam.clb_mmpix
            ret_coords['y'] = drill_ypos + (240 - self.mycnccam.real_closest_circle[1])/self.mycnccam.clb_mmpix
            ret_coords['valid'] = True
            print("Xpos" + str(ret_coords['x']))
            
            print("Ypos" + str(ret_coords['y']))
            
            
        
        
        
        return ret_coords
    
    def on_btn_cnc_sethome_clicked(self, button):
        print("set home")
        
        if self.tgl_cnc_sethome.get_active():
            self.mycnccam.cnc_set_home()
            self.tgl_cnc_sethome.set_active(False)
            
            self.home_coord['x'] = 0
            self.home_coord['y'] = 0
            self.home_coord['active'] = False
            
            self.current_coordinate_X = 0
            self.current_coordinate_Y = 0
        
        
    
    def on_tgl_auto_home_toggled(self, button):
        if  self.tgl_cnc_calibrate.get_active():
            print("auto homing")
        else:
            print("auto home cancel")
        
        
        self.darea.queue_draw()
    
    def on_tgl_cnc_calibrate_toggled(self, button):
        if  self.tgl_cnc_calibrate.get_active():
            self.calibrate_stage = 1
            for an_anchor in self.calib_anchor:
                #print("clear calib" + str(i))
                an_anchor[0] = -1
                an_anchor[3] = False
            
            self.lbl_calib_level.set_text("1")
        else:
            self.lbl_calib_level.set_text("cancel")
        
        
        self.darea.queue_draw()
    
    def on_btn_cnc_setmm_clicked(self, button):
        print("set mm")
        
        cnc_mm_movement = 1
        cnc_mm_string = self.txt_mm.get_text()
        
        
        try:
            cnc_mm_movement = float(cnc_mm_string)
        
        except ValueError:
            print ("Not a float")
        
        
        if cnc_mm_movement > 50:
            cnc_mm_movement = 50
        
        
        if cnc_mm_movement <= 0:
            cnc_mm_movement = 1
        
        
        self.mycnccam.cnc_step_size = cnc_mm_movement
        self.lbl_cnc_mm.set_text(str(cnc_mm_movement))
        print("set to " + str(cnc_mm_movement) + " [" + cnc_mm_string + "] ")
    
    def on_btn_cnc_up_clicked(self, button):
        print("up")
        
        #self.mycnccam.camera_display()
        
        #self.mycnccam.cnc_step_size = cnc_mm_movement
        self.mycnccam.current_coordinate_Y += self.mycnccam.cnc_step_size
        self.mycnccam.cnc_goto( self.mycnccam.current_coordinate_X, self.mycnccam.current_coordinate_Y)
        
        self.mycnccam.camera_display()
    
    def on_btn_cnc_left_clicked(self, button):
        print("left")
        #self.mycnccam.camera_display()
        self.mycnccam.current_coordinate_X -= self.mycnccam.cnc_step_size
        self.mycnccam.cnc_goto( self.mycnccam.current_coordinate_X, self.mycnccam.current_coordinate_Y)
        self.mycnccam.camera_display()
    
    def on_btn_cnc_down_clicked(self, button):
        print("down")
        #self.mycnccam.camera_display()
        self.mycnccam.current_coordinate_Y -= self.mycnccam.cnc_step_size
        self.mycnccam.cnc_goto( self.mycnccam.current_coordinate_X, self.mycnccam.current_coordinate_Y)
        self.mycnccam.camera_display()
    
    def on_btn_cnc_right_clicked(self, button):
        print("right")
        #self.mycnccam.camera_display()
        self.mycnccam.current_coordinate_X += self.mycnccam.cnc_step_size
        self.mycnccam.cnc_goto( self.mycnccam.current_coordinate_X, self.mycnccam.current_coordinate_Y)
        self.mycnccam.camera_display()
    
    def on_btn_cnc_home_clicked(self, button):
        print("home")
        self.mycnccam.camera_display()
        self.mycnccam.cnc_goto( 0, 0)
        self.mycnccam.camera_display()
    
    def on_tgl_manual_verify_toggled(self, button):
        print("manual mode")
    
    def on_btn_drill_check_clicked(self, button):
        print("check")
        
        
        self.verify_drill(self.mycnccam.current_coordinate_index)
    
    def on_tgl_drill_verify_toggled(self, button):
        print("verify")
        
        if  self.tgl_drill_verify.get_active():
            #self.drill_to_verify = 0
            #self.verification_complete = False
            
            thread = threading.Thread(target=self.verf_thread)
            thread.daemon = True
            thread.start()
        
        
        
        self.darea.queue_draw()
    

if __name__ == "__main__":
    
    myHandler = Handler()
    builder = Gtk.Builder()
    builder.add_from_file("fundrill.gui")
    builder.connect_signals(myHandler)
    
    window = builder.get_object("window1")
    myHandler.camimage = builder.get_object("camimage")
    myHandler.darea = builder.get_object("darea")
    
    myHandler.tgl_cnc_sethome = builder.get_object("tgl_cnc_sethome")
    myHandler.btn_cnc_sethome = builder.get_object("btn_cnc_sethome")
    myHandler.tgl_cnc_autohome = builder.get_object("tgl_cnc_autohome")
    myHandler.btn_cnc_setmm = builder.get_object("btn_cnc_setmm")
    myHandler.txt_mm = builder.get_object("txt_mm")
    myHandler.tgl_cnc_calibrate = builder.get_object("tgl_cnc_calibrate")
    myHandler.lbl_calib_level = builder.get_object("lbl_calib_level")
    
    myHandler.btn_cnc_up = builder.get_object("btn_cnc_up")
    myHandler.btn_cnc_left = builder.get_object("btn_cnc_left")
    myHandler.btn_cnc_right = builder.get_object("btn_cnc_right")
    myHandler.btn_cnc_down = builder.get_object("btn_cnc_down")
    myHandler.lbl_cnc_mm = builder.get_object("lbl_cnc_mm")
    
    myHandler.btn_cnc_home = builder.get_object("btn_cnc_home")
    myHandler.btn_drill_verify = builder.get_object("btn_drill_verify")
    myHandler.tgl_manual_verify = builder.get_object("tgl_manual_verify")
    myHandler.tgl_drill_verify = builder.get_object("tgl_drill_verify")
    myHandler.btn_drill_check = builder.get_object("btn_drill_check")
    
    window.show_all()
    
    
    myHandler.mycnccam = CNCcam(sys.argv[1:])
    
    myHandler.mycnccam.displayHandler = myHandler
    ##myHandler.init_margins()
    
    
    Gtk.main()
    
    
    



