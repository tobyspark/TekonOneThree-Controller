#!/usr/bin/python
# -*- coding: utf-8 -*-

log_to_console = True

host_info = { 'tektononethree-pi1': { 'unit': 2, 'upsidedown': True}, 'tektononethree-pi3': { 'unit': 1, 'upsidedown': False} }

import time
import os
import socket
import cPickle as pickle

# LED STRIPS
from dotstar import Adafruit_DotStar

# OLED DISPLAY
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306
import Image
import ImageDraw
import ImageFont

# DRIVE via ARDUINO
import serial
drive_max_steps = 20000

# HOST
host_name = socket.gethostname()
if host_name not in host_info:
  print('FATAL: Hostname not recognised')
  exit()
host_unit = host_info[host_name]['unit'] 
host_upsidedown = host_info[host_name]['upsidedown'] 

if (log_to_console):
  print("Init start...")

### OLED

oled = Adafruit_SSD1306.SSD1306_128_64(rst=24, dc=23, spi=SPI.SpiDev(0, 0, max_speed_hz=8000000))
oled.begin()
oled.clear()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
image = Image.new('1', (oled.width, oled.height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0,0,oled.width, oled.height), outline=0, fill=0)

# Load default font.
font = ImageFont.truetype('miso-bold.ttf', 64)

# Draw something
draw.text((0, 0), 'INIT',  font=font, fill=255)
if host_upsidedown:
  oled.image(image.rotate(180))
else:
  oled.image(image)
oled.display()

# Keep a reference to the original buffer, to assign back after assigning to cached buffers
buffer_draw = oled._buffer
# Create a cache of buffer 'images' to pre-render display
buffer_cache = [None] * 10
for i in range(10):
  draw.rectangle((0,0,oled.width, oled.height), outline=0, fill=0)
  draw.text((0, 0), 'RUN ' + str(i),  font=font, fill=255)
  if host_upsidedown: 
    oled.image(image.rotate(180))
  else:
    oled.image(image)
  buffer_cache[i] = list(oled._buffer)

### DRIVE via ARDUINO

# Protocol is Command 2 MSB + Payload 6LSB = 1 Byte
# Command 11 = Sequence init vpos start
# Command 01 = Sequence run vpos start
# Command 00 = Continuing payload
# vpos is split into 3x 8bits, ie. 15bits ie. 2^18. ie. ~0-250k max range. 

arduinoSerial = serial.Serial("/dev/ttyAMA0", baudrate=115200, timeout=0, writeTimeout=0)

### LED 

# numpixels_F = 216 # Number of LEDs in strip
# numpixels_R = 90

numpixels_F = 72+67+69 # Number of LEDs in strip
numpixels_R = 80

datapin_R_RGB   = 16 # 36 Physical, 16 GPIO
clockpin_R_RGB  = 20 # 38 Physical, 20 GPIO
datapin_R_W = 25 # 22 Physical
clockpin_R_W = 12 # 32 Physical
datapin_F_RGB = 22 # 15 Physical
clockpin_F_RGB = 18 # 12 Physical
datapin_F_W = 17 # 11 Physical
clockpin_F_W = 27 # 13 Physical

# Initialise without image buffers, sending data direct for speed
# bytearray order: FF B G R
led_r_rgb = Adafruit_DotStar(0, datapin_R_RGB, clockpin_R_RGB)
led_r_w = Adafruit_DotStar(0, datapin_R_W, clockpin_R_W)
led_f_rgb = Adafruit_DotStar(0, datapin_F_RGB, clockpin_F_RGB)
led_f_w = Adafruit_DotStar(0, datapin_F_W, clockpin_F_W)

# Calculate gamma correction table, makes mid-range colors look 'right'
gamma = bytearray(256)
for i in range(256):
  gamma[i] = int(pow(float(i) / 255.0, 2.7) * 255.0 + 0.5)

# Initialise LED strips

for strip in [led_r_rgb, led_r_w, led_f_rgb, led_f_w]:
  strip.begin()           # Initialize pins for output

  offBytes = bytearray(numpixels_F * 4)
  for i in range(0, numpixels_F*4, 4):
    offBytes[i] = 0xFF
    offBytes[i+1] = 0x0
    offBytes[i+2] = 0x0
    offBytes[i+3] = 0x0

  strip.show(offBytes)

if (log_to_console):
  print("Init complete")

### SEQUENCE FILES

sequences = {};

items_command_index = 0
items_unit_index = 1
items_frame_index = 2
items_meta_time_index = 3 # time is in milliseconds
items_meta_vpos_index = 4 # vpos is 0-1
items_led_start_index = 3

sequence_folder = "tekton_sequences"
sequence_listdir = os.listdir(sequence_folder)
sequence_listdir.sort()
while True:
  for sequence_name in sequence_listdir:
    sequence_path = os.path.join(sequence_folder, sequence_name)
    
    name = os.path.splitext(sequence_name)[0]
    ext = os.path.splitext(sequence_name)[1]
    
    print("SEQUENCE: "+ name)
    
    # HACKATTACK. This assumes filename convention etc., down the line might have files / OSC with interleaved unit lines
    if name[0] != str(host_unit):
      print 'Reject sequence, wrong host for ' + sequence_path
      continue
    
    if ext not in ['.txt', '.TXT']:
      print 'Reject sequence file extension for ' + sequence_path 
      continue
    
    draw.rectangle((0,0,oled.width, oled.height), outline=0, fill=0)  
    draw.text((0, 0), 'S/'+ name[2:5],  font=font, fill=255)
    if host_upsidedown:
      oled.image(image.rotate(180))
    else:
      oled.image(image)
    oled.display()
    
    if (os.path.isfile(sequence_path + '.pickle')):
      with open(sequence_path + '.pickle', 'rb') as input:
        sequence_meta = pickle.load(input) 
        sequence_led_f_www = pickle.load(input) 
        sequence_led_f_rgb = pickle.load(input) 
        sequence_led_r_www = pickle.load(input) 
        sequence_led_r_rgb = pickle.load(input) 
      
      if (log_to_console):
        print("Sequence unpickled " + name)
    
    else:
      if (log_to_console):
        print("New sequence. Parse starting for " + name)
      
      sequence_file = open(sequence_path)
      sequence_line_count = sum(1 for line in sequence_file)
      sequence_file.seek(0)
      
      sequence_meta = [None] * sequence_line_count
      sequence_led_f_www = [None] * sequence_line_count
      sequence_led_f_rgb = [None] * sequence_line_count
      sequence_led_r_www = [None] * sequence_line_count
      sequence_led_r_rgb = [None] * sequence_line_count
      
      for line in sequence_file:
        items = line.rstrip().split(' ')
        
        # Basic test of line validity
        if len(items) < 5:
          print("Cannot parse line: " + str(line))
          continue
          
        frame_number = int(items[items_frame_index])
        frame_index = frame_number - 1 # assumes 1 is starting frame
        
        if items[items_command_index] == "/meta":
          sequence_meta[frame_index] = ( frame_number , int(items[items_meta_time_index]) , float(items[items_meta_vpos_index]) )
              
        elif items[items_command_index] == "/led_f":
          pixel_count = (len(items) - items_led_start_index) / 3
          pixel_index = 0
          sequence_led_f_www[frame_index] = bytearray(pixel_count * 4)
          sequence_led_f_rgb[frame_index] = bytearray(pixel_count * 4)
          
          for i in range(items_led_start_index, len(items), 3):
            
            red = int(items[i])
            blue = int(items[i+1])
            green = int(items[i+2])
             
            if red == blue == green:
              
              sequence_led_f_www[frame_index][pixel_index]   = 0xFF
              sequence_led_f_www[frame_index][pixel_index+1] = gamma[blue]
              sequence_led_f_www[frame_index][pixel_index+2] = gamma[green]
              sequence_led_f_www[frame_index][pixel_index+3] = gamma[red]
              
              sequence_led_f_rgb[frame_index][pixel_index]   = 0xFF
              sequence_led_f_rgb[frame_index][pixel_index+1] = 0x0
              sequence_led_f_rgb[frame_index][pixel_index+2] = 0x0
              sequence_led_f_rgb[frame_index][pixel_index+3] = 0x0
              
            else:
              
              sequence_led_f_www[frame_index][pixel_index]   = 0xFF
              sequence_led_f_www[frame_index][pixel_index+1] = 0x0
              sequence_led_f_www[frame_index][pixel_index+2] = 0x0
              sequence_led_f_www[frame_index][pixel_index+3] = 0x0
              
              sequence_led_f_rgb[frame_index][pixel_index]   = 0xFF
              sequence_led_f_rgb[frame_index][pixel_index+1] = gamma[blue]
              sequence_led_f_rgb[frame_index][pixel_index+2] = gamma[green]
              sequence_led_f_rgb[frame_index][pixel_index+3] = gamma[red]
            
            pixel_index += 4
          
        elif items[items_command_index] == "/led_r":
          pixel_count = (len(items) - items_led_start_index) / 3
          pixel_index = 0
          sequence_led_r_www[frame_index] = bytearray(pixel_count * 4)
          sequence_led_r_rgb[frame_index] = bytearray(pixel_count * 4)
          
          for i in range(items_led_start_index, len(items), 3):
            
            red = int(items[i])
            blue = int(items[i+1])
            green = int(items[i+2])
             
            if red == blue == green:
              
              sequence_led_r_www[frame_index][pixel_index]   = 0xFF
              sequence_led_r_www[frame_index][pixel_index+1] = gamma[blue]
              sequence_led_r_www[frame_index][pixel_index+2] = gamma[green]
              sequence_led_r_www[frame_index][pixel_index+3] = gamma[red]
              
              sequence_led_r_rgb[frame_index][pixel_index]   = 0xFF
              sequence_led_r_rgb[frame_index][pixel_index+1] = 0x0
              sequence_led_r_rgb[frame_index][pixel_index+2] = 0x0
              sequence_led_r_rgb[frame_index][pixel_index+3] = 0x0
              
            else:
              
              sequence_led_r_www[frame_index][pixel_index]   = 0xFF
              sequence_led_r_www[frame_index][pixel_index+1] = 0x0
              sequence_led_r_www[frame_index][pixel_index+2] = 0x0
              sequence_led_r_www[frame_index][pixel_index+3] = 0x0
              
              sequence_led_r_rgb[frame_index][pixel_index]   = 0xFF
              sequence_led_r_rgb[frame_index][pixel_index+1] = gamma[blue]
              sequence_led_r_rgb[frame_index][pixel_index+2] = gamma[green]
              sequence_led_r_rgb[frame_index][pixel_index+3] = gamma[red]
            
            pixel_index += 4
            
        else:
          print("Cannot parse line: " + line)
      
      if (log_to_console):
        print("Parse complete")
        
      # Store for next time round.
      with open(sequence_path + '.pickle', 'wb') as output:
        pickle.dump(sequence_meta, output, pickle.HIGHEST_PROTOCOL)
        pickle.dump(sequence_led_f_www, output, pickle.HIGHEST_PROTOCOL)
        pickle.dump(sequence_led_f_rgb, output, pickle.HIGHEST_PROTOCOL)
        pickle.dump(sequence_led_r_www, output, pickle.HIGHEST_PROTOCOL)
        pickle.dump(sequence_led_r_rgb, output, pickle.HIGHEST_PROTOCOL)
    
    ### GO! --------------------------------
    
    draw.rectangle((0,0,oled.width, oled.height), outline=0, fill=0)
    draw.text((0, 0), 'CUE',  font=font, fill=255)
    if host_upsidedown:
      oled.image(image.rotate(180))
    else:
      oled.image(image)
    oled.display()
    
    # Command 11 = Sequence init vpos start
    vpos_steps = int(sequence_meta[frame_index][2]*drive_max_steps)
    arduinoSerial.write(chr(0xC0 + ((vpos_steps >> 12) & 0x3F)) + chr((vpos_steps >> 6) & 0x3F) + chr(vpos_steps & 0x3F))
    if (log_to_console):
      print("Sequence init sent, vpos " + str(vpos_steps))
    time.sleep(1.0/60)
      
    arduinoSerial.flushInput()
    while arduinoSerial.read(1) != "S": 
      # Command 11 = Sequence init vpos start
      vpos_steps = int(sequence_meta[frame_index][2]*drive_max_steps)
      arduinoSerial.write(chr(0xC0 + ((vpos_steps >> 12) & 0x3F)) + chr((vpos_steps >> 6) & 0x3F) + chr(vpos_steps & 0x3F))
      if (log_to_console):
        print("Sequence init sent, vpos " + str(vpos_steps) + ". Waiting for start signal from Drive")
      time.sleep(1.0/60)
    
    draw.rectangle((0,0,oled.width, oled.height), outline=0, fill=0)
    draw.text((0, 0), 'RUN',  font=font, fill=255)
    if host_upsidedown:
      oled.image(image.rotate(180))
    else:
      oled.image(image)
    oled.display()
        
    if (log_to_console):
      print("Go!")
    
    start_time = int(time.time() * 1000)
  
    for meta in sequence_meta:
      
      if not meta:
        continue
      
      presentation_time = meta[1]
      frame_index = meta[0] - 1
      
      while presentation_time > int(time.time() * 1000) - start_time:
        oled._buffer = buffer_cache[frame_index % 10]  
        oled.display() 
        time.sleep(0.0001)
      
      # Set Drive
      
      #Command 01 = Sequence run vpos start
      vpos_steps = int(meta[2]*drive_max_steps)
      arduinoSerial.write(chr(0x40 + ((vpos_steps >> 12) & 0x3F)) + chr((vpos_steps >> 6) & 0x3F) + chr(vpos_steps & 0x3F))
      
      # Set LED Strips
      
      if sequence_led_f_www[frame_index] is not None:
        led_f_w.show(sequence_led_f_www[frame_index])
      if sequence_led_f_rgb[frame_index] is not None:
        led_f_rgb.show(sequence_led_f_rgb[frame_index])
      if sequence_led_r_www[frame_index] is not None:
          led_r_w.show(sequence_led_r_www[frame_index])
      if sequence_led_r_rgb[frame_index] is not None:
        led_r_rgb.show(sequence_led_r_rgb[frame_index])
      
      if (log_to_console):
        print("Frame " + str(frame_index) + " vpos_steps " + str(vpos_steps))
    
    # Sequence End
    
    for strip in [led_r_rgb, led_r_w, led_f_rgb, led_f_w]:
      offBytes = bytearray(numpixels_F * 4)
      for i in range(0, numpixels_F*4, 4):
        offBytes[i] = 0xFF
        offBytes[i+1] = 0x0
        offBytes[i+2] = 0x0
        offBytes[i+3] = 0x0
      
      strip.show(offBytes)
    
    oled._buffer = buffer_draw
  
