#!/usr/bin/python
# -*- coding: utf-8 -*-

import time

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
font = ImageFont.load_default()

# Draw something
draw.text((0, 0), 'TEKTON ONE THREE',  font=font, fill=255)
oled.image(image)
oled.display()

oled_status1_pos = (0,20)
oled_status1_poz = (oled.width,39)
oled_status2_pos = (0,40)
oled_status2_poz = (oled.width, 59)

### DRIVE via ARDUINO

arduinoSerial = serial.Serial("/dev/ttyAMA0", baudrate=115200, writeTimeout=0)

### LED 

numpixels_F = 216 # Number of LEDs in strip
numpixels_R = 90

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
  strip.setBrightness(64) # Limit brightness to ~1/4 duty cycle

  offBytes = bytearray(numpixels_F * 4)
  for i in range(0, numpixels_F*4, 4):
    offBytes[i] = 0xFF
    offBytes[i+1] = 0x0
    offBytes[i+2] = 0x0
    offBytes[i+3] = 0x0

  strip.show(offBytes)

### SEQUENCE FILES

items_command_index = 0
items_unit_index = 1
items_frame_index = 2
items_meta_time_index = 3 # time is in milliseconds
items_meta_vpos_index = 4 # vpos is 0-1
items_led_start_index = 3

sequence_file = open('tekton_sequence.txt')
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

### GO! --------------------------------

arduinoSerial.write('T')

while True:

  start_time = int(time.time() * 1000)
  time_stamp = 0

  for meta in sequence_meta:
    
    if not meta:
      continue
    
    current_time = int(time.time() * 1000) - start_time
    frame_index = meta[0] - 1
    
    # Set Drive
      
    arduinoSerial.write(chr(int(meta[2]*255.0)))
    
    # Set LED Strips
    
    # print(str(frame_index) + ", " + sequence_led_f_www[frame_index])
    
    led_f_w.show(sequence_led_f_www[frame_index])
    led_f_rgb.show(sequence_led_f_rgb[frame_index])
    led_r_w.show(sequence_led_r_www[frame_index])
    led_r_rgb.show(sequence_led_r_rgb[frame_index])
      
    # draw.rectangle([oled_status1_pos, oled_status1_poz], outline=0, fill=0)
    # draw.text(oled_status1_pos, "Frame:  " + items[frame_index],  font=font, fill=255)
    # draw.rectangle([oled_status2_pos, oled_status2_poz], outline=0, fill=0)
    # draw.text(oled_status2_pos, "Position: " + items[items_meta_vpos_index],  font=font, fill=255)
    # oled.image(image)  
    # oled.display()
      
  print("Time offset: " + str(int(time.time() * 1000) - start_time - time_stamp))

