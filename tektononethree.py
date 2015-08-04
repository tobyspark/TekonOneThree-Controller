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

led_r_rgb = Adafruit_DotStar(numpixels_R, datapin_R_RGB, clockpin_R_RGB)
led_r_w = Adafruit_DotStar(numpixels_R, datapin_R_W, clockpin_R_W)
led_f_rgb = Adafruit_DotStar(numpixels_F, datapin_F_RGB, clockpin_F_RGB)
led_f_w = Adafruit_DotStar(numpixels_F, datapin_F_W, clockpin_F_W)

for strip in [led_r_rgb, led_r_w, led_f_rgb, led_f_w]:
  strip.begin()           # Initialize pins for output
  strip.setBrightness(64) # Limit brightness to ~1/4 duty cycle

  for led_index in range(0, numpixels_F):
    strip.setPixelColor(led_index, 0x0)

  strip.show()

### SEQUENCE FILES

command_index = 0
unit_index = 1
frame_index = 2
meta_time_index = 3 # time is in milliseconds
meta_vpos_index = 4 # vpos is 0-1
led_start_index = 3

with open('tekton_sequence.txt') as f:
  sequence_contents = f.readlines();

arduinoSerial.write('T')

while True:

  start_time = int(time.time() * 1000)
  time_stamp = 0

  for line in sequence_contents:
    current_time = int(time.time() * 1000) - start_time
    
    items = line.rstrip().split(' ')

    if items[command_index] == "/meta":
      time_stamp = int(items[meta_time_index])
      
      arduinoSerial.write(chr(int(float(items[meta_vpos_index])*255)))
      
      draw.rectangle([oled_status1_pos, oled_status1_poz], outline=0, fill=0)
      draw.text(oled_status1_pos, "Frame:  " + items[frame_index],  font=font, fill=255)
      draw.rectangle([oled_status2_pos, oled_status2_poz], outline=0, fill=0)
      draw.text(oled_status2_pos, "Position: " + items[meta_vpos_index],  font=font, fill=255)
      oled.image(image)  
      oled.display()
      
    elif items[command_index] == "/led_f":
      led_end_index = len(items)-led_start_index
      led_counter = 0;
      for i in range(led_start_index, led_end_index, 3):
        if items[i]:
          red = int(float(items[i]))
          blue = int(float(items[i+1]))
          green = int(float(items[i+2]))
           
          if red == blue == green:
            led_f_w.setPixelColor(led_counter, red)
            led_f_rgb.setPixelColor(led_counter, 0)
          else:
            color = red << 16
            color += green << 8
            color += blue
            led_f_w.setPixelColor(led_counter, 0)
            led_f_rgb.setPixelColor(led_counter, color)
        led_counter += 1
      led_f_w.show()
      led_f_rgb.show()
    elif items[command_index] == "/led_r":
      led_end_index = len(items)-led_start_index
      led_counter = 0;
      for i in range(led_start_index, led_end_index, 3):
        if items[i]:
          red = int(float(items[i]))
          blue = int(float(items[i+1]))
          green = int(float(items[i+2]))
          
          if red == blue == green:
            led_r_w.setPixelColor(led_counter, red)
            led_r_rgb.setPixelColor(led_counter, 0)
          else:
            color = red << 16
            color += green << 8
            color += blue
            led_r_w.setPixelColor(led_counter, 0)
            led_r_rgb.setPixelColor(led_counter, color)
        led_counter += 1
      led_r_w.show()
      led_r_rgb.show()
    elif items[command_index] == "message":
      pass
    else:
      print("Cannot parse line: " + line)
      
  print("Time offset: " + str(int(time.time() * 1000) - start_time - time_stamp))

