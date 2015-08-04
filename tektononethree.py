#!/usr/bin/python

import time
from dotstar import Adafruit_DotStar

frame_index = 0
command_index = 1
time_index = 2
led_start_index = 2

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

led_r_rgb.begin()           # Initialize pins for output
led_r_rgb.setBrightness(64) # Limit brightness to ~1/4 duty cycle

led_r_w.begin()          
led_r_w.setBrightness(64)

led_f_w.begin()          
led_f_w.setBrightness(64)

sequence_file = open('tekton_sequence.txt', 'r')

led_f_rgb.show()
led_r_rgb.show()
led_f_w.show()
led_r_w.show()

while True:

  for line in sequence_file:
    items = line.rstrip().split(',')
    time_stamp = items[time_index]
    #vert_pos = items[2]
    led_end_index = len(items)-led_start_index
    
    led_counter = 0;
    
    if items[0] != "preset":
      if items[command_index] == "meta":
        pass
      elif items[command_index] == "led_f":
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
      elif items[command_index] == "led_r":
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
      else:
        print("Cannot parse line: " + line)
    
  sequence_file.seek(0)

