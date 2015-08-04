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

# DRIVE
import RPi.GPIO as GPIO

### OLED

oled = Adafruit_SSD1306.SSD1306_128_64(rst=24, dc=23, spi=SPI.SpiDev(0, 0, max_speed_hz=8000000))
oled.begin()
oled.clear()
oled.display()

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

### DRIVE

GPIO.setmode(GPIO.BCM)
drive_fault_pin = 26 # 37 Header 
drive_enable_pin = 13 # 33
drive_inpos_pin = 19 # 35 
drive_step_pin = 5 # 29
drive_dir_pin = 6 # 31
prox_sensor_pin = 21 # 40

#GPIO.setwarnings(False)

GPIO.setup(drive_fault_pin, GPIO.IN)
GPIO.setup(drive_enable_pin, GPIO.OUT)
GPIO.setup(drive_inpos_pin, GPIO.IN)
GPIO.setup(drive_step_pin, GPIO.OUT)
GPIO.setup(drive_dir_pin, GPIO.OUT)
GPIO.setup(prox_sensor_pin, GPIO.IN)

drive_steps_max = 20000
drive_steps = 0
drive_dir   = True
drive_pulse = False

### LED

frame_index = 0
command_index = 1
time_index = 2
vpos_index = 3
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

for strip in [led_r_rgb, led_r_w, led_f_rgb, led_f_w]:
  strip.begin()           # Initialize pins for output
  strip.setBrightness(64) # Limit brightness to ~1/4 duty cycle

  for led_index in range(0, numpixels_F):
    strip.setPixelColor(led_index, 0x0)

  strip.show()

sequence_file = open('tekton_sequence.txt', 'r')

while True:

  for line in sequence_file:
    items = line.rstrip().split(',')
    time_stamp = items[time_index]
    led_end_index = len(items)-led_start_index
    
    led_counter = 0;
    
    if items[0] != "preset":
      if items[command_index] == "meta":
        time_stamp = int(items[time_index])
        
        new_steps = int(items[vpos_index] * drive_steps_max)
        while new_steps != drive_steps:
          if drive_dir != (new_steps > drive_steps):
            drive_dir = (new_steps > drive_steps)
            GPIO.output(drive_dir_pin, drive_dir)
            time.sleep(0.00001)
            continue
          GPIO.output(drive_step_pin, True)
          time.sleep(0.00001)
          GPIO.output(drive_step_pin, False)
          if drive_dir: 
            drive_steps += 1
          else:
            drive_steps += -1
          
        draw.rectangle([oled_status1_pos, oled_status1_poz], outline=0, fill=0)
        draw.text(oled_status1_pos, "Frame:  " + items[frame_index],  font=font, fill=255)
        draw.rectangle([oled_status2_pos, oled_status2_poz], outline=0, fill=0)
        draw.text(oled_status2_pos, "Position: " + items[vpos_index],  font=font, fill=255)
        oled.image(image)  
        oled.display()
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

