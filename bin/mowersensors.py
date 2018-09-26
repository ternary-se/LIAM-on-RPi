import sys,os
import curses
import time
import datetime

import RPi.GPIO as GPIO
import atexit
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
from numpy import median
from collections import deque
import mowerfunctions
from random import *
import syslog

##mower stuff
GPIO.setmode(GPIO.BCM)
def cleanGPIO():
    GPIO.cleanup()
atexit.register(cleanGPIO)
# BWF pins and variables
GPIO.setup(22, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(24, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(25, GPIO.IN, pull_up_down = GPIO.PUD_UP)
SPI_PORT   = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
#wheelload_readings = deque([0, 0, 0, 0, 0])
wheelload_readings = deque([0, 0, 0,])
cutterload_readings = deque([0, 0, 0, 0, 0])

runmode = "manual"
bypasscounter = 0
rplacement = "inside"
lplacement = "inside"

### used variables ###
voltage_readings = deque([0, 0, 0, 0, 0])
wheelload_readings = deque([0, 0, 0,])
cutterload_readings = deque([0, 0, 0, 0, 0])

robot = mowerfunctions.Robot(stop_at_exit = False)

class Sensorread(object):

    global voltage_readings
    global wheelload_readings
    global cutterload_readings
    global robot
    
    def readvoltage(self):
        a0 = mcp.read_adc(0)
        # put a0 first in the list and make a median value for stability
        voltage_readings.rotate(1)
        voltage_readings[0] = a0
        a0_median = median(voltage_readings)
        voltage = (float(a0_median) * 3.2680645161290322580645161290323 * 4.0228013029315960912052117263844) / 1023
        return voltage

    def readcoils(self):
        # read # of hits of BWF coils during 0.1s
        rhitcount = 0
        lhitcount = 0
        bhitcount = 0
        prev_rinput = 0
        prev_linput = 0
        prev_binput = 0
        bwf_start = time.time()
        elapsed_bwf = time.time() - bwf_start
        while (elapsed_bwf <= .10):
          #take a reading
          rinput = GPIO.input(25)
          linput = GPIO.input(24)
          binput = GPIO.input(22)
          if ((not prev_rinput) and rinput):
            rhitcount += 1
          if ((not prev_linput) and linput):
            lhitcount += 1
          if ((not prev_binput) and binput):
            bhitcount += 1
          prev_rinput = rinput
          prev_linput = linput
          prev_binput = binput
          elapsed_bwf = time.time() - bwf_start

        hitcount = [lhitcount, rhitcount, bhitcount]
        return hitcount

    def readmotorcurrent(self):
        wheelload_readings.rotate(1)
        wheelload_readings[0] = mcp.read_adc(2)
        a2_median = median(wheelload_readings)
        return a2_median

    def readcuttercurrent(self):
        cutterload_readings.rotate(1)
        cutterload_readings[0] = mcp.read_adc(4)
        a4_median = median(cutterload_readings)
        return a4_median

### escapes: make shure the robot really stops before changing direction to avoid great currents

    def escapehitrightcoil(self):
        syslog.syslog("Liam-on-RPi: entering escapehitrightcoil")
        #mb robot = mowerfunctions.Robot(stop_at_exit = False)
        robot.stop()
        time.sleep(1.85)
        # read a low value for initialising variable
        wheelload = self.readmotorcurrent()
        wheelload = self.readmotorcurrent()
        wheelload = self.readmotorcurrent()
        wheelload = self.readmotorcurrent()
        #assumes the coast is clear backwards

        #that was dumb robot.backward(255, 1.5)
        #that was dumb robot.stop()
        robot.backward(255)
        fromtime = time.time()
        a = 0
        # loop acts like sleep but does sensor reading while turning
        turntime = 1.5
        while (a == 0):
          timediff = time.time() - fromtime
          # wait a little while before reading motor current to stabilise it
          if (timediff > 0.2):
              wheelload = self.readmotorcurrent()
          coil = self.readcoils()
          # if rear coil passes BWF, times up or we hit something
          if ( 17 <= coil[2] <= 27) or (timediff >= turntime) or (wheelload > 600):
            robot.stop()
            #time.sleep(0.85)
            a = 1
            if (timediff >= turntime):
              returncode = "reverse ok"
            else:
              returncode = "reverse fail"
            syslog.syslog("Liam-on-RPi: escapehitrightcoil reverse, rearcoil = " + str(coil[2]) + ", timediff = " + str(timediff) + "/" + str(turntime) + ", wheel = " + str(wheelload) + " returned " + returncode)

        time.sleep(0.85)
        turntime = random() * 1.35
        #syslog.syslog("Liam-on-RPi: escapehitrightcoil turntime = " + str(turntime))
        robot.left(255)
        fromtime = time.time()
        a = 0
        # loop acts like sleep but does sensor reading while turning
        while (a == 0):
          timediff = time.time() - fromtime
          # wait a little while before reading motor current to stabilise it
          if (timediff > 0.2):
              wheelload = self.readmotorcurrent()
          coil = self.readcoils()
          # if left coil passes BWF, times up or we hit something
          if ( 17 <= coil[0] <= 27) or (timediff >= turntime) or (wheelload > 600):
            robot.stop()
            time.sleep(0.85)
            a = 1
            if (timediff >= turntime):
              returncode = "ok"
            else:
              returncode = "fail"
            syslog.syslog("Liam-on-RPi: escapehitrightcoil obstacle, leftcoil = " + str(coil[0]) + ", timediff = " + str(timediff) + "/" + str(turntime) + ", wheel = " + str(wheelload) + " returned " + returncode)
            return returncode


    def escapehitleftcoil(self):
        syslog.syslog("Liam-on-RPi: entering escapehitleftcoil")
        #mb robot = mowerfunctions.Robot(stop_at_exit = False)
        robot.stop()
        time.sleep(1.85)
        # read a low value for initialising variable
        wheelload = self.readmotorcurrent()
        wheelload = self.readmotorcurrent()
        wheelload = self.readmotorcurrent()
        wheelload = self.readmotorcurrent()
        #assumes the coast is clear backwards
        #that was dumb robot.backward(255, 1.5)
        #robot.stop()
        robot.backward(255)
        fromtime = time.time()
        a = 0
        # loop acts like sleep but does sensor reading while turning
        turntime = 1.5
        while (a == 0):
          timediff = time.time() - fromtime
          # wait a little while before reading motor current to stabilise it
          if (timediff > 0.2):
              wheelload = self.readmotorcurrent()
          coil = self.readcoils()
          # if rear coil passes BWF, times up or we hit something
          if ( 17 <= coil[2] <= 27) or (timediff >= turntime) or (wheelload > 600):
            robot.stop()
            #time.sleep(0.85)
            a = 1
            if (timediff >= turntime):
              returncode = "reverse ok"
            else:
              returncode = "reverse fail"
            syslog.syslog("Liam-on-RPi: escapehitrightcoil reverse, rearcoil = " + str(coil[2]) + ", timediff = " + str(timediff) + "/" + str(turntime) + ", wheel = " + str(wheelload) + " returned " + returncode)

        time.sleep(0.85)
        turntime = random() * 1.35
        robot.right(255)
        fromtime = time.time()
        a = 0
        # loop acts like sleep but does sensor reading while turning
        while (a == 0):
          timediff = time.time() - fromtime
          # wait a little while before reading motor current to stabilise it
          if (timediff > 0.2):
              wheelload = self.readmotorcurrent()
          coil = self.readcoils()
          # if right coil passes BWF, times up or we hit something
          if ( 17 <= coil[1] <= 27) or (timediff >= turntime) or (wheelload > 600):
            robot.stop()
            time.sleep(0.85)
            a = 1
            if (timediff >= turntime):
              returncode = "ok"
            else:
              returncode = "fail"
            syslog.syslog("Liam-on-RPi: escapehitleftcoil obstacle, rightcoil = " + str(coil[1]) + ", timediff = " + str(timediff) + "/" + str(turntime) + ", wheel = " + str(wheelload) + " returned " + returncode)
            return returncode
