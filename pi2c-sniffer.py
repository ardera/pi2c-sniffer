#!/usr/bin/env python3

import time
import pigpio
import math
from functools import reduce
from datetime import datetime

class I2CPacket:
	"""
	Unit of I2C Communication.
	Starts with the START condition and ends with the STOP condition.
	Consists entirely of smaller I2C Parcels, and a timestamp.

	String form of I2C communication:
		"[" is the START-condition. "]" is STOP condition.
		"AA" is a 7-bit slave address, "RR" is a command or register byte, "XX" is a data byte.
		"R" is a read-bit, "W" is a write bit. "+" is ACK, "-" is NACK.

	A "read packet" in this context (queried by isReadPacket()) is structured like this:
		[AAW+RR+[AAR+XX+XX+XX+XX-] (as many XX-es as wanted by the mater, or possible for the slave)
		this essentially means that the master wants to know the contents of slave AA's register RR. The contents are the XX's in the slaves transmission to the master.
	
	A "write packet" in this context (queried by isWritePacket()) is structured like this:
		[AAW+RR+XX+XX+XX+XX+]
		this is done by the master when it wants to write some data to slave AA's register RR.

	I actually have no idea whether it's common for the I2C comm. to be structured in this "read packet" / "write packet" form I described above.
	But I know for sure that the FT54-series of touchscreen controllers uses them.
	"""

	startTimestamp = time.time()

	def __init__(self, timestamp = None):
		self.timestamp = (timestamp if timestamp != None else time.time()) - I2CPacket.startTimestamp
		self.parcels = []

	def addParcel(self, parcel):
		self.parcels.append(parcel)

	def firstParcel(self):
		return self.parcels[0]
	
	def secondParcel(self):
		return self.parcels[1] if len(self.parcels) > 1 else None

	def isWritePacket(self):
		return (len(self.parcels) == 1) and (not self.firstParcel().is_read)
	def isReadPacket(self):
		return (len(self.parcels) == 2) and (not self.firstParcel().is_read) and (self.secondParcel().is_read)

	def getRegister(self):
		return self.firstParcel().data[0]
	def getData(self):
		return self.secondParcel().data if self.isReadPacket() else self.firstParcel().data[1:]

	def __str__(self):
		return "[{:3d}s {:06d}us] {}".format(
			math.floor(self.timestamp),
			math.floor((self.timestamp - math.floor(self.timestamp))*1000000),
			", ".join(map(lambda parcel: str(parcel), self.parcels))
		)

class I2CParcel:
	"""
		Smaller Unit of I2C Communication.
		Starts with a START condition and ends with another START, or a STOP condition.
		Each parcel is either read (from slave to master) or write (from master to slave).
		Each parcel has a 7-bit slave address.
		It's possible for a parcel to have 0 data bytes.
	"""
	def __init__(self, address = 0, is_read = True, data = None):
		self.address = address
		self.is_read = is_read
		self.data = data if data != None else []
	
	def setAddress(self, address = 0):
		self.address = address
	
	def setIsRead(self, is_read = True):
		self.is_read = is_read

	def addDataByte(self, byte):
		self.data.append(byte)
	
	def __str__(self):
		return "{}(0x{:02X}) {}".format(
			"R" if self.is_read else "W",
			self.address,
			", ".join(map(lambda datum: "0x{:02X}".format(datum), self.data))
		)

class I2CSniffer:
	def __init__(self, SCLPin, SDAPin, onPacketCallback=None, set_as_inputs=True, pigpio_instance=None):
		self.shouldStartStopPigpio = False if pigpio_instance != None else True
		self.pigpio_instance = pigpio_instance if pigpio_instance != None else pigpio.pi()
		
		self.SCLPin = SCLPin
		self.SDAPin = SDAPin

		self.FALLING = 0
		self.RISING = 1
		self.STEADY = 2

		self.packet = None
		self.parcel = None
		self.packetHistory = []
		self.onPacketCallback = onPacketCallback

		self.oldSCL = 1
		self.oldSDA = 1
		self.value = 0
		self.nbits = 0

		if set_as_inputs:
			self.pigpio_instance.set_mode(self.SCLPin, pigpio.INPUT)
			self.pigpio_instance.set_mode(self.SDAPin, pigpio.INPUT)

	def __del__(self):
		if self.shouldStartStopPigpio:
			self.pigpio_instance.stop()

	def getPacketHistory(self):
		return self.packetHistory
	
	def getLastestPacket(self):
		return self.packetHistory[len(self.packetHistory)-1]
	
	def getSecondLatestPacket(self):
		return self.packetHistory[len(self.packetHistory)-2]

	def getLatestWithPredicate(self, predicate):
		for packet in reversed(self.packetHistory):
			if predicate(packet):
				return packet

	def _parse(self, SCL, SDA):
		if SCL != self.oldSCL:
			self.oldSCL = SCL
			xSCL = self.RISING if SCL else self.FALLING
		else:
			xSCL = self.STEADY

		if SDA != self.oldSDA:
			self.oldSDA = SDA
			xSDA = self.RISING if SDA else self.FALLING
		else:
			xSDA = self.STEADY


		if (xSCL == self.RISING) and (self.parcel != None):
			self.nbits += 1
			self.value = (self.value << 1) | SDA

			if self.nbits == 7:									# address read
				self.parcel.setAddress(self.value)
				self.value = 0
			elif self.nbits == 8:								# read R/W bit
				self.parcel.setIsRead(self.value == 1)
				self.value = 0
			elif (self.nbits-10) %9 +1 == 8:							# read data byte
				self.parcel.addDataByte(self.value)
				self.value = 0
			elif (self.nbits-10) %9 +1 == 9:							# ACK/NACK bit
				# self.ack = self.value
				self.value = 0

		elif (xSCL == self.STEADY) and (SCL == 1):
			if xSDA == self.FALLING:		# START condition
				if self.packet == None:
					self.packet = I2CPacket()
				else:
					self.packet.addParcel(self.parcel)
				
				self.parcel = I2CParcel()
				self.nbits = 0
				self.value = 0
				
			elif (xSDA == self.RISING) and (self.parcel != None):		# STOP condition
				packet = self.packet
				
				self.packet.addParcel(self.parcel)
				self.parcel = None
				self.packetHistory.append(self.packet)
				self.packet = None
				
				if self.onPacketCallback != None:
					self.onPacketCallback(packet, self)
				
	def _cb(self, gpio, level, tick):
		"""
		Check which line has altered state (ignoring watchdogs) and
		call the parser with the new state.
		"""
		SCL = self.oldSCL
		SDA = self.oldSDA

		if gpio == self.SCLPin:
			SCL = level

		if gpio == self.SDAPin:
			SDA = level

		self._parse(SCL, SDA)

	def start(self):
		self.cbA = self.pigpio_instance.callback(self.SCLPin, pigpio.EITHER_EDGE, self._cb)
		self.cbB = self.pigpio_instance.callback(self.SDAPin, pigpio.EITHER_EDGE, self._cb)

	def stop(self):
		self.cbA.cancel()
		self.cbB.cancel()

	def sniff(self, seconds = 60):
		self.start()
		time.sleep(seconds)
		self.stop()

		return self.getPacketHistory()


lastMovement = None
avgInterval = 0
nSamples = 0
def onPacketCallback(p1, sniffer):
	global lastMovement, avgInterval, nSamples

	print(str(p1))	# print the received packet

	if (p1.isReadPacket()) and (p1.getRegister() == 3):
		p2 = sniffer.getLatestWithPredicate(lambda p: (p != p1) and (p.isReadPacket()) and (p.getRegister() == 3))
		
		if (p2 != None) and (len(p1.getData()) > 0) and (len(p2.getData()) > 0):
			if p1.getData() != p2.getData():
				print("  touch position CHANGED")
				if lastMovement != None:
					print("    time between movements: {:6d}us".format(math.floor((p1.timestamp - lastMovement)*1000000)))
					print("    AVERAGE time between movements: {:6d}us".format(math.floor(avgInterval*1000000)))
				
					avgInterval = (avgInterval*nSamples + (p1.timestamp - lastMovement)) / (nSamples +1)
					nSamples += 1
				lastMovement = p1.timestamp
			else:
				print("  touch position did NOT change")

	dt = p1.timestamp - sniffer.getSecondLatestPacket().timestamp
	print("  dt: {:6d}us".format(math.floor(dt*1000000)))


if __name__ == "__main__":
	I2CSniffer(23, 24, onPacketCallback = onPacketCallback).sniff(10)
