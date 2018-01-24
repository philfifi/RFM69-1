#!/usr/bin/env python2

from RFM69py.RFM69registers import *

import spidev
import OPi.GPIO as GPIO
import time
import queue
from threading import Thread
import logging
logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())

EPSILON = 0.0001
DEFAULT_RETRY_WAIT_TIME_MS = 100

class RFPacket(object):
    def __init__(self, targetID, senderID, CTL, data, rssi=None):
        self.targetID = targetID
        self.senderID = senderID
        self.CTL = CTL
        self.data = data
        self.rssi = rssi

    def isAck(self):
        return len(self.data) == 0 and self.CTL & 0x80

    def requestAck(self):
        return self.CTL & 0x40

    def __str__(self):
        return "From={} To={} RSSI={} isAck={} requestAck={} data={}".format( \
            self.senderID, self.targetID, self.rssi, bool(self.isAck()), bool(self.requestAck()), self.data)


class RFM69AckSender(Thread):
    def __init__(self, q, sendACK):
        Thread.__init__(self)
        self.q = q
        self.sendACK = sendACK
        self.start()

    def run(self):
        logger.info("Ack sender started")
        while True:
            p_ack = self.q.get()
            logger.debug("Sending Ack to ID=%d" % p_ack.senderID)
            self.sendACK(p_ack.senderID)





class RFM69(object):
    def __init__(self, freqBand, nodeID, networkID, isRFM69HW = False, intPin = 18, rstPin = None, spiBus = 0, spiDevice = 0):

        self.freqBand = freqBand
        self.nodeID = nodeID
        self.networkID = networkID
        self.isRFM69HW = isRFM69HW
        self.intPin = intPin
        self.rstPin = rstPin
        self.spiBus = spiBus
        self.spiDevice = spiDevice
        self.intLock = False
        self.mode = ""
        self.promiscuousMode = False
        self.SENDERID = 0
        self.TARGETID = 0
        self.PAYLOADLEN = 0
        self.ACK_REQUESTED = 0
        self.ACK_RECEIVED = 0
        self.RSSI = 0
        self.DATA = []

        self.pk_queue = queue.Queue()
        self.recv_ack_queue = queue.Queue()
        self.send_ack_queue = queue.Queue()

        GPIO.setmode(GPIO.SUNXI)
        GPIO.setup(self.intPin, GPIO.IN)
        GPIO.setup("PG8", GPIO.OUT)
        if self.rstPin is not None:
            GPIO.setup(self.rstPin, GPIO.OUT)

        frfMSB = {RF69_315MHZ: RF_FRFMSB_315, RF69_433MHZ: RF_FRFMSB_433,
                  RF69_868MHZ: RF_FRFMSB_868, RF69_915MHZ: RF_FRFMSB_915}
        frfMID = {RF69_315MHZ: RF_FRFMID_315, RF69_433MHZ: RF_FRFMID_433,
                  RF69_868MHZ: RF_FRFMID_868, RF69_915MHZ: RF_FRFMID_915}
        frfLSB = {RF69_315MHZ: RF_FRFLSB_315, RF69_433MHZ: RF_FRFLSB_433,
                  RF69_868MHZ: RF_FRFLSB_868, RF69_915MHZ: RF_FRFLSB_915}

        self.CONFIG = {
          0x01: [REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY],
          #no shaping
          0x02: [REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00],
          #default:4.8 KBPS
          0x03: [REG_BITRATEMSB, RF_BITRATEMSB_55555],
          0x04: [REG_BITRATELSB, RF_BITRATELSB_55555],
          #default:5khz, (FDEV + BitRate/2 <= 500Khz)
          0x05: [REG_FDEVMSB, RF_FDEVMSB_50000],
          0x06: [REG_FDEVLSB, RF_FDEVLSB_50000],

          0x07: [REG_FRFMSB, frfMSB[freqBand]],
          0x08: [REG_FRFMID, frfMID[freqBand]],
          0x09: [REG_FRFLSB, frfLSB[freqBand]],

          # looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
          # +17dBm and +20dBm are possible on RFM69HW
          # +13dBm formula: Pout=-18+OutputPower (with PA0 or PA1**)
          # +17dBm formula: Pout=-14+OutputPower (with PA1 and PA2)**
          # +20dBm formula: Pout=-11+OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
          #0x11: [REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111],
          #over current protection (default is 95mA)
          #0x13: [REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95],

          # RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4khz)
          #//(BitRate < 2 * RxBw)
          0x19: [REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2],
          #for BR-19200: //* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
          #DIO0 is the only IRQ we're using
          0x25: [REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01],
          #must be set to dBm = (-Sensitivity / 2) - default is 0xE4=228 so -114dBm
          0x28: [REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN],
          # Clear the fifo (p70)
          0x29: [REG_RSSITHRESH, 220],
          #/* 0x2d */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
          0x2e: [REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0],
          #attempt to make this compatible with sync1 byte of RFM12B lib
          0x2f: [REG_SYNCVALUE1, 0x2D],
          #NETWORK ID
          0x30: [REG_SYNCVALUE2, networkID],
          0x37: [REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF |
                RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF],
          #in variable length mode: the max frame size, not used in TX
          0x38: [REG_PAYLOADLENGTH, 66],
          #* 0x39 */ { REG_NODEADRS, nodeID }, //turned off because we're not using address filtering
          #TX on FIFO not empty
          0x3C: [REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE],
          #RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
          0x3d: [REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF],
          #for BR-19200: //* 0x3d */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, //RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
          #* 0x6F */ { REG_TESTDAGC, RF_DAGC_CONTINUOUS }, // run DAGC continuously in RX mode
          # run DAGC continuously in RX mode, recommended default for AfcLowBetaOn=0
          0x6F: [REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0],
          0x00: [255, 0]
        }

        #initialize SPI
        self.spi = spidev.SpiDev()
        self.spi.open(self.spiBus, self.spiDevice)
        self.spi.max_speed_hz = 4000000

        # Hard reset the RFM module
        if self.rstPin is not None:
            GPIO.output(self.rstPin, GPIO.HIGH);
            time.sleep(0.1)
            GPIO.output(self.rstPin, GPIO.LOW);
            time.sleep(0.1)

        #verify chip is syncing?
        logger.debug(">> syncing")
        while self.readReg(REG_SYNCVALUE1) != 0xAA:
            self.writeReg(REG_SYNCVALUE1, 0xAA)

        while self.readReg(REG_SYNCVALUE1) != 0x55:
            self.writeReg(REG_SYNCVALUE1, 0x55)
        logger.debug("<< syncing")

        #write config
        for value in self.CONFIG.values():
            self.writeReg(value[0], value[1])

        self.encrypt(0)
        self.setHighPower(self.isRFM69HW)
        # Wait for ModeReady
        self.setMode(RF69_MODE_STANDBY, waitReady=True)

        self.ack_sender = RFM69AckSender(self.send_ack_queue, self.sendACK)

        GPIO.remove_event_detect(self.intPin)
        GPIO.add_event_detect(self.intPin, GPIO.RISING, callback=self.interruptHandler)

    def setFrequency(self, FRF):
        self.writeReg(REG_FRFMSB, FRF >> 16)
        self.writeReg(REG_FRFMID, FRF >> 8)
        self.writeReg(REG_FRFLSB, FRF)

    def setMode(self, newMode, waitReady=False):
        if newMode == self.mode:
            return

        if newMode == RF69_MODE_TX:
            self.writeReg(REG_OPMODE, (self.readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER)
            if self.isRFM69HW:
                self.setHighPowerRegs(True)
        elif newMode == RF69_MODE_RX:
            self.writeReg(REG_OPMODE, (self.readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER)
            if self.isRFM69HW:
                self.setHighPowerRegs(False)
        elif newMode == RF69_MODE_SYNTH:
            self.writeReg(REG_OPMODE, (self.readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER)
        elif newMode == RF69_MODE_STANDBY:
            self.writeReg(REG_OPMODE, (self.readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY)
        elif newMode == RF69_MODE_SLEEP:
            self.writeReg(REG_OPMODE, (self.readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP)
        else:
            return

        # we are using packet mode, so this check is not really needed
        # but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
        if self.mode == RF69_MODE_SLEEP or waitReady:
            while self.readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY == 0x00:
                time.sleep(EPSILON)

        self.mode = newMode;

    def sleep(self):
        self.setMode(RF69_MODE_SLEEP)

    def setAddress(self, addr):
        self.nodeID = addr
        self.writeReg(REG_NODEADRS, self.nodeID)

    def setNetwork(self, networkID):
        self.networkID = networkID
        self.writeReg(REG_SYNCVALUE2, networkID)

    def setPowerLevel(self, powerLevel):
        if powerLevel > 31:
            powerLevel = 31
        self.powerLevel = powerLevel
        self.writeReg(REG_PALEVEL, (self.readReg(REG_PALEVEL) & 0xE0) | self.powerLevel)

    def canSend(self):
        if self.intLock:
            logger.debug("In interrupt")
            return False
        if self.mode == RF69_MODE_STANDBY:
            logger.debug("In mode STANDBY")
            return True
        #if signal stronger than -100dBm is detected assume channel activity
        irqflags1 = self.getIrqFlags1()
        if self.mode == RF69_MODE_RX:
            if not irqflags1 & RF_IRQFLAGS1_MODEREADY:
                logger.debug("Mode NOT ready")
                return False
            rssi = self.readRSSI()
            if rssi > CSMA_LIMIT:
                logger.debug("Rssi %d > limit" % rssi)
                return False
            logger.debug("RX mode and rssi ok")
            return True
        if irqflags1 & RF_IRQFLAGS1_AUTOMODE:
            logger.debug("In intermediate mode")
            return False
        logger.debug("Not in a good mode")
        return False

    def waitCanSend(self):
        "Return true till we can send, False if reached the timeout"
        now = time.time()
        while True:
            if self.canSend():
                return True
            if time.time() - now > RF69_CSMA_LIMIT_S:
                return False
            time.sleep(EPSILON)

    def send(self, toAddress, buff = "", requestACK = False, retryWaitTime=DEFAULT_RETRY_WAIT_TIME_MS):
        self.writeReg(REG_PACKETCONFIG2, (self.readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART)
        if not self.waitCanSend():
            return False
        good = self.sendFrame(toAddress, buff, requestACK, sendACK=False)
        if requestACK:
            # Wait for the Ack
            try:
                p = self.recv_ack_queue.get(timeout = retryWaitTime/1000.)
            except queue.Empty:
                good = False
            else:
                if p.senderID == toAddress:
                    good = True
        return good

#    to increase the chance of getting a packet across, call this function instead of send
#    and it handles all the ACK requesting/retrying for you :)
#    The only twist is that you have to manually listen to ACK requests on the other side and send back the ACKs
#    The reason for the semi-automaton is that the lib is ingterrupt driven and
#    requires user action to read the received data and decide what to do with it
#    replies usually take only 5-8ms at 50kbps@915Mhz

    def sendWithRetry(self, toAddress, buff = "", retries = 3, retryWaitTime=DEFAULT_RETRY_WAIT_TIME_MS):
        good = False
        for i in range(0, retries):
            good = self.send(toAddress, buff, requestACK=True, retryWaitTime=retryWaitTime)
            if good:
                break
            else:
                logger.info("Retring !")
        return good

    def sendACK(self, toAddress = 0, buff=[]):
        toAddress = toAddress if toAddress > 0 else self.SENDERID
        while not self.canSend():
            time.sleep(EPSILON)
        self.sendFrame(toAddress, buff, requestACK=False, sendACK=True)

    def sendFrame(self, toAddress, buff, requestACK, sendACK):
        #turn off receiver to prevent reception while filling fifo
        self.setMode(RF69_MODE_STANDBY, waitReady=True)

        buff = buff[:RF69_MAX_DATA_LEN]

        ack = 0
        if sendACK:
            ack = 0x80
        elif requestACK:
            ack = 0x40

        data = [len(buff) + 3,
                toAddress,
                self.nodeID,
                ack] + buff

        # DIO0 is "Packet Sent"
#        self.writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00)

        self.setAutoMode(RF_AUTOMODES_ENTER_FIFOLEVEL, RF_AUTOMODES_EXIT_PACKETSENT, RF_AUTOMODES_INTERMEDIATE_TRANSMITTER);

        self.setFifoThreshold(RF_FIFOTHRESH_TXSTART_FIFOTHRESH, len(data)-1)
        self.writeFIFO(data)

        # Quickly go to RX mode
        n = time.time()
        while True:
            cur_mode = self.readReg(REG_OPMODE) & (0x03<<2)
            if cur_mode == RF_OPMODE_TRANSMITTER:
                # With automode, will go in RX directly after transmission
                break
            elif cur_mode == RF_OPMODE_STANDBY:
                # Either the transmission just completes, or not started yet
                if time.time() -n > 0.001:
                    break
            else:
                assert False, "Bad mode !" + hex(cur_mode)
            time.sleep(EPSILON)
        self.setMode(RF69_MODE_RX)

        # Wait to exit the intermediate mode
        n = time.time()
        good = False
        while (time.time() - n) < .2:
            if not (self.getIrqFlags1() & RF_IRQFLAGS1_AUTOMODE):
                good = True
                break
            time.sleep(EPSILON)

        # And disable auto mode
        self.setAutoMode(RF_AUTOMODES_ENTER_OFF, RF_AUTOMODES_EXIT_OFF, RF_AUTOMODES_INTERMEDIATE_SLEEP);
        return good


    def writeFIFO(self, buff):
        logger.debug([REG_FIFO | 0x80] + buff)
        self.spi.xfer2([REG_FIFO | 0x80] + buff)

    def fetch_packet(self):
        PAYLOADLEN, TARGETID, SENDERID, CTLbyte = self.spi.xfer2([REG_FIFO & 0x7f,0,0,0,0])[1:]
        if PAYLOADLEN > 66:
            PAYLOADLEN = 66
        DATALEN = PAYLOADLEN -3

        if DATALEN > 0:
            DATA = self.spi.xfer2([REG_FIFO & 0x7f] + [0 for i in range(0, DATALEN)])[1:]
        else:
            DATA = []

        return RFPacket(TARGETID, SENDERID, CTLbyte, DATA)

    def getRssiIrqFlags(self):
        addrs = [REG_RSSIVALUE, REG_DIOMAPPING1, REG_DIOMAPPING2,
                     REG_IRQFLAGS1, REG_IRQFLAGS2]
        rssi, __, __,irqflags1, irqflags2 = self.spi.xfer(addrs + [0])[1:]
#        logger.debug("irqflags1: 0x%02x  %s" % (irqflags1, bin(irqflags1)))
#        logger.debug("irqflags2: 0x%02x  %s" % (irqflags2, bin(irqflags2)))
        rssi = (rssi*-1) >> 1
        return rssi, irqflags1, irqflags2

    def getIrqFlags1(self):
        irqflags1 = self.readReg(REG_IRQFLAGS1)
#        logger.debug("irqflags1: 0x%02x  %s" % (irqflags1, bin(irqflags1)))
        return irqflags1

    def getIrqFlags2(self):
        irqflags2 = self.readReg(REG_IRQFLAGS2)
#        logger.debug("irqflags2: 0x%02x" % irqflags2)
        return irqflags2

    def interruptHandler(self, pin):
        # Can be called either at the start of a transmission,
        # Or at the end of a reception
        GPIO.output("PG8", GPIO.HIGH)
        self.intLock = True
        rssi, irqflags1, irqflags2 = self.getRssiIrqFlags()
        if self.mode == RF69_MODE_RX and irqflags2 & RF_IRQFLAGS2_PAYLOADREADY:
            p = self.fetch_packet()
            p.rssi = rssi
            logger.info(p)
            if p.isAck():
                self.recv_ack_queue.put(p)
            else:
                if p.targetID in [self.nodeID, RF69_BROADCAST_ADDR] or self.promiscuousMode:
                    self.pk_queue.put(p)
                    if p.requestAck() and p.targetID == self.nodeID:
                        self.send_ack_queue.put(p)
        else:
            logger.info("No packet to fetch in the interrupt")

        GPIO.output("PG8", GPIO.LOW)
        self.intLock = False

    def receiveBegin(self):
        logger.debug("receiveBegin")
        while self.intLock:
            time.sleep(.1)
        self.SENDERID = 0
        self.TARGETID = 0
        self.PAYLOADLEN = 0
        self.ACK_REQUESTED = 0
        self.ACK_RECEIVED = 0
        self.RSSI = 0
        if (self.readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY):
            # avoid RX deadlocks
            self.writeReg(REG_PACKETCONFIG2, (self.readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART)
        #set DIO0 to "PAYLOADREADY" in receive mode
        self.writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01)
        self.setMode(RF69_MODE_RX, waitReady=True)

    def receiveDone(self):
        if (self.mode == RF69_MODE_RX or self.mode == RF69_MODE_STANDBY) and self.PAYLOADLEN > 0:
            self.setMode(RF69_MODE_STANDBY)
            return True
        if self.readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_TIMEOUT:
            # https://github.com/russss/rfm69-python/blob/master/rfm69/rfm69.py#L112
            # Russss figured out that if you leave alone long enough it times out
            # tell it to stop being silly and listen for more packets
            self.writeReg(REG_PACKETCONFIG2, (self.readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART)
        elif self.mode == RF69_MODE_RX:
            # already in RX no payload yet
            return False
        self.receiveBegin()
        return False

    def readRSSI(self, forceTrigger = False):
        rssi = 0
        if forceTrigger:
            self.writeReg(REG_RSSICONFIG, RF_RSSI_START)
            while self.readReg(REG_RSSICONFIG) & RF_RSSI_DONE == 0x00:
                time.sleep(EPSILON)
        rssi = self.readReg(REG_RSSIVALUE) * -1
        rssi = rssi >> 1
        return rssi

    def encrypt(self, key):
        self.setMode(RF69_MODE_STANDBY)
        if key != 0 and len(key) == 16:
            self.spi.xfer([REG_AESKEY1 | 0x80] + [int(ord(i)) for i in list(key)])
            self.writeReg(REG_PACKETCONFIG2,(self.readReg(REG_PACKETCONFIG2) & 0xFE) | RF_PACKET2_AES_ON)
        else:
            self.writeReg(REG_PACKETCONFIG2,(self.readReg(REG_PACKETCONFIG2) & 0xFE) | RF_PACKET2_AES_OFF)

    def readReg(self, addr):
        return self.spi.xfer([addr & 0x7F, 0])[1]

    def writeReg(self, addr, value):
        self.spi.xfer([addr | 0x80, value])

    def promiscuous(self, onOff):
        self.promiscuousMode = onOff

    def setHighPower(self, onOff):
        if onOff:
            self.writeReg(REG_OCP, RF_OCP_OFF)
            #enable P1 & P2 amplifier stages
            self.writeReg(REG_PALEVEL, (self.readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON)
        else:
            self.writeReg(REG_OCP, RF_OCP_ON)
            #enable P0 only
            self.writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | powerLevel)

    def setFifoThreshold(self, condition, threshold):
        self.writeReg(REG_FIFOTHRESH, condition + threshold%128)

    def setHighPowerRegs(self, onOff):
        if onOff:
            self.writeReg(REG_TESTPA1, 0x5D)
            self.writeReg(REG_TESTPA2, 0x7C)
        else:
            self.writeReg(REG_TESTPA1, 0x55)
            self.writeReg(REG_TESTPA2, 0x70)

    def setAutoMode(self, enter, exit, intermediate):
        self.writeReg(REG_AUTOMODES, enter+exit+intermediate)

    def readAllRegs(self):
        results = []
        for address in range(1, 0x50):
            results.append([str(hex(address)), str(bin(self.readReg(address)))])
        return results

    def readTemperature(self, calFactor):
        self.setMode(RF69_MODE_STANDBY)
        self.writeReg(REG_TEMP1, RF_TEMP1_MEAS_START)
        while self.readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING:
            time.sleep(EPSILON)
        # COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction
        #'complement'corrects the slope, rising temp = rising val
        return (int(~self.readReg(REG_TEMP2)) * -1) + COURSE_TEMP_COEF + calFactor


    def rcCalibration(self):
        self.writeReg(REG_OSC1, RF_OSC1_RCCAL_START)
        while self.readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE == 0x00:
            time.sleep(EPSILON)

    def shutdown(self):
        self.setHighPower(False)
        self.sleep()
        GPIO.cleanup()
