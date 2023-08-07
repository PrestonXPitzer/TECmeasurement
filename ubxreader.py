"""
pygnssutils - gnssapp.py

*** FOR ILLUSTRATION ONLY - NOT FOR PRODUCTION USE ***

Skeleton GNSS application which continuously receives, parses and prints
NMEA, UBX or RTCM data from a receiver until the stop Event is set or
stop() method invoked. Assumes receiver is connected via serial USB or UART1 port.

The app also implements basic methods needed by certain pygnssutils classes.

Optional keyword arguments:

- sendqueue - any data placed on this Queue will be sent to the receiver
  (e.g. UBX commands/polls or NTRIP RTCM data). Data must be a tuple of 
  (raw_data, parsed_data).
- idonly - determines whether the app prints out the entire parsed message,
  or just the message identity.
- enableubx - suppresses NMEA receiver output and substitutes a minimum set
  of UBX messages instead (NAV-PVT, NAV-SAT, NAV-DOP, RXM-RTCM).
- showhacc - show estimate of horizonal accuracy in metres (if available).

Created on 27 Jul 2023

:author: semuadmin
:copyright: SEMU Consulting © 2023
:license: BSD 3-Clause


Modified:

:P.Pitzer 2023-08-01:
    -modified skeleton to read RAWX messages and calculate TEC
"""
# pylint: disable=invalid-name, too-many-instance-attributes

from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser
from queue import Empty, Queue
from threading import Event, Thread
from time import sleep

import datetime as dt
import matplotlib.pyplot as plt

from pynmeagps import NMEAMessageError, NMEAParseError
from pyrtcm import RTCMMessage, RTCMMessageError, RTCMParseError
from serial import Serial

import csv

from pyubx2 import (
    NMEA_PROTOCOL,
    RTCM3_PROTOCOL,
    UBX_PROTOCOL,
    UBXMessage,
    UBXMessageError,
    UBXParseError,
    UBXReader,
    SET
)

CONNECTED = 1
data = [] 
times = []
def findMatchers(gnssIdBlock, svIdBlock):
    for i in range(len(gnssIdBlock)):
        for j in range(len(gnssIdBlock)):
            if gnssIdBlock[i] != gnssIdBlock[j] and svIdBlock[i] == svIdBlock[j]:   
                return i,j
    return None, None 

def determineFrequency(gnssId, sigId):
    if gnssId == 0 and sigId == 0:
        return 1575.42e6 # L1C/A
    elif gnssId == 0 and sigId == 3:
        return 1227.6e6 # L2CL
    elif gnssId == 0 and sigId == 4:
        return 1227.6e6 # L2CM
    elif gnssId == 0 and sigId == 6:
        return 1176.45e6 # L5 I
    elif gnssId == 0 and sigId == 7:
        return 1176.45e6 #L5 Q
    elif gnssId == 1 and sigId == 0:
        return 1575.42e6 # SBAS L1C/A
    elif gnssId == 2 and sigId == 0:
        return 1575.42e6 # GALI E1 C
    elif gnssId == 2 and sigId == 1:
        return 1207.14e6 # E1 B
    elif gnssId == 2 and sigId == 3:
        return 1176.45e6 # E5a 
    elif gnssId == 2 and sigId == 4:
        return 1176.45e6 #E5a
    elif gnssId == 2 and sigId == 5:
        return 1207.14e6 # E5b
    elif gnssId == 2 and sigId == 6:
        return 1207.14e6 # E5b
    elif gnssId == 3 and sigId == 0:
        return 1561.091e6 # B1I D1
    elif gnssId == 3 and sigId == 1:
        return 1561.091e6 # B1I D2
    elif gnssId == 3 and sigId == 2:
        return 1207.14e6 # B2I D1
    elif gnssId == 3 and sigId == 3:
        return 1207.14e6 # B2I D2
    elif gnssId == 3 and sigId == 5:
        return 1575.42e6 # B1C
    elif gnssId == 3 and sigId == 7:
        return 1176.45e6 #B2a
    elif gnssId == 5 and sigId == 0:
        return 1575.42e6 # QZSS L1C/A 
    elif gnssId == 5 and sigId == 1:
        return 1575.42e6 # QZSS L1S
    elif gnssId == 5 and sigId == 4:
        return 1227.6e6 # QZSS L2 CM
    elif gnssId == 6 and sigId == 0:
        return 1598.0625e6 #GLONASS L1
    elif gnssId == 6 and sigId == 2:
        return 1242.9375e6 #GLONASS L2
    elif gnssId == 7 and sigId == 0:
        return 1176.45e6 #NAVIC L5


        

def calc_tec(f1:float, f2:float, pseudo1:float, pseudo2:float) -> float:
    """
    Calculates the TEC value using eqn 6 from that one paper
    """
    term1:float = (1/(40.3))
    term2:float = (f1*f2)/(f1-f2)
    term3:float = (pseudo2 - pseudo1)
    return abs(term1*term2*term3)

def time_conversion(TOW, WN, leap_seconds):
    """
    converts gps time to utc time
    """
    epoch = dt.datetime(1980, 1, 6, 0, 0, 0)
    elapsed = dt.timedelta(seconds=(TOW+leap_seconds), weeks = WN)
    return epoch + elapsed




class GNSSSkeletonApp:
    """
    Skeleton GNSS application which communicates with a GNSS receiver.
    """

    def __init__(
        self, port: str, baudrate: int, timeout: float, stopevent: Event, **kwargs
    ):
        """
        Constructor.

        :param str port: serial port e.g. "/dev/ttyACM1"
        :param int baudrate: baudrate
        :param float timeout: serial timeout in seconds
        :param Event stopevent: stop event
        """

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.stopevent = stopevent
        self.sendqueue = kwargs.get("sendqueue", None)
        self.idonly = kwargs.get("idonly", True)
        self.enableubx = kwargs.get("enableubx", False)
        self.showhacc = kwargs.get("showhacc", False)
        self.stream = None
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.sep = 0

    def __enter__(self):
        """
        Context manager enter routine.
        """

        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        """
        Context manager exit routine.

        Terminates app in an orderly fashion.
        """

        self.stop()

    def run(self):
        """
        Run GNSS reader/writer.
        """

        self.enable_ubx(self.enableubx)

        self.stream = Serial(self.port, self.baudrate, timeout=self.timeout)
        self.stopevent.clear()

        read_thread = Thread(
            target=self._read_loop,
            args=(
                self.stream,
                self.stopevent,
                self.sendqueue,
            ),
            daemon=True,
        )
        read_thread.start()

    def stop(self):
        """
        Stop GNSS reader/writer.
        """

        self.stopevent.set()
        if self.stream is not None:
            self.stream.close()

    def _read_loop(self, stream: Serial, stopevent: Event, sendqueue: Queue):
        """
        THREADED
        Reads and parses incoming GNSS data from the receiver,
        and sends any queued output data to the receiver.

        :param Serial stream: serial stream
        :param Event stopevent: stop event
        :param Queue sendqueue: queue for messages to send to receiver
        """
        global data
        global times

        ubr = UBXReader(
            stream, protfilter=(NMEA_PROTOCOL | UBX_PROTOCOL | RTCM3_PROTOCOL)
        )
        while not stopevent.is_set():
            try:
                if stream.in_waiting:
                    _, parsed_data = ubr.read()
                    if parsed_data:
                        # extract current navigation solution
                        self._extract_coordinates(parsed_data)

                        # if it's an RXM-RTCM message, show which RTCM3 message
                        # it's acknowledging and whether it's been used or not.""
                        if parsed_data.identity == "RXM-RTCM":
                            nty = (
                                f" - {parsed_data.msgType} "
                                f"{'Used' if parsed_data.msgUsed > 0 else 'Not used'}"
                            )
                        else:
                            nty = ""

                        if self.idonly:
                            print(f"GNSS>> {parsed_data.identity}{nty}")
                        else:
                            if parsed_data.identity == 'RXM-RAWX':
                                #createa a list of the pseudorange measurements up to 32
                                try:
                                    psuedorangeBlock = [parsed_data.prMes_01, parsed_data.prMes_02, parsed_data.prMes_03, parsed_data.prMes_04, 
                                                    parsed_data.prMes_05, parsed_data.prMes_06, parsed_data.prMes_07, parsed_data.prMes_08, 
                                                    parsed_data.prMes_09, parsed_data.prMes_10, parsed_data.prMes_11, parsed_data.prMes_12,
                                                    parsed_data.prMes_13, parsed_data.prMes_14, parsed_data.prMes_15, parsed_data.prMes_16,
                                                    parsed_data.prMes_17, parsed_data.prMes_18, parsed_data.prMes_19, parsed_data.prMes_20,
                                                    parsed_data.prMes_21, parsed_data.prMes_22, parsed_data.prMes_23, parsed_data.prMes_24,
                                                    parsed_data.prMes_25, parsed_data.prMes_26, parsed_data.prMes_27, parsed_data.prMes_28,
                                                    parsed_data.prMes_29, parsed_data.prMes_30, parsed_data.prMes_31, parsed_data.prMes_32]
                                except AttributeError:
                                    continue
                                #do the same thing for gnssID
                                try:

                                    gnssIdBlock = [parsed_data.gnssId_01, parsed_data.gnssId_02, parsed_data.gnssId_03, parsed_data.gnssId_04, 
                                               parsed_data.gnssId_05, parsed_data.gnssId_06, parsed_data.gnssId_07, parsed_data.gnssId_08, 
                                               parsed_data.gnssId_09, parsed_data.gnssId_10, parsed_data.gnssId_11, parsed_data.gnssId_12,
                                               parsed_data.gnssId_13, parsed_data.gnssId_14, parsed_data.gnssId_15, parsed_data.gnssId_16,
                                               parsed_data.gnssId_17, parsed_data.gnssId_18, parsed_data.gnssId_19, parsed_data.gnssId_20,
                                               parsed_data.gnssId_21, parsed_data.gnssId_22, parsed_data.gnssId_23, parsed_data.gnssId_24,
                                               parsed_data.gnssId_25, parsed_data.gnssId_26, parsed_data.gnssId_27, parsed_data.gnssId_28,
                                               parsed_data.gnssId_29, parsed_data.gnssId_30, parsed_data.gnssId_31, parsed_data.gnssId_32]
                                except AttributeError:
                                    continue
                                #and svId
                                try:
                                    svIdBlock = [parsed_data.svId_01, parsed_data.svId_02, parsed_data.svId_03, parsed_data.svId_04, 
                                             parsed_data.svId_05, parsed_data.svId_06, parsed_data.svId_07, parsed_data.svId_08, 
                                             parsed_data.svId_09, parsed_data.svId_10, parsed_data.svId_11, parsed_data.svId_12,
                                             parsed_data.svId_13, parsed_data.svId_14, parsed_data.svId_15, parsed_data.svId_16,
                                             parsed_data.svId_17, parsed_data.svId_18, parsed_data.svId_19, parsed_data.svId_20,
                                             parsed_data.svId_21, parsed_data.svId_22, parsed_data.svId_23, parsed_data.svId_24,
                                             parsed_data.svId_25, parsed_data.svId_26, parsed_data.svId_27, parsed_data.svId_28,
                                             parsed_data.svId_29, parsed_data.svId_30, parsed_data.svId_31, parsed_data.svId_32]
                                except AttributeError:
                                    continue
                                try:
                                    doMesBlock = [parsed_data.doMes_01, parsed_data.doMes_02, parsed_data.doMes_03, parsed_data.doMes_04, 
                                              parsed_data.doMes_05, parsed_data.doMes_06, parsed_data.doMes_07, parsed_data.doMes_08, 
                                              parsed_data.doMes_09,parsed_data.doMes_10, parsed_data.doMes_11, parsed_data.doMes_12,
                                                parsed_data.doMes_13, parsed_data.doMes_14, parsed_data.doMes_15, parsed_data.doMes_16,
                                                parsed_data.doMes_17, parsed_data.doMes_18, parsed_data.doMes_19, parsed_data.doMes_20,
                                                parsed_data.doMes_21, parsed_data.doMes_22, parsed_data.doMes_23, parsed_data.doMes_24,
                                                parsed_data.doMes_25, parsed_data.doMes_26, parsed_data.doMes_27, parsed_data.doMes_28,
                                                parsed_data.doMes_29, parsed_data.doMes_30, parsed_data.doMes_31, parsed_data.doMes_32]
                                except AttributeError:
                                    continue
                                #sigId hopium that it is here
                                try:
                                    sigIdBlock = [parsed_data.sigId_01, parsed_data.sigId_02, parsed_data.sigId_03, parsed_data.sigId_04, 
                                              parsed_data.sigId_05, parsed_data.sigId_06, parsed_data.sigId_07, parsed_data.sigId_08, 
                                              parsed_data.sigId_09, parsed_data.sigId_10, parsed_data.sigId_11, parsed_data.sigId_12,
                                                parsed_data.sigId_13, parsed_data.sigId_14, parsed_data.sigId_15, parsed_data.sigId_16,
                                                parsed_data.sigId_17, parsed_data.sigId_18, parsed_data.sigId_19, parsed_data.sigId_20,
                                                parsed_data.sigId_21, parsed_data.sigId_22, parsed_data.sigId_23, parsed_data.sigId_24,
                                                parsed_data.sigId_25, parsed_data.sigId_26, parsed_data.sigId_27, parsed_data.sigId_28,
                                                parsed_data.sigId_29, parsed_data.sigId_30, parsed_data.sigId_31, parsed_data.sigId_32]
                                except AttributeError:
                                    continue

                                #find two indicies where gnssId is different and svId is the same
                                i,j = findMatchers(gnssIdBlock, svIdBlock)
                                if i is not None and j is not None:
                                    f1 = determineFrequency(gnssIdBlock[i], sigIdBlock[i]) + doMesBlock[i]
                                    f2 = determineFrequency(gnssIdBlock[j], sigIdBlock[j]) + doMesBlock[j]


                                    tec = calc_tec(f1,f2,psuedorangeBlock[i], psuedorangeBlock[j])
                                    print("TEC", tec)
                                    data.append(tec)
                                    time = time_conversion(parsed_data.rcvTow,parsed_data.week, parsed_data.leapS)
                                    times.append(time)

                # send any queued output data to receiver
                self._send_data(ubr.datastream, sendqueue)

            except (
                UBXMessageError,
                UBXParseError,
                NMEAMessageError,
                NMEAParseError,
                RTCMMessageError,
                RTCMParseError,
            ) as err:
                print(f"Error parsing data stream {err}")
                continue

    def _extract_coordinates(self, parsed_data: object):
        """
        Extract current navigation solution from NMEA or UBX message.

        :param object parsed_data: parsed NMEA or UBX navigation message
        """

        if hasattr(parsed_data, "lat"):
            self.lat = parsed_data.lat
        if hasattr(parsed_data, "lon"):
            self.lon = parsed_data.lon
        if hasattr(parsed_data, "alt"):
            self.alt = parsed_data.alt
        if hasattr(parsed_data, "hMSL"):  # UBX hMSL is in mm
            self.alt = parsed_data.hMSL / 1000
        if hasattr(parsed_data, "sep"):
            self.sep = parsed_data.sep
        if hasattr(parsed_data, "hMSL") and hasattr(parsed_data, "height"):
            self.sep = (parsed_data.height - parsed_data.hMSL) / 1000
        if self.showhacc and hasattr(parsed_data, "hAcc"):  # UBX hAcc is in mm
            unit = 1 if parsed_data.identity == "PUBX00" else 1000
            #print(f"Estimated horizontal accuracy: {(parsed_data.hAcc / unit):.3f} m")

    def _send_data(self, stream: Serial, sendqueue: Queue):
        """
        Send any queued output data to receiver.
        Queue data is tuple of (raw_data, parsed_data).

        :param Serial stream: serial stream
        :param Queue sendqueue: queue for messages to send to receiver
        """

        if sendqueue is not None:
            try:
                while not sendqueue.empty():
                    data = sendqueue.get(False)
                    raw, parsed = data
                    source = "NTRIP>>" if isinstance(parsed, RTCMMessage) else "GNSS<<"
                    if self.idonly:
                        print(f"{source} {parsed.identity}")
                    else:
                        print(parsed)
                    stream.write(raw)
                    sendqueue.task_done()
            except Empty:
                pass

    def enable_ubx(self, enable: bool):
        """
        Enable UBX output and suppress NMEA.

        :param bool enable: enable UBX and suppress NMEA output
        """

        layers = 1
        transaction = 0
        cfg_data = []
        for port_type in ("USB", "UART1"):
            cfg_data.append((f"CFG_{port_type}OUTPROT_NMEA", not enable))
            cfg_data.append((f"CFG_{port_type}OUTPROT_UBX", enable))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_PVT_{port_type}", enable))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_SAT_{port_type}", enable * 4))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_DOP_{port_type}", enable * 4))
            cfg_data.append((f"CFG_MSGOUT_UBX_RXM_RTCM_{port_type}", enable))

        msg = UBXMessage.config_set(layers, transaction, cfg_data)
        self.sendqueue.put((msg.serialize(), msg))

    def get_coordinates(self) -> tuple:
        """
        Return current receiver navigation solution.
        (method needed by certain pygnssutils classes)

        :return: tuple of (connection status, lat, lon, alt and sep)
        :rtype: tuple
        """

        return (CONNECTED, self.lat, self.lon, self.alt, self.sep)

    def set_event(self, eventtype: str):
        """
        Create event.
        (stub method needed by certain pygnssutils classes)

        :param str eventtype: name of event to create
        """

        # create event of specified eventtype


if __name__ == "__main__":
    # send a payload to the reciever
    # B5 62 06 8A 09 00 01 01 00 00 A7 02 91 20 01 F6 79
    msg2 = UBXReader.parse(b"\xb5\x62\x06\x8a\x09\x00\x01\x01\x00\x00\xa7\x02\x91\x20\x01\xf6\x79", msgmode=SET)
    arp = ArgumentParser(
        formatter_class=ArgumentDefaultsHelpFormatter,
    )
    arp.add_argument(
        "-P", "--port", required=False, help="Serial port", default="COM8"
    )
    arp.add_argument(
        "-B", "--baudrate", required=False, help="Baud rate", default=38400, type=int
    )
    arp.add_argument(
        "-T", "--timeout", required=False, help="Timeout in secs", default=3, type=float
    )

    args = arp.parse_args()
    send_queue = Queue()
    stop_event = Event()

    try:
        print("Starting GNSS reader/writer...\n")
        with GNSSSkeletonApp(
            args.port,
            int(args.baudrate),
            float(args.timeout),
            stop_event,
            sendqueue=send_queue,
            idonly=False,
            enableubx=True,
            showhacc=True,
        ) as gna:
            gna.run()
            
            while True:
                sleep(1)

    except KeyboardInterrupt:
        stop_event.set()
        print("Terminated by user")

        #once the threads are finished, create a plot of the data and display/save it
        for item in range(len(data)):
            data[item] = data[item] / 10**16
        plt.plot(times, data, color = 'blue')
        plt.xlabel("Time - UTC")
        plt.ylabel("TEC - TECU")
        plt.title("Total Electron Content")
        plt.show()

        #save the data to a csv file
        with open('data.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time", "TEC"])
            for i in range(len(data)):
                writer.writerow([times[i], data[i]])
        print("Data saved to data.csv")
        
