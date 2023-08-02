"""
ubxpoller.py

This example illustrates how to read, write and display UBX messages
concurrently using threads and queues. This represents a useful
generic pattern for many end user applications.

It implements three threads which run concurrently:
1) a read thread which continuously reads UBX data from the receiver.
2) a write thread which sends UBX commands or polls to the receiver.
3) a display thread which displays parsed UBX data at the terminal.
UBX data is passed between threads using queues.

Press CTRL-C to terminate.

FYI: Since Python implements a Global Interpreter Lock (GIL),
threads are not strictly concurrent, though this is of minor
practical consequence here. True concurrency could be
achieved using multiprocessing (i.e. separate interpreter
processes rather than threads) but this is non-trivial in
this context as serial streams cannot be shared between
processes. A discrete hardware I/O process must be implemented
e.g. using RPC server techniques.

Created on 07 Aug 2021

:author: semuadmin
:copyright: SEMU Consulting © 2021
:license: BSD 3-Clause
"""
# pylint: disable=invalid-name

from queue import Queue
from sys import platform
from threading import Event, Lock, Thread
from time import sleep
from serial import Serial
from pyubx2 import POLL, UBX_PROTOCOL, UBXMessage, UBXReader, SET


L1 = 1575.42e6 # L1 Frequency
L2 = 1227.60e6 # L2 Frequency




def calculateTEC(p1,p2, f1, f2):
    return (1/40.3)*((f1*f2)/(f1-f2))*(p2-p1)


def read_data(
    stream: object,
    ubr: UBXReader,
    queue: Queue,
    lock: Lock,
    stop: Event,
):
    """
    THREADED
    Read and parse incoming UBX data and place
    raw and parsed data on queue
    """
    # pylint: disable=unused-variable, broad-except

    while not stop.is_set():
        if stream.in_waiting:
            try:
                lock.acquire()
                (raw_data, parsed_data) = ubr.read()
                lock.release()
                if parsed_data:
                    queue.put((raw_data, parsed_data))
            except Exception as err:
                print(f"\n\nSomething went wrong {err}\n\n")
                continue


def write_data(stream: object, queue: Queue, lock: Lock, stop: Event):
    """
    THREADED
    Read queue and send UBX message to device
    """

    while not stop.is_set():
        if queue.empty() is False:
            message = queue.get()
            lock.acquire()
            stream.write(message.serialize())
            lock.release()
            queue.task_done()


def display_data(queue: Queue, stop: Event):
    """
    THREADED
    Get UBX data from queue and display.
    """
    # pylint: disable=unused-variable,

    while not stop.is_set():
        if queue.empty() is False:
            (raw, parsed) = queue.get()
            print(parsed)
            parsed
            queue.task_done()

if __name__ == "__main__":
    # set port, baudrate and timeout to suit your device configuration
    if platform == "win32":  # Windows
        port = "COM8"
    elif platform == "darwin":  # MacOS
        port = "/dev/tty.usbmodem2101"
    else:  # Linux
        port = "/dev/ttyACM1"
    baudrate = 9600
    timeout = 0.1

    with Serial(port, baudrate, timeout=timeout) as serial_stream:
        ubxreader = UBXReader(serial_stream, protfilter=UBX_PROTOCOL)

        serial_lock = Lock()
        read_queue = Queue()
        send_queue = Queue()
        stop_event = Event()

        read_thread = Thread(
            target=read_data,
            args=(
                serial_stream,
                ubxreader,
                read_queue,
                serial_lock,
                stop_event,
            ),
        )
        write_thread = Thread(
            target=write_data,
            args=(
                serial_stream,
                send_queue,
                serial_lock,
                stop_event,
            ),
        )
        display_thread = Thread(
            target=display_data,
            args=(
                read_queue,
                stop_event,
            ),
        )

        print("\nStarting handler threads. Press Ctrl-C to terminate...")
        read_thread.start()
        write_thread.start()
        display_thread.start()

        # loop until user presses Ctrl-C
        while not stop_event.is_set():
            try:
                # DO STUFF IN THE BACKGROUND...
                # poll the receiver port configuration using CFG-PRT
                print("\nPolling port configuration CFG-PRT...\n")
                for prt in (0, 1, 2, 3, 4):  # I2C, UART1, UART2, USB, SPI
                    msg = UBXMessage("CFG", "CFG-PRT", POLL, portID=prt)
                    send_queue.put(msg)
                sleep(1)

            except KeyboardInterrupt:  # capture Ctrl-C
                print("\n\nTerminated by user.")
                stop_event.set()

        print("\nStop signal set. Waiting for threads to complete...")
        read_thread.join()
        write_thread.join()
        display_thread.join()
        print("\nProcessing complete")


def findFreq(sigID):
    if sigID == 0:
        return "L1"
    elif (sigID == 3 or sigID == 4):
        return "L2"
    else:
        return None



