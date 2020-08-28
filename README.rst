A Python driver for the K-LD7 radar module
==========================================

This is a simple, high-level interface to the K-LD7_ radar module. It
lets the user read and set the various radar parameters as described
in the datasheet_ and fetch the various sorts of raw or processed data
frames from the sensor, either individually or as a stream. 

Tracking the distance, speed, angle and magnitude of the most
promenant moving object in range is as simple as:

.. code-block:: python

    from kld7 import KLD7
    with KLD7("/dev/tty.usbserial") as radar:
        for target_info in radar.stream_TDAT():
            print(target_info)

Documentation
-------------

The API for this module is documented on ReadTheDocs_.
            
Overview
--------

According the the datasheet:

    "The K-LD7 is a fully digital low cost Doppler radar that can
    measure speed, direction, distance and angle of moving objects in
    front of the sensor. The digital structure and wide power supply
    range make it very easy to use this sensor in any stand-alone or
    MCU based application."

The module can detect and measure multiple moving objects and provide
filtered tracking information about the most prominent one. Various
detection criteria can be stored and a set of programmable output pins
on the module can be configured to indicate detection status, allowing
for stand-alone operation once the coniguration paramters have been
set.

The K-LD7 module includes an integrated signal processor and allows
access to the radar signal at a number of levels including raw ADC
samples, FFT output, candidate targets, most significant tracked
target and flags for the tracked target relative to detection
thresholds. Each level of processing provides higher-level information
and thus reduces the volume of data that needs to be sent to the host.
This driver allows users to access each of these levels of data to be
fetched, either as a single snamshot or as a stream of data frames.

Configuration paramters
-----------------------

The K-LD7 module has 22 configurable radar paramters that can be used
to control the sensitivity, distance and speed range and detection
thresholds used during filtering, as well as for configuring the
programmable output pins. These paramters can be read and set though
a :any:`RadarParamProxy` object accessible as the :any:`params`
attribute of the driver object.

            
.. _K-LD7: https://www.rfbeam.ch/product?id=40

.. _datasheet: https://www.rfbeam.ch/files/products/40/downloads/Datasheet_K-LD7.pdf

.. _ReadTheDocs: https://kld7.readthedocs.io/en/latest/
