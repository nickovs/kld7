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
    with KLD7("/dev/tty.usbserial') as radar:
        for target_info in radar.stream_TDAT():
            print(target_info)


.. _K-LD7: https://www.rfbeam.ch/product?id=40

.. _datasheet: https://www.rfbeam.ch/files/products/40/downloads/Datasheet_K-LD7.pdf

