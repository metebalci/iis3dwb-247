# iis3dwb-247

This is a demo application to save data from ST's IIS3DWB wideband accelerometer sensor directly to a persistent storage (SD card). Therefore, it can be used 24/7, as long as there is enough free storage.

It is designed to be used on a Raspberry Pi (tested on a RPi 4 4GB) running the standard Raspian (64-bit) distribution.

# Usage

Build the application with `make`.

Run `./iis3dwb <seconds>`. 

# Design

The demo application configures the IIS3DWB sensor to run in continuous FIFO mode (where the new sample overwrites the older one) and enables timestamp batching in FIFO with no decimation (or decimation=1). This means there is going to be always a pair of timestamp and accelerometer samples one after other in the FIFO. The FIFO can contain maximum 512 samples (thus 256 timestamps and 256 accelerometer data for this demo). A FIFO overrun (overflow) is indicated with FIFO_OVR_IO flag in FIFO_STATUS2 and the number of entries in FIFO is indicated with DIFF_FIO in FIFO_STATUS1 and FIFO_STATUS2 registers.

The demo is started by providing a duration in seconds (which is stored in `duration` variable). By reading the INTERNAL_FINE_FREQUENCY register, actual output data rate (ODR) is calculated and the amount of samples required is kept in a variable called `nsamples`. This is actually a double of what is required because each accelerometer sample is paired with a timestamp sample.

After configuring the sensor, the demo creates two (p)threads.

First thread is `sensor_reader`, which does the synchronuous SPI transaction to read a certain amount of samples from the FIFO. The SPI transaction transmit buffer is always the same since the command is always the same, and the receive buffer is always part of the circular 2D buffer (a particular line, more information below). `sensor_reader` continues its operation until requested amount of samples are read from the sensor in an infinite while loop.

Second thread is `file_writer`, which reads the FIFO samples from the circular 2D buffer (lines) and writes them to a file (named `fifo.bin` by default). `file_writer` continues its operation until requested amount of samples are written to the output file in an infinite while loop.

## Circular 2D Buffer

A traditional circular buffer keeps a single byte at each position. It is possible to use a similar construct in this application but I decided to make a variation which does not have a single byte but another (fixed length) byte array at each position, thus the circular buffer has two dimensions. I called the byte array at each position of the circular buffer a `line`. It is configurable at compile-time but each line can contain maximum 256 FIFO samples (each sample is 7 bytes), and the maximum size of a line is `2 + 256 * 7` bytes. The first byte of a line contains how many FIFO samples the line actually contains (although the line might have a larger size). The second byte of a line is not used, because `line+1` is used as SPI receive buffer, and the first byte of a FIFO read transaction will contain a dummy byte. Thus, the actual samples are stored starting at `line+2`.

The empty and full status of this circular 2D buffer is checked by the threads using `fifo_rx_nlines` variable. Hence, this is the only time a mutex is used when increasing or decreasing this variable.

## Thread Affinity

The demo is run on a RPi 4, which has 4 cores and the threads are assigned to a specific core and Linux is prevented from using these cores.

First, the kernel cmdline in `/boot/cmdline.txt` is modified to contain `isolcpus=2-3`. This isolates the core 2 and 3 from the Linux scheduler, thus no process is automatically run on these cores.

Then, after creating the threads, demo application sets the affinity of `sensor_reader` thread to core 2 and `file_writer` thread to core 3.

In effect, core 2 and core 3 is only used by the demo application without any interruption from Linux.

# Performance
