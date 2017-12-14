# phyNODE Base Example

This project is located in `examples/phynode_base/`.

This example demonstrates the usage of the sensors and the ePaper display via
their corresponding shell driver.

## Usage
Connect the phyNODE-Athena via USB and open a serial terminal:
```
minicom -D /dev/ttyACM0
```
Once connected a command prompt should be visible (if not you may have to
initially hit *Enter*):
```
phyNODE:>
```
Type `help` to get a list of available commands:
```
phyNODE:>help
help      print command description/usage
version   print version of all the registered modules
history   print history
board
sleep

hdc1010
hdc1010 get
hdc1010 init
hdc1010 reset

tcs37727
tcs37727 get
tcs37727 init
tcs37727 active
tcs37727 standby

ccs811
ccs811 get
ccs811 init
ccs811 suspend
ccs811 resume
ccs811 error

epd
epd init
epd print
epd suspend
```
With these commands it is now possible to read and write information from and to
the corresponding peripherals.

### HDC1010 -- Temperature and Relative Humidity
```
phyNODE:>hdc1010 init
--> Initialized HDC1010

phyNODE:>hdc1010 get
--> hdc1010 values: T: 2596 HR: 4254
```
Which translates to 25.96°C air temperature and 42.54% relative humidity.

### TCS37727 -- Light Intensity and Color
```
phyNODE:>tcs37727 init

--> Initialized tcs37727

phyNODE:>tcs37727 active

--> tcs37727 is active now

phyNODE:>tcs37727 get

--> tcs37727 values: R: 747 G: 798 B: 717 C: 2415 CT: 5046 Lux: 582 AGAIN: 4 ATIME: 200000
```
The resulting values are:

* RGB raw values: 747/798/717
* Illuminance: 582 lx
* Color temperature: 5046 K

### CCS811 -- Gas Sensor for Monitoring Indoor Air Quality
```
phyNODE:>ccs811 init

--> Initialized CCS811

phyNODE:>ccs811 get

--> ccs811 values: eCO2: 454 TVOC: 8 status: 98
```
The CO2 level measured in this example is approximately 454 ppm.

> **Note:** The CCS811 may need some time to initialize before returning
> reasonable values.

### EPD -- ePaper Display
```
phyNODE:>epd init
EPD initialized.

phyNODE:>epd print test
4-->printed
```
This will print "test" on the ePaper display.
