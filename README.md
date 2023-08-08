# Boonton 92 series

<ins>How to fix & add auto range option to your 92 series RF Millivolt Meter</ins>

Some years ago I came across a 92BD RF Millivolt meter which was in reasonable condition
but after some time started to produce some issue which indiicated that the chopper tube
was close to end of life and obviously these are no longer available which kind meant that
the meter was no longer usable.

I came across an possible solution being the total replacement of the front end with a new
a couple of low drift op-amps and it's associated gain selection, the design was made by
Jacques Audet based on pref board with the proper connector suitable for the 92 series backplane.
I started off by making a PCB for it but never build one myself sofar.

As part of another project I became interessed in the usage of SSR which used Mosfet's and
it's drive circuit being electrically isolated from the signal. These device can driven with
a few mAmp and the mosfet will turn on within uSec and have a very low R-on resistance.

The idea to replace the chopper tube with 2 SSR's (both within one physical package) was born
and my initial mcu based extention board for the autoranging was replace by a new pcb which
would take care of both: the already working autoranging and the drive circuit for the SSR.

As signal levels are fairly low and any noise infuence was to be avoided I made an additional
small PCB which is directly mounted behind the signal input connector with fairly short leads 
to the SSR PCB and one lead to the original analog input on the backplane.
A set of the controle wires which are twisted are fed from the SSR PCB to a couple of free pin
on the backplane connector J103 which was originally intended to hold the auto ranging board.
On this connector we also need to make one additional connection being a jumper wire to provide
5 Vdc to the auto ranging connector j103.

The current design consist of two part being: the autoranging circuit and the SSR control circuit
which are both controlled by the MCU for which I used the ESP32 (and Yes both BT & WiFi are being
turned off) and the MCU choice was based on previous projects and it large amount of IO pins.

As the 92 series RF meter is using negative control voltage (upto -15 Vdc) and the MCU it's pins
are only 3.3 Vdc tolerant we are using 9 optocouplers (8 for the range selection and 1 for auto
range mode) to overcome this difference in control voltages. All what needed is to measure the
analog voltage which needs to be reduced to less than 3 Vdc which is done with a small divider
resistor network.

The chopper part takes the original chopper tube connector being plugged directly onto the pcb and
its voltage level being reduced to less than 3.3Vdc as input to the MCU. The MCU will adjust the signal
output to the SSR slightly as obtain a contact timing which is simular to the chopper tube and to
assure is has an break before make SPDT contact behaviour. Driving the opto input side of  the SSR
is easy and only requires a single resistor to limit the opto couple led current.

V2.2 PCB picture, it works fine but needed some tweaking on the physical size, connector position, some
GPIO ports where changed, an JTAG port, seriel port was ommited and few resistors.
Mind the small pcb within the large one needs to be removed and it will be used behind the instrument
input connector mounted on two short standoff's using the holes of the chopper tube bracket.

![20230806_115556](https://github.com/ph-wheels/Boonton_92BD/assets/10708995/48fdfadd-2625-4f61-920f-065d6dd4d30e)

This picture is from V2.2 PCB, V2.7 still to be build and will make a much cleaner impression !
A few other pictures (in main folder) as to provide some detail on the build & mods needed

The small SSR PCB will only hold the original cap's which came from the chopper tube socket, mind the black strip
to be connected to the ground pad's of C1 & C3 and the SSR then using some small coax to be routed to the input
connector and the analog main PCB. Reason for the cut out SSR PCB is to reduce PCB fabrication cost and the main
PCB has so much real estate left that it was an obvious choice to make.

Have fun building
