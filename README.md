# Boonton-92BD
How to fix the 92 series RF Millivolt Meter

Some years ago I came across a 92BD RF Millivolt meter which was in reasonable condition
but after some time started to produce some issue which indiicated that the chopper tube
was close to end of life and obviously these are no longer available which kind meant that
the meter was no longer usable.

I came across an possible solution being the total replacement of the front end with a new
a couble of low drift op-amps and it's associated gain selection, the design was made by
Jacques Audet based on pref board with the proper connect suitable for the 92BD backplane.
I started off with making a PCB for it but never build one myself sofar.

As part of another project I became interessed ith the usage of SSR which used Mosfet's and
it's drive circuit being electrically isolated from the signal. These device can driven with
a few mAmp and the mosfet will turn on within uSec and have a very low R-on resistance.

The idea to replace the chopper tube with 2 SSR's (both within one physical package) was born
and my initial extention board for the autoranging was replace by a new pcb which would take
care of both the already working autoranging and the drive circuit for the SSR part.

As signal levels are fairly low and any noise infuence was to be avoided I made an additional
smaal PCB which is being directly mount at the signal input connector with fairly short leads 
to the SSR PCB and one lead to the original input on the backplane.
A set of the controle wires which are twisted are fed from the SSR PCB to a couple of free pin
on the backplane connector which was originally intended to hold the auto ranging board.
On this connector we also need to make one additional connection being a jumper wire to provide
5 Vdc to the auto ranging connector.

More to follow ......
