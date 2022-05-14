# Yongnuo_Sniffer

## Arduino project for exploring the Yongnuo 560 protocol

While the basic 603 protocol, the communication between the Yongnuo RF-603 units for setting focus, flash and shutter release, has been around for a while, I was curious to see how the data is transferred from the Yongnuo YN560-TX or the Yongnuo Speedlite YN560-IV. With these two devices, in addition to the capabilities of the RF-603 base units, you can control the power, zoom factor, and some other parameters of the Speedlite 560 III and IV flashes.

The 603 protocol was researched by robotfreak and I used his code [RoboFreak/YongnuoRF](https://github.com/robotfreak/YongnuoRF) as a starting point for my experiments.

While robotfreak explored the protocol by sniffing the programming of the A7105 chip on the SPI-bus, i explored the protocol by sniffing the data with an HackRF One SDR.

For testing i used a YN560-TX and a Speedlite YN560 IV as senders and a Speedlite YN560-III and -IV as receivers.

## data package
All Yongnuo devices I had for testing use an A7105 chip for wireless communication. Although the chip has implemented some advanced features like automatic CRC, data whitening, etc., Yongnuo only uses some very basic functions. A packet consist of a preamble, a data id and the payload:

    1) 4 bytes of a repeating pattern: here AA AA AA AA
    2) 4 bytes data-id. This is the 'address' and the chip only reports data packets received with the programmed data id
    allowing it to use several communication with the same frequency, but different data-id's
    3) the payload. While the payload in 603-mode is always 2 bytes, the payload in 560-mode differs (2, 4, 16 and 32 bytes)

From robotfreak's work, it was known that the RF-603 protocol uses a fixed data id of 35 99 9A 5A and a fixed payload of 2 bytes and no CRC (although the A7105 chip provides built-in CRC processing). 

Sending parameters to the speedlites is more complex and requires multiple data packets with different data id's. The complexity is probably required to be compatible with the simpler 603 devices.

data id | payload length | payload | comment
--------| -------------- | ------- | -------
35 99 9A 5A | 2 | 44 BB | ('wake up') ['normal' 603 packet]
35 99 9A 5A | 2 | 32 CD | (change mode command) ['normal' 603 packet]
35 99 9A 33 | 4 | 32 CD 0F 0D | (YN560 IV sends .. 0F 0E)
22 99 9A 33 | 15 | 14 bytes parameters + 1 byte CRC | 
22 99 9A 33 | 2 | 32 CD | (change mode command)

## Speedlite parameters
The 15 bytes contains always a complete parameter set for one of the six possible groups:

offset | length | data (hex)
------ | -------| ----------------------------
 0 | 2 | ID : fixed 30 CF 
 2 | 1 | group (bit 5 .. 7: A = 20; b = 40, ... ) or 0F (broadcast group ?)
 3 | 1 | (unknown)
 4 | 1 | zoom level (in mm)
 5 | 1 | flash mode (0 = off; 1 = Multi-Flash; 2 = Manual)
 6 | 1 | (unknown)
 7 | 1 | (unknown)
 8 | 1 | power level Manual mode (00 = 1/128, 01 = 1/128 + 0.3, ... 18 = 1/1)
 9 | 1 | power level Multi-Flash mode
10 | 1 | Multi-Flash mode: number of flashes
11 | 1 | Multi-Flash mode: frequency
12 | 1 | (unknown) (non 0 with 560 IV)
13 | 1 | (unknown) (non 0 with 560 IV)
14 | 1 | CRC (simple sum off all bytes)


## Speedlite ACT
In ACT mode the YN560-TX and YN560-IV sends repeatingly every second 41 bytes of data. The communication protocol is very similar and uses the same data id's:

data id | payload length | payload | comment
------- | -------- | --------------- | ------------
35 99 9A 5A | 2 | 44 BB | ('wake up') ['normal' 603 packet]
35 99 9A 5A | 2 | 32 CD | (change mode command) ['normal' 603 packet]
35 99 9A 33 | 4 | 32 CD 29 28 | 
22 99 9A 33 | 41 | 41 bytes parameters | 
22 99 9A 33 | 2 | 32 CD | (change mode command)

