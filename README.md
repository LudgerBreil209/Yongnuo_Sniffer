# Yongnuo_Sniffer

Arduino project for exploring the Yongnuo 560 protocol

While the basic 603-protocol, the communication between Yongnuo RF-603 devices for setting focus, flash, release shutter, was explored, i was curious how the more elaborated protocol with the Yongnuo YN560-TX and Yongnuo Speedlite YN560-IV works. With these devices you can control, in addition to the capabilities of the basic RF-603 devices, the power, zoom factor and some more parameters of the Speedlite 560 III and IV flashes.

The 603-protocol was explored by robotfreak and i used his code (robotfreak/YongnuoRF) as a starting point for my experiments.

While robotfreak explored the protocol by sniffing the programming of the A7105 chip on the SPI-bus, i explored the protocol by sniffing the data with an HackRF One SDR.

For testing i had an Yongnuo YN560-TX and a Speedlite YN560 IV as senders and a Speedlite YN560-III as a receiver.

All(?) the Yongnuo equipment i used, use a A7105 chip for the wireless communication. DEspite the chip implemented some advanced feature like automatic CRC, data whitening, etc. Yongnuo only use some very basic stuff. 

A packet contains of a peamble, the data id and the payload. 
Thhe preamble contains of 
    1) 4 bytes of a repeating pattern: here 0xAAAAAAAA
    2) 4 bytes data-id. This is the 'address' and the chip only reports data packets received with the programmed data id
    allowing it to use several communication with the same frequency, but different data-id's
    3) the payload. While the payload in 603-mode is always 2 bytes, the payload in 560-mode differs (2, 4, 16 and 32 bytes)

From the work of robotfreak it was known that the RF-603 protocol used an ID-Code of 0x35999A5A and a fixed payload of 2 bytes and no CRC (despite the with a fact, that the A7105 chip offers a build in CRC-handling). He found several payload 

1.) release flash: 0x88, 0x77
2.) release shutter: 0x22 0xdd
3.) set focus: 0x11 0xee

Sending the data to the flashes goes as :

1.) id = 0x35999A5A payload 2 bytes: 0x44BB ('wake up') ['normal' 603 packet]
2.) id = 0x35999A5A payload 2 bytes: 0x32CD (switch command)
3.) id = 0x35999A33 payload 4 bytes: 0x32CD0F0D
4.) id = 0x22999A33 payload 15 bytes: parameter data
5.) id = 0x22999A33 payload 2 bytes: 0x32CD (switch command)
