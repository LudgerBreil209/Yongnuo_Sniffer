# Yongnuo_Sniffer

Arduino project for exploring the Yongnuo 560 protocol

While the basic 603-protocol, the communication between Yongnuo RF-603 devices for setting focus, flash, release shutter, was explored, i was curious how the more elaborated protocol with the Yongnuo YN560-TX and Yongnuo Speedlite YN560-IV works. With these devices you can control, in addition to the capabilities of the basic RF-603 devices, the power, zoom factor and some more parameters of the Speedlite 560 III and IV flashes.

The 603-protocol was explored by robotfreak and i used his code (robotfreak/YongnuoRF) as a starting point for my experiments.

While robotfreak explored the protocol by sniffing the programming of the A7105 chip on the SPI-bus, i explored the protocol by sniffing the data with an HackRF One SDR.

For testing i had an Yongnuo YN560-TX and a Speedlite YN560 IV Á sender and a Speedlite YN560-III Á receiver.

From the work of robotfreak it was known that the RF-603 protocol used an ID-Code of 0x35999A5A and a fixed payload of 2 bytes and no CRC (despite the fact, that the A7105 chip offers a build in CRC-handling). He found several payload 

1.) release flash: 0x88, 0x77
2.) release shutter: 0x22 0xdd
3.) set focus: 0x11 0xee




Sending the data to the flashes goes as :

1.) 'normal' 603 packet: id = 0x35999A5A payload = 0x44BB ('wake up')
2.) packet with id = 0x35999A5A and payload = 0x32CD (switch command)
3.) packet with id = 0x35999A33 and payload = 0x32CD0F0D
4.) packet with the data, id = 0x22999A33 and 15 bytes payload
5.) packet with id = 0x22999A33 payload = 0x32CD (switch command)
