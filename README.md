# Breakthrough-Starshot Project Team

Breakthrough Starshot is based off of Breakthrough Initiatives project to send nano spacecraft to nearby stars. A "mothership" deploys 
multiple nano spacecraft. Then, they are accelerated to travel to celestial bodies. 

Our team's goal is to deploy nano spacecraft in an inexpensive manner. We are doing this by using ordinary electronics. The computers on
our mothership are 2 Adafruit Microcontrollers. One Adafruit is known as the master computer, the other is known as the slave computer. 
The master computer contains code that manages deployment (e.g. turn off/on magnetorquers, enable/disable Rockblock communication system).
The slave computer stores the readings from the different sensors on our mothership and "hands it over" to the master computer when the
master computer requests data from the slave computer. 

The master computer code is in the file FlightMain.ino and the slave computer code is in the file SlaveMain.ino 
