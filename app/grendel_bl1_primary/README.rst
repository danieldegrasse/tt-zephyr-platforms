This application will implement the BL1 for the primary SMC chiplet.
The BL1 will eventually be responsible for initializing the system and loading
the next stage bootloader.

Currently it simply prints a message over UART to validate communication with
secondary chiplets.
