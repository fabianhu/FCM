FCM
===

FCM Flight Controller Firmware

See the project page here: 
https://www.huslik-elektronik.de/cms/embedded-hardware/fcm-flight-control-module/7-fcm-flight-control-module

Contents:

	Bootloader - the bootloader AVR32 project
	
	doc -  the main documentation folder -> read FCM Manual.pdf.
	
	hardware - here are the hardware sources. "hexa2wide" is our board. The actual physically produced board is under versions. This is the next release!
	
	software - the FCM copter control firmware main project


    Size & weight:
        45 x 27 x 8 mm
        10 g
    CPU
        AVR32 UC3B0256
        32 bit
        48MHz
    Sensors:
        Gyroscope (3 axis)
        Accelometer (3 axis)
        Magnetometer (3 axis)
        Barometer
    Interfaces:
        4x PPM input or
        Receiver serial input for SPEKTRUM and HoTT or
        Receiver PPM sum signal input
        2x extra TTL USART (for Bluetooth, GPS, Telemetry etc.)
        6x PPM output for speed controllers (SimonK firmware recommended)
        optocoupler for 5 PPM outputs (placement variant)
        Pulse input for ultrasonic distance measurement
        USB for configuration and updates
    Software features:
        FabOS32 RTOS
        Quaternion based attitude control
        Altitude hold
        Any copter configuration via mixing matrix
        RC compatibility
            HoTT SUMD receiver signal
            SPEKTRUM sattelite receiver signal
            4-channel "any RX" classic mode
            PPM sum signal
        Telemetry
            HoTT telemetry with settings menue
            Bluetooth telemetry with settings menue, Attitude and graph display
            MAVlink prepared
        PID setting via extra RC analog channels
        Software is "closed source" = all own code with minor extensions with LGPL
        Software is open-sourced to hardware owners. (SVN access)
        USB Bootloader
    Being tested:
        GPS integration with waypoint processing and return-to-home
        Throw to start

The FCM is not yet a finished product and it is not for sale as a readily produced module.


Visit also the projects:
	FCM-android and
	FCM-Manager
