EESchema Schematic File Version 2
LIBS:SensorPod_Breakout_v1-rescue
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:TEENSYPP2
LIBS:WIZ812MJ
LIBS:XBEE
LIBS:74LVC1T45
LIBS:SensorPod_Breakout_v1-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 4
Title "Power"
Date "2017-02-26"
Rev "1.0"
Comp "Good Measure Design, LLC."
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	9350 4800 9350 4700
$Comp
L GND #PWR057
U 1 1 5758465A
P 9350 4800
F 0 "#PWR057" H 9350 4550 50  0001 C CNN
F 1 "GND" H 9350 4650 50  0000 C CNN
F 2 "" H 9350 4800 60  0000 C CNN
F 3 "" H 9350 4800 60  0000 C CNN
	1    9350 4800
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 57584674
P 9050 4550
F 0 "C1" H 9075 4650 50  0000 L CNN
F 1 "4.7u" H 9075 4450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 9088 4400 30  0001 C CNN
F 3 "" H 9050 4550 60  0000 C CNN
	1    9050 4550
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 575846D1
P 9350 4550
F 0 "C2" H 9375 4650 50  0000 L CNN
F 1 "4.7u" H 9375 4450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 9388 4400 30  0001 C CNN
F 3 "" H 9350 4550 60  0000 C CNN
	1    9350 4550
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P2
U 1 1 575C7D9F
P 2300 5650
F 0 "P2" H 2300 5750 50  0000 C CNN
F 1 "CONN_01X01" V 2400 5650 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_2.5mm" H 2300 5650 60  0001 C CNN
F 3 "" H 2300 5650 60  0000 C CNN
	1    2300 5650
	0    -1   -1   0   
$EndComp
NoConn ~ 2300 5850
$Comp
L CONN_01X01 P3
U 1 1 575C8070
P 2850 5650
F 0 "P3" H 2850 5750 50  0000 C CNN
F 1 "CONN_01X01" V 2950 5650 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_2.5mm" H 2850 5650 60  0001 C CNN
F 3 "" H 2850 5650 60  0000 C CNN
	1    2850 5650
	0    -1   -1   0   
$EndComp
NoConn ~ 2850 5850
$Comp
L CONN_01X01 P4
U 1 1 575C8107
P 3400 5650
F 0 "P4" H 3400 5750 50  0000 C CNN
F 1 "CONN_01X01" V 3500 5650 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_2.5mm" H 3400 5650 60  0001 C CNN
F 3 "" H 3400 5650 60  0000 C CNN
	1    3400 5650
	0    -1   -1   0   
$EndComp
NoConn ~ 3400 5850
$Comp
L CONN_01X01 P5
U 1 1 575C810E
P 3950 5650
F 0 "P5" H 3950 5750 50  0000 C CNN
F 1 "CONN_01X01" V 4050 5650 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_2.5mm" H 3950 5650 60  0001 C CNN
F 3 "" H 3950 5650 60  0000 C CNN
	1    3950 5650
	0    -1   -1   0   
$EndComp
NoConn ~ 3950 5850
Text Notes 2750 5350 0    60   ~ 0
Mounting Holes
$Comp
L +5V #PWR059
U 1 1 57636557
P 8050 4250
F 0 "#PWR059" H 8050 4100 50  0001 C CNN
F 1 "+5V" H 8050 4390 50  0000 C CNN
F 2 "" H 8050 4250 60  0000 C CNN
F 3 "" H 8050 4250 60  0000 C CNN
	1    8050 4250
	1    0    0    -1  
$EndComp
Text Notes 7450 3850 0    60   ~ 0
Battery and Charger Connection\nDaughterboard provides battery power and charging.
$Comp
L +5V #PWR061
U 1 1 58D01C94
P 4550 3350
F 0 "#PWR061" H 4550 3200 50  0001 C CNN
F 1 "+5V" H 4550 3490 50  0000 C CNN
F 2 "" H 4550 3350 60  0000 C CNN
F 3 "" H 4550 3350 60  0000 C CNN
	1    4550 3350
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR062
U 1 1 58D02542
P 3400 3750
F 0 "#PWR062" H 3400 3600 50  0001 C CNN
F 1 "+3.3V" H 3400 3890 50  0000 C CNN
F 2 "" H 3400 3750 50  0000 C CNN
F 3 "" H 3400 3750 50  0000 C CNN
	1    3400 3750
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR063
U 1 1 58D02674
P 1700 3750
F 0 "#PWR063" H 1700 3600 50  0001 C CNN
F 1 "VCC" H 1700 3900 50  0000 C CNN
F 2 "" H 1700 3750 50  0000 C CNN
F 3 "" H 1700 3750 50  0000 C CNN
	1    1700 3750
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR064
U 1 1 58D0270A
P 2200 3750
F 0 "#PWR064" H 2200 3600 50  0001 C CNN
F 1 "+5V" H 2200 3890 50  0000 C CNN
F 2 "" H 2200 3750 60  0000 C CNN
F 3 "" H 2200 3750 60  0000 C CNN
	1    2200 3750
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR065
U 1 1 58D02739
P 2900 3750
F 0 "#PWR065" H 2900 3600 50  0001 C CNN
F 1 "VCC" H 2900 3900 50  0000 C CNN
F 2 "" H 2900 3750 50  0000 C CNN
F 3 "" H 2900 3750 50  0000 C CNN
	1    2900 3750
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR066
U 1 1 58D02768
P 4750 3550
F 0 "#PWR066" H 4750 3400 50  0001 C CNN
F 1 "+3.3V" H 4750 3690 50  0000 C CNN
F 2 "" H 4750 3550 50  0000 C CNN
F 3 "" H 4750 3550 50  0000 C CNN
	1    4750 3550
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 58D029E4
P 1950 3850
F 0 "R9" V 2030 3850 50  0000 C CNN
F 1 "0" V 1950 3850 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 1880 3850 50  0001 C CNN
F 3 "" H 1950 3850 50  0000 C CNN
	1    1950 3850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1700 3750 1700 3850
Wire Wire Line
	1700 3850 1800 3850
Wire Wire Line
	2100 3850 2200 3850
Wire Wire Line
	2200 3850 2200 3750
$Comp
L R R10
U 1 1 58D02B31
P 3150 3850
F 0 "R10" V 3230 3850 50  0000 C CNN
F 1 "DNP" V 3150 3850 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 3080 3850 50  0001 C CNN
F 3 "" H 3150 3850 50  0000 C CNN
	1    3150 3850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2900 3750 2900 3850
Wire Wire Line
	2900 3850 3000 3850
Wire Wire Line
	3300 3850 3400 3850
Wire Wire Line
	3400 3850 3400 3750
$Comp
L CONN_01X04 P7
U 1 1 58D02F16
P 4250 3600
F 0 "P7" H 4250 3850 50  0000 C CNN
F 1 "LM3671" V 4350 3600 50  0000 C CNN
F 2 "LM3671:LM3671" H 4250 3600 50  0001 C CNN
F 3 "" H 4250 3600 50  0000 C CNN
	1    4250 3600
	-1   0    0    1   
$EndComp
Wire Wire Line
	4550 3550 4450 3550
Wire Wire Line
	4550 3350 4550 3550
Wire Wire Line
	4550 3450 4450 3450
Connection ~ 4550 3450
Wire Wire Line
	4450 3650 4750 3650
Wire Wire Line
	4750 3650 4750 3550
Wire Wire Line
	4450 3750 4550 3750
Wire Wire Line
	4550 3750 4550 3850
$Comp
L GND #PWR067
U 1 1 58D0337A
P 4550 3850
F 0 "#PWR067" H 4550 3600 50  0001 C CNN
F 1 "GND" H 4550 3700 50  0000 C CNN
F 2 "" H 4550 3850 60  0000 C CNN
F 3 "" H 4550 3850 60  0000 C CNN
	1    4550 3850
	1    0    0    -1  
$EndComp
Text Notes 2550 2950 0    60   ~ 0
Power Selection Circuitry
Text Notes 1650 4400 0    60   ~ 0
VCC is power to/from the Teensy. By default it is 5V and provided by USB.\nIf desired, the Teensy can be adapted (by means of a trace cut) for 3.3V\noperation. If this is done, remove R9 and place R10.
$Comp
L CONN_01X05 J?
U 1 1 58DEE009
P 7650 4550
F 0 "J?" H 7650 4850 50  0000 C CNN
F 1 "CONN_01X05" V 7750 4550 50  0000 C CNN
F 2 "" H 7650 4550 50  0001 C CNN
F 3 "" H 7650 4550 50  0001 C CNN
	1    7650 4550
	-1   0    0    1   
$EndComp
Wire Wire Line
	9050 4800 9050 4700
$Comp
L GND #PWR?
U 1 1 58DEE2A3
P 9050 4800
F 0 "#PWR?" H 9050 4550 50  0001 C CNN
F 1 "GND" H 9050 4650 50  0000 C CNN
F 2 "" H 9050 4800 60  0000 C CNN
F 3 "" H 9050 4800 60  0000 C CNN
	1    9050 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 4650 8050 4850
$Comp
L GND #PWR?
U 1 1 58DEE2CC
P 8050 4850
F 0 "#PWR?" H 8050 4600 50  0001 C CNN
F 1 "GND" H 8050 4700 50  0000 C CNN
F 2 "" H 8050 4850 60  0000 C CNN
F 3 "" H 8050 4850 60  0000 C CNN
	1    8050 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 4650 8050 4650
Wire Wire Line
	7850 4750 8050 4750
Connection ~ 8050 4750
Wire Wire Line
	7850 4450 8050 4450
Wire Wire Line
	8050 4450 8050 4250
Wire Wire Line
	7850 4350 8050 4350
Connection ~ 8050 4350
$Comp
L +5V #PWR?
U 1 1 58DEE43B
P 9050 4250
F 0 "#PWR?" H 9050 4100 50  0001 C CNN
F 1 "+5V" H 9050 4390 50  0000 C CNN
F 2 "" H 9050 4250 60  0000 C CNN
F 3 "" H 9050 4250 60  0000 C CNN
	1    9050 4250
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 58DEE461
P 9350 4250
F 0 "#PWR?" H 9350 4100 50  0001 C CNN
F 1 "+5V" H 9350 4390 50  0000 C CNN
F 2 "" H 9350 4250 60  0000 C CNN
F 3 "" H 9350 4250 60  0000 C CNN
	1    9350 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 4250 9050 4400
Wire Wire Line
	9350 4250 9350 4400
$Comp
L VCC #PWR?
U 1 1 58DEE8C5
P 8350 4450
F 0 "#PWR?" H 8350 4300 50  0001 C CNN
F 1 "VCC" H 8350 4600 50  0000 C CNN
F 2 "" H 8350 4450 50  0000 C CNN
F 3 "" H 8350 4450 50  0000 C CNN
	1    8350 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 4550 8350 4550
Wire Wire Line
	8350 4550 8350 4450
Wire Wire Line
	10150 4800 10150 4700
$Comp
L GND #PWR?
U 1 1 58DEEA47
P 10150 4800
F 0 "#PWR?" H 10150 4550 50  0001 C CNN
F 1 "GND" H 10150 4650 50  0000 C CNN
F 2 "" H 10150 4800 60  0000 C CNN
F 3 "" H 10150 4800 60  0000 C CNN
	1    10150 4800
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58DEEA4D
P 10150 4550
F 0 "C?" H 10175 4650 50  0000 L CNN
F 1 "4.7u" H 10175 4450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 10188 4400 30  0001 C CNN
F 3 "" H 10150 4550 60  0000 C CNN
	1    10150 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 4250 10150 4400
$Comp
L VCC #PWR?
U 1 1 58DEEA6A
P 10150 4250
F 0 "#PWR?" H 10150 4100 50  0001 C CNN
F 1 "VCC" H 10150 4400 50  0000 C CNN
F 2 "" H 10150 4250 50  0000 C CNN
F 3 "" H 10150 4250 50  0000 C CNN
	1    10150 4250
	1    0    0    -1  
$EndComp
$EndSCHEMATC
