EESchema Schematic File Version 2
LIBS:knielsenlib
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
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Photores R1
U 1 1 568146E0
P 2750 2750
F 0 "R1" H 2958 2796 50  0000 L CNN
F 1 "Photores" H 2958 2704 50  0000 L CNN
F 2 "Diodes_ThroughHole:Diode_TO-220_Vertical" V 2680 2750 50  0001 C CNN
F 3 "" H 2750 2750 50  0000 C CNN
	1    2750 2750
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR01
U 1 1 56814AED
P 2750 2250
F 0 "#PWR01" H 2750 2100 50  0001 C CNN
F 1 "VCC" H 2770 2424 50  0000 C CNN
F 2 "" H 2750 2250 50  0000 C CNN
F 3 "" H 2750 2250 50  0000 C CNN
	1    2750 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 2250 2750 2500
$Comp
L R R2
U 1 1 56814B06
P 3050 3200
F 0 "R2" V 2842 3200 50  0000 C CNN
F 1 "600k" V 2934 3200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2980 3200 50  0001 C CNN
F 3 "" H 3050 3200 50  0000 C CNN
	1    3050 3200
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 56814B49
P 3500 3200
F 0 "R3" V 3292 3200 50  0000 C CNN
F 1 "200k" V 3384 3200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3430 3200 50  0001 C CNN
F 3 "" H 3500 3200 50  0000 C CNN
	1    3500 3200
	0    1    1    0   
$EndComp
$Comp
L GND #PWR02
U 1 1 56814B74
P 3750 3300
F 0 "#PWR02" H 3750 3050 50  0001 C CNN
F 1 "GND" H 3758 3126 50  0000 C CNN
F 2 "" H 3750 3300 50  0000 C CNN
F 3 "" H 3750 3300 50  0000 C CNN
	1    3750 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 2850 3750 3300
Wire Wire Line
	3650 3200 4000 3200
Wire Wire Line
	3350 3200 3200 3200
Wire Wire Line
	2900 3200 2750 3200
Wire Wire Line
	2750 3200 2750 3000
$Comp
L CONN_01X03 P1
U 1 1 56814BA5
P 3750 2450
F 0 "P1" H 3827 2488 50  0000 L CNN
F 1 "CONN_01X03" H 3827 2396 50  0000 L CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x03" H 3750 2450 50  0001 C CNN
F 3 "" H 3750 2450 50  0000 C CNN
	1    3750 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 2350 2750 2350
Connection ~ 2750 2350
Wire Wire Line
	3550 2450 3300 2450
Wire Wire Line
	3300 2450 3300 3200
Connection ~ 3300 3200
Wire Wire Line
	3550 2550 3550 2850
Wire Wire Line
	3550 2850 3750 2850
Connection ~ 3750 3200
$Comp
L PWR_FLAG #FLG03
U 1 1 56814C96
P 3050 2300
F 0 "#FLG03" H 3050 2395 50  0001 C CNN
F 1 "PWR_FLAG" H 3050 2524 50  0000 C CNN
F 2 "" H 3050 2300 50  0000 C CNN
F 3 "" H 3050 2300 50  0000 C CNN
	1    3050 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 2300 3050 2350
Connection ~ 3050 2350
$Comp
L PWR_FLAG #FLG04
U 1 1 56814CDD
P 4000 3200
F 0 "#FLG04" H 4000 3295 50  0001 C CNN
F 1 "PWR_FLAG" H 4000 3424 50  0000 C CNN
F 2 "" H 4000 3200 50  0000 C CNN
F 3 "" H 4000 3200 50  0000 C CNN
	1    4000 3200
	1    0    0    -1  
$EndComp
$EndSCHEMATC
