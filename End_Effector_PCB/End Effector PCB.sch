EESchema Schematic File Version 4
LIBS:End Effector PCB-cache
EELAYER 26 0
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
L power:+5V #PWR08
U 1 1 5C03003A
P 2250 900
F 0 "#PWR08" H 2250 750 50  0001 C CNN
F 1 "+5V" H 2300 1050 50  0000 C CNN
F 2 "" H 2250 900 50  0001 C CNN
F 3 "" H 2250 900 50  0001 C CNN
	1    2250 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 1100 2250 950 
$Comp
L power:GND #PWR017
U 1 1 5C0309D9
P 4750 3800
F 0 "#PWR017" H 4750 3550 50  0001 C CNN
F 1 "GND" H 4755 3627 50  0000 C CNN
F 2 "" H 4750 3800 50  0001 C CNN
F 3 "" H 4750 3800 50  0001 C CNN
	1    4750 3800
	1    0    0    -1  
$EndComp
$Comp
L MCU_Microchip_ATmega:ATmega328P-AU U1
U 1 1 5C08303E
P 2250 2600
F 0 "U1" H 2150 1700 50  0000 C CNN
F 1 "ATmega328P-AU" H 2150 1600 50  0000 C CNN
F 2 "Package_QFP:TQFP-32_7x7mm_P0.8mm" H 2250 2600 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega328_P%20AVR%20MCU%20with%20picoPower%20Technology%20Data%20Sheet%2040001984A.pdf" H 2250 2600 50  0001 C CNN
	1    2250 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 1100 2350 950 
Wire Wire Line
	2350 950  2250 950 
Connection ~ 2250 950 
Wire Wire Line
	2250 950  2250 900 
$Comp
L power:GND #PWR09
U 1 1 5C08338E
P 2250 4100
F 0 "#PWR09" H 2250 3850 50  0001 C CNN
F 1 "GND" H 2255 3927 50  0000 C CNN
F 2 "" H 2250 4100 50  0001 C CNN
F 3 "" H 2250 4100 50  0001 C CNN
	1    2250 4100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5C083670
P 1150 1400
F 0 "C2" V 898 1400 50  0000 C CNN
F 1 "0.1u" V 989 1400 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1188 1250 50  0001 C CNN
F 3 "~" H 1150 1400 50  0001 C CNN
	1    1150 1400
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5C083712
P 650 1550
F 0 "#PWR01" H 650 1300 50  0001 C CNN
F 1 "GND" H 655 1377 50  0000 C CNN
F 2 "" H 650 1550 50  0001 C CNN
F 3 "" H 650 1550 50  0001 C CNN
	1    650  1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 1400 1300 1400
Wire Wire Line
	1000 1400 650  1400
Wire Wire Line
	650  1400 650  1550
$Comp
L power:+5V #PWR02
U 1 1 5C083981
P 800 2350
F 0 "#PWR02" H 800 2200 50  0001 C CNN
F 1 "+5V" H 850 2500 50  0000 C CNN
F 2 "" H 800 2350 50  0001 C CNN
F 3 "" H 800 2350 50  0001 C CNN
	1    800  2350
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR04
U 1 1 5C0839BF
P 1350 2350
F 0 "#PWR04" H 1350 2200 50  0001 C CNN
F 1 "+5V" H 1400 2500 50  0000 C CNN
F 2 "" H 1350 2350 50  0001 C CNN
F 3 "" H 1350 2350 50  0001 C CNN
	1    1350 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5C0839D0
P 800 2500
F 0 "C1" H 915 2546 50  0000 L CNN
F 1 "0.1u" H 915 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 838 2350 50  0001 C CNN
F 3 "~" H 800 2500 50  0001 C CNN
	1    800  2500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5C083A4F
P 1350 2500
F 0 "C3" H 1465 2546 50  0000 L CNN
F 1 "0.1u" H 1465 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1388 2350 50  0001 C CNN
F 3 "~" H 1350 2500 50  0001 C CNN
	1    1350 2500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5C083A77
P 800 2800
F 0 "#PWR03" H 800 2550 50  0001 C CNN
F 1 "GND" H 805 2627 50  0000 C CNN
F 2 "" H 800 2800 50  0001 C CNN
F 3 "" H 800 2800 50  0001 C CNN
	1    800  2800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5C083A8C
P 1350 2800
F 0 "#PWR05" H 1350 2550 50  0001 C CNN
F 1 "GND" H 1355 2627 50  0000 C CNN
F 2 "" H 1350 2800 50  0001 C CNN
F 3 "" H 1350 2800 50  0001 C CNN
	1    1350 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	800  2650 800  2800
Wire Wire Line
	1350 2650 1350 2800
NoConn ~ 1650 1600
NoConn ~ 1650 1700
$Comp
L Device:R R2
U 1 1 5C0849FE
P 3700 1350
F 0 "R2" H 3770 1396 50  0000 L CNN
F 1 "1K" H 3770 1305 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3630 1350 50  0001 C CNN
F 3 "~" H 3700 1350 50  0001 C CNN
	1    3700 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal Y1
U 1 1 5C084A9F
P 4750 2350
F 0 "Y1" H 4750 2618 50  0000 C CNN
F 1 "Crystal" H 4750 2527 50  0000 C CNN
F 2 "Crystal_SMD_5032x_4pin:Crystal_SMD_5032X-4Pin_5.0x3.2mm" H 4750 2350 50  0001 C CNN
F 3 "~" H 4750 2350 50  0001 C CNN
	1    4750 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5C084FD2
P 4300 2600
F 0 "C5" H 4415 2646 50  0000 L CNN
F 1 "22pF" H 4415 2555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4338 2450 50  0001 C CNN
F 3 "~" H 4300 2600 50  0001 C CNN
	1    4300 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5C085074
P 5150 2600
F 0 "C6" H 5265 2646 50  0000 L CNN
F 1 "22pF" H 5265 2555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5188 2450 50  0001 C CNN
F 3 "~" H 5150 2600 50  0001 C CNN
	1    5150 2600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5C0850B6
P 4300 2900
F 0 "#PWR014" H 4300 2650 50  0001 C CNN
F 1 "GND" H 4305 2727 50  0000 C CNN
F 2 "" H 4300 2900 50  0001 C CNN
F 3 "" H 4300 2900 50  0001 C CNN
	1    4300 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 2750 4300 2850
Wire Wire Line
	5150 2750 5150 2850
Wire Wire Line
	4300 2850 5150 2850
Connection ~ 4300 2850
Wire Wire Line
	4300 2850 4300 2900
Wire Wire Line
	4600 2350 4300 2350
Wire Wire Line
	4300 2350 4300 2450
Wire Wire Line
	4300 2350 4300 2100
Wire Wire Line
	4300 2100 2850 2100
Connection ~ 4300 2350
Wire Wire Line
	2850 2000 5150 2000
Wire Wire Line
	5150 2000 5150 2350
Wire Wire Line
	4900 2350 5150 2350
Connection ~ 5150 2350
Wire Wire Line
	5150 2350 5150 2450
Wire Wire Line
	2850 1900 3700 1900
Wire Wire Line
	3700 1900 3700 1500
$Comp
L Device:LED D1
U 1 1 5C086275
P 4100 1300
F 0 "D1" V 4138 1183 50  0000 R CNN
F 1 "LED" V 4047 1183 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 4100 1300 50  0001 C CNN
F 3 "~" H 4100 1300 50  0001 C CNN
	1    4100 1300
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5C0862FE
P 4100 1500
F 0 "#PWR013" H 4100 1250 50  0001 C CNN
F 1 "GND" H 4105 1327 50  0000 C CNN
F 2 "" H 4100 1500 50  0001 C CNN
F 3 "" H 4100 1500 50  0001 C CNN
	1    4100 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 1200 3700 950 
Wire Wire Line
	3700 950  4100 950 
Wire Wire Line
	4100 950  4100 1150
Wire Wire Line
	4100 1450 4100 1500
Text GLabel 3900 1900 2    50   Input ~ 0
SCK
Wire Wire Line
	3700 1900 3900 1900
Connection ~ 3700 1900
Text GLabel 2950 3100 2    50   Input ~ 0
RXD
Wire Wire Line
	2850 3100 2950 3100
Text GLabel 2950 3200 2    50   Input ~ 0
TXD
Wire Wire Line
	2850 3200 2950 3200
Text GLabel 2850 1800 2    50   Input ~ 0
MISO
Text GLabel 2850 1700 2    50   Input ~ 0
MOSI
Text GLabel 3050 1400 2    50   Input ~ 0
PB0
Wire Wire Line
	2850 1400 3050 1400
$Comp
L Connector_Generic:Conn_01x06 J1
U 1 1 5C090DA4
P 1450 5250
F 0 "J1" H 1800 5050 50  0000 C CNN
F 1 "Conn_01x06" H 1800 5200 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 1450 5250 50  0001 C CNN
F 3 "~" H 1450 5250 50  0001 C CNN
	1    1450 5250
	-1   0    0    1   
$EndComp
Text GLabel 1850 5350 2    50   Input ~ 0
TXD
Text GLabel 1850 5250 2    50   Input ~ 0
RXD
Wire Wire Line
	1650 5250 1850 5250
Wire Wire Line
	1650 5350 1850 5350
$Comp
L power:+5V #PWR010
U 1 1 5C092034
P 2400 5050
F 0 "#PWR010" H 2400 4900 50  0001 C CNN
F 1 "+5V" H 2450 5200 50  0000 C CNN
F 2 "" H 2400 5050 50  0001 C CNN
F 3 "" H 2400 5050 50  0001 C CNN
	1    2400 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 5150 2400 5150
Wire Wire Line
	2400 5150 2400 5050
$Comp
L power:GND #PWR07
U 1 1 5C092AE0
P 2150 4700
F 0 "#PWR07" H 2150 4450 50  0001 C CNN
F 1 "GND" H 2155 4527 50  0000 C CNN
F 2 "" H 2150 4700 50  0001 C CNN
F 3 "" H 2150 4700 50  0001 C CNN
	1    2150 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 5050 1950 4950
Wire Wire Line
	1950 4550 2150 4550
Wire Wire Line
	2150 4550 2150 4700
Wire Wire Line
	1650 5050 1950 5050
Wire Wire Line
	1650 4950 1950 4950
Connection ~ 1950 4950
Wire Wire Line
	1950 4950 1950 4550
$Comp
L Device:C C4
U 1 1 5C094DF8
P 2400 5450
F 0 "C4" V 2700 5450 50  0000 C CNN
F 1 "0.1u" V 2600 5450 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2438 5300 50  0001 C CNN
F 3 "~" H 2400 5450 50  0001 C CNN
	1    2400 5450
	0    1    1    0   
$EndComp
Wire Wire Line
	1650 5450 2250 5450
Text GLabel 3000 5450 2    50   Input ~ 0
RESET
Wire Wire Line
	2550 5450 2750 5450
$Comp
L Device:R R1
U 1 1 5C096CB9
P 2750 5100
F 0 "R1" H 2820 5146 50  0000 L CNN
F 1 "10K" H 2820 5055 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2680 5100 50  0001 C CNN
F 3 "~" H 2750 5100 50  0001 C CNN
	1    2750 5100
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR012
U 1 1 5C096E35
P 2750 4850
F 0 "#PWR012" H 2750 4700 50  0001 C CNN
F 1 "+5V" H 2800 5000 50  0000 C CNN
F 2 "" H 2750 4850 50  0001 C CNN
F 3 "" H 2750 4850 50  0001 C CNN
	1    2750 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 4950 2750 4850
Wire Wire Line
	2750 5250 2750 5450
Connection ~ 2750 5450
Wire Wire Line
	2750 5450 3000 5450
$Comp
L Connector_Generic:Conn_01x10 J2
U 1 1 5C098CD3
P 1500 6700
F 0 "J2" H 1850 6600 50  0000 C CNN
F 1 "Conn_01x10" H 1850 6700 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x10_P2.54mm_Vertical" H 1500 6700 50  0001 C CNN
F 3 "~" H 1500 6700 50  0001 C CNN
	1    1500 6700
	-1   0    0    1   
$EndComp
Text GLabel 1700 6200 2    50   Input ~ 0
ADC0
Text GLabel 1700 6300 2    50   Input ~ 0
ADC1
Text GLabel 1700 6400 2    50   Input ~ 0
PB0
Text GLabel 1700 6600 2    50   Input ~ 0
MOSI
Text GLabel 1700 6700 2    50   Input ~ 0
MISO
Text GLabel 1700 6800 2    50   Input ~ 0
SCK
Text GLabel 1700 6900 2    50   Input ~ 0
RESET
$Comp
L power:+5V #PWR011
U 1 1 5C099101
P 2500 6850
F 0 "#PWR011" H 2500 6700 50  0001 C CNN
F 1 "+5V" H 2550 7000 50  0000 C CNN
F 2 "" H 2500 6850 50  0001 C CNN
F 3 "" H 2500 6850 50  0001 C CNN
	1    2500 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 7000 2500 7000
Wire Wire Line
	2500 7000 2500 6850
$Comp
L power:GND #PWR06
U 1 1 5C09A173
P 1950 7350
F 0 "#PWR06" H 1950 7100 50  0001 C CNN
F 1 "GND" H 1955 7177 50  0000 C CNN
F 2 "" H 1950 7350 50  0001 C CNN
F 3 "" H 1950 7350 50  0001 C CNN
	1    1950 7350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 7100 1950 7350
Wire Wire Line
	1950 7100 1700 7100
$Comp
L Connector_Generic:Conn_01x08 J3
U 1 1 5C8ED067
P 3900 4050
F 0 "J3" H 3820 3425 50  0000 C CNN
F 1 "Conn_01x08" H 3820 3516 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical" H 3900 4050 50  0001 C CNN
F 3 "~" H 3900 4050 50  0001 C CNN
	1    3900 4050
	-1   0    0    1   
$EndComp
Text GLabel 4100 3850 2    50   Input ~ 0
SCL
Text GLabel 4100 3950 2    50   Input ~ 0
SDA
Wire Wire Line
	4500 3550 4500 3650
Wire Wire Line
	4500 3650 4100 3650
Wire Wire Line
	4750 3800 4750 3750
Wire Wire Line
	4750 3750 4100 3750
Text GLabel 2850 2800 2    50   Input ~ 0
SCL
Text GLabel 2850 2700 2    50   Input ~ 0
SDA
Text Label 3650 3700 0    50   ~ 0
VCC
Text Label 3650 3800 0    50   ~ 0
GND
Text Label 3650 3900 0    50   ~ 0
SCL
Text Label 3650 4000 0    50   ~ 0
SDA
Text Label 3650 4100 0    50   ~ 0
XDA
Text Label 3650 4200 0    50   ~ 0
XCL
Text Label 3650 4300 0    50   ~ 0
AD0
Text Label 3650 4400 0    50   ~ 0
INT
NoConn ~ 4100 4050
NoConn ~ 4100 4150
NoConn ~ 4100 4350
$Comp
L power:GND #PWR015
U 1 1 5C8FC72E
P 4350 4300
F 0 "#PWR015" H 4350 4050 50  0001 C CNN
F 1 "GND" H 4355 4127 50  0000 C CNN
F 2 "" H 4350 4300 50  0001 C CNN
F 3 "" H 4350 4300 50  0001 C CNN
	1    4350 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 4250 4350 4250
Wire Wire Line
	4350 4250 4350 4300
Text Label 3500 3300 0    50   ~ 0
MPU_Connection
NoConn ~ 2850 3800
NoConn ~ 2850 3700
NoConn ~ 2850 3600
NoConn ~ 2850 3500
NoConn ~ 2850 3400
NoConn ~ 2850 3300
NoConn ~ 2850 2600
NoConn ~ 2850 2500
NoConn ~ 2850 1600
$Comp
L power:+5V #PWR0101
U 1 1 5C97F0B6
P 4500 3550
F 0 "#PWR0101" H 4500 3400 50  0001 C CNN
F 1 "+5V" H 4550 3700 50  0000 C CNN
F 2 "" H 4500 3550 50  0001 C CNN
F 3 "" H 4500 3550 50  0001 C CNN
	1    4500 3550
	1    0    0    -1  
$EndComp
Text GLabel 2950 2300 2    50   Input ~ 0
ADC0
Wire Wire Line
	2850 2300 2950 2300
Text GLabel 2950 2400 2    50   Input ~ 0
ADC1
Wire Wire Line
	2850 2400 2950 2400
Text GLabel 2850 2900 2    50   Input ~ 0
RESET
NoConn ~ 1700 6500
NoConn ~ 2850 1500
$EndSCHEMATC
