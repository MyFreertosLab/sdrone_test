EESchema Schematic File Version 4
LIBS:axis2-cache
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
L axis2-rescue:Conn_02x10_Odd_Even-Connector_Generic J1
U 1 1 6072B308
P 3000 4700
F 0 "J1" H 3050 5317 50  0000 C CNN
F 1 "JA" H 3050 5226 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x10_P2.54mm_Vertical" H 3000 4700 50  0001 C CNN
F 3 "~" H 3000 4700 50  0001 C CNN
	1    3000 4700
	1    0    0    -1  
$EndComp
$Comp
L axis2-rescue:Conn_02x10_Odd_Even-Connector_Generic J2
U 1 1 6072B4E0
P 6500 4700
F 0 "J2" H 6550 5317 50  0000 C CNN
F 1 "JB" H 6550 5226 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x10_P2.54mm_Vertical" H 6500 4700 50  0001 C CNN
F 3 "~" H 6500 4700 50  0001 C CNN
	1    6500 4700
	1    0    0    -1  
$EndComp
NoConn ~ 2800 4400
NoConn ~ 2800 4500
NoConn ~ 2800 4600
NoConn ~ 2800 4700
NoConn ~ 2800 4800
NoConn ~ 2800 5100
NoConn ~ 2800 5200
NoConn ~ 3300 5200
NoConn ~ 3300 5100
NoConn ~ 3300 5000
NoConn ~ 3300 4800
NoConn ~ 3300 4700
NoConn ~ 3300 4600
NoConn ~ 3300 4500
NoConn ~ 3300 4400
NoConn ~ 3300 4300
NoConn ~ 6300 4300
NoConn ~ 6300 4400
NoConn ~ 6300 4500
NoConn ~ 6300 4600
NoConn ~ 6300 4800
NoConn ~ 6300 4900
NoConn ~ 6300 5100
NoConn ~ 6300 5200
NoConn ~ 6800 5200
NoConn ~ 6800 5100
$Comp
L gy-9250:gy-9250 U1
U 1 1 6073C1B2
P 4900 4800
F 0 "U1" H 4900 5700 50  0000 L CNN
F 1 "gy-9250" V 5200 4750 50  0000 L CNN
F 2 "gy-9250_footprint:gy-9250" H 4900 5050 50  0001 C CNN
F 3 "" H 4900 5050 50  0001 C CNN
	1    4900 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 4900 2800 4900
Text GLabel 4350 5000 0    50   Output ~ 0
MISO
Text GLabel 2650 4900 0    50   Input ~ 0
MISO
Wire Wire Line
	2650 5000 2800 5000
Text GLabel 4350 4550 0    50   Input ~ 0
MOSI
Text GLabel 3500 4900 2    50   Output ~ 0
SCL
Wire Wire Line
	3300 4900 3500 4900
Text GLabel 2650 5000 0    50   Output ~ 0
MOSI
Text GLabel 2600 4300 0    50   Output ~ 0
+3V3
Wire Wire Line
	2600 4300 2800 4300
Text GLabel 4350 4100 0    50   Input ~ 0
+3V3
Text GLabel 6800 5000 2    50   Output ~ 0
NCS
Text GLabel 4350 5300 0    50   Input ~ 0
NCS
NoConn ~ 6800 4900
NoConn ~ 6800 4700
NoConn ~ 6800 4500
NoConn ~ 6800 4400
Text GLabel 6300 5000 0    50   Output ~ 0
INT
Text GLabel 4350 5150 0    50   Input ~ 0
INT
Text GLabel 4350 4400 0    50   Input ~ 0
SCL
NoConn ~ 4350 5450
NoConn ~ 4350 4700
NoConn ~ 4350 4850
$Comp
L axis2-rescue:L7805-Regulator_Linear U2
U 1 1 60754FE6
P 5400 1500
F 0 "U2" H 5400 1742 50  0000 C CNN
F 1 "L7805" H 5400 1651 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 5425 1350 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 5400 1450 50  0001 C CNN
	1    5400 1500
	1    0    0    -1  
$EndComp
Text GLabel 2700 1100 1    50   Input ~ 0
+12V
Wire Wire Line
	2700 1350 2700 1100
Wire Wire Line
	2700 1900 2700 1650
Text GLabel 4800 1500 0    50   Input ~ 0
+12V
$Comp
L axis2-rescue:GND-power #PWR0101
U 1 1 60756A5D
P 1550 1100
F 0 "#PWR0101" H 1550 850 50  0001 C CNN
F 1 "GND" H 1555 927 50  0000 C CNN
F 2 "" H 1550 1100 50  0001 C CNN
F 3 "" H 1550 1100 50  0001 C CNN
	1    1550 1100
	-1   0    0    1   
$EndComp
$Comp
L axis2-rescue:+12V-power #PWR0102
U 1 1 60756DB1
P 1050 1100
F 0 "#PWR0102" H 1050 950 50  0001 C CNN
F 1 "+12V" H 1065 1273 50  0000 C CNN
F 2 "" H 1050 1100 50  0001 C CNN
F 3 "" H 1050 1100 50  0001 C CNN
	1    1050 1100
	1    0    0    -1  
$EndComp
$Comp
L axis2-rescue:PWR_FLAG-power #FLG02
U 1 1 607576CC
P 1550 1400
F 0 "#FLG02" H 1550 1475 50  0001 C CNN
F 1 "PWR_FLAG" H 1550 1573 50  0000 C CNN
F 2 "" H 1550 1400 50  0001 C CNN
F 3 "~" H 1550 1400 50  0001 C CNN
	1    1550 1400
	-1   0    0    1   
$EndComp
$Comp
L axis2-rescue:PWR_FLAG-power #FLG01
U 1 1 60757862
P 1050 1400
F 0 "#FLG01" H 1050 1475 50  0001 C CNN
F 1 "PWR_FLAG" H 1050 1573 50  0000 C CNN
F 2 "" H 1050 1400 50  0001 C CNN
F 3 "~" H 1050 1400 50  0001 C CNN
	1    1050 1400
	-1   0    0    1   
$EndComp
Wire Wire Line
	1050 1400 1050 1100
Wire Wire Line
	1550 1400 1550 1100
$Comp
L axis2-rescue:CP1-Device C1
U 1 1 607588D9
P 2700 1500
F 0 "C1" H 2815 1546 50  0000 L CNN
F 1 "100uF" H 2815 1455 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 2700 1500 50  0001 C CNN
F 3 "~" H 2700 1500 50  0001 C CNN
	1    2700 1500
	1    0    0    -1  
$EndComp
$Comp
L axis2-rescue:CP1-Device C2
U 1 1 60758924
P 4450 2350
F 0 "C2" H 4565 2396 50  0000 L CNN
F 1 "100uF" H 4565 2305 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 4450 2350 50  0001 C CNN
F 3 "~" H 4450 2350 50  0001 C CNN
	1    4450 2350
	1    0    0    -1  
$EndComp
$Comp
L axis2-rescue:Conn_01x02_Female-Connector J3
U 1 1 60759B12
P 4850 1950
F 0 "J3" V 4790 1762 50  0000 R CNN
F 1 "5VPower" V 4900 2000 50  0000 R CNN
F 2 "Connector_JST:JST_XH_B2B-XH-AM_1x02_P2.50mm_Vertical" H 4850 1950 50  0001 C CNN
F 3 "~" H 4850 1950 50  0001 C CNN
	1    4850 1950
	0    -1   -1   0   
$EndComp
$Comp
L axis2-rescue:Conn_01x02_Female-Connector J4
U 1 1 6075A23A
P 3250 1000
F 0 "J4" V 3190 812 50  0000 R CNN
F 1 "12VPower" V 3099 812 50  0000 R CNN
F 2 "Connector_BarrelJack:BarrelJack_Wuerth_6941xx301002" H 3250 1000 50  0001 C CNN
F 3 "~" H 3250 1000 50  0001 C CNN
	1    3250 1000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4800 1500 5100 1500
Text Label 5400 2050 1    50   ~ 0
GND
Wire Wire Line
	5400 1800 5400 2050
Text Label 4250 4250 2    50   ~ 0
GND
Wire Wire Line
	4350 4250 4250 4250
Text Label 7050 4800 2    50   ~ 0
GND
Wire Wire Line
	6800 4800 7050 4800
Text Label 3250 1500 1    50   ~ 0
GND
Wire Wire Line
	3250 1200 3250 1500
Text GLabel 3350 1350 3    50   Input ~ 0
+12V
Wire Wire Line
	3350 1350 3350 1200
Text GLabel 4850 2500 3    50   Input ~ 0
+5V
Text Label 5000 2550 3    50   ~ 0
GND
Text Label 4450 2800 1    50   ~ 0
GND
Wire Wire Line
	4450 2500 4450 2800
Text GLabel 6050 1500 2    50   Output ~ 0
+5V
Wire Wire Line
	5700 1500 6050 1500
Text GLabel 4450 2100 1    50   Input ~ 0
+5V
Wire Wire Line
	4450 2200 4450 2100
Text Label 2700 1900 1    50   ~ 0
GND
$Comp
L axis2-rescue:Heatsink_Pad_2Pin-Mechanical HS1
U 1 1 60765A1F
P 5400 1000
F 0 "HS1" H 5587 1041 50  0000 L CNN
F 1 "Heatsink_Pad_2Pin" H 5587 950 50  0000 L CNN
F 2 "Heatsink:Heatsink_Stonecold_HS-132_32x14mm_2xFixation1.5mm" H 5412 950 50  0001 C CNN
F 3 "~" H 5412 950 50  0001 C CNN
	1    5400 1000
	1    0    0    -1  
$EndComp
NoConn ~ 5300 1100
NoConn ~ 5500 1100
NoConn ~ 6350 2000
$Comp
L dk_Rectangular-Connectors-Headers-Male-Pins:22-23-2021 J5
U 1 1 607713B7
P 7000 1000
F 0 "J5" H 6912 960 50  0000 R CNN
F 1 "Power Motor 1" H 6912 1051 50  0000 R CNN
F 2 "Connector:Deans" H 7200 1200 60  0001 L CNN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/Molex%20PDFs/A-6373-N_Series_Dwg_2010-12-03.pdf" H 7200 1300 60  0001 L CNN
F 4 "WM4200-ND" H 7200 1400 60  0001 L CNN "Digi-Key_PN"
F 5 "22-23-2021" H 7200 1500 60  0001 L CNN "MPN"
F 6 "Connectors, Interconnects" H 7200 1600 60  0001 L CNN "Category"
F 7 "Rectangular Connectors - Headers, Male Pins" H 7200 1700 60  0001 L CNN "Family"
F 8 "https://media.digikey.com/pdf/Data%20Sheets/Molex%20PDFs/A-6373-N_Series_Dwg_2010-12-03.pdf" H 7200 1800 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/molex/22-23-2021/WM4200-ND/26667" H 7200 1900 60  0001 L CNN "DK_Detail_Page"
F 10 "CONN HEADER VERT 2POS 2.54MM" H 7200 2000 60  0001 L CNN "Description"
F 11 "Molex" H 7200 2100 60  0001 L CNN "Manufacturer"
F 12 "Active" H 7200 2200 60  0001 L CNN "Status"
	1    7000 1000
	-1   0    0    1   
$EndComp
$Comp
L dk_Rectangular-Connectors-Headers-Male-Pins:22-23-2021 J6
U 1 1 6075CB28
P 7950 1000
F 0 "J6" H 7862 960 50  0000 R CNN
F 1 "Power Motor 2" H 7862 1051 50  0000 R CNN
F 2 "Connector:Deans" H 8150 1200 60  0001 L CNN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/Molex%20PDFs/A-6373-N_Series_Dwg_2010-12-03.pdf" H 8150 1300 60  0001 L CNN
F 4 "WM4200-ND" H 8150 1400 60  0001 L CNN "Digi-Key_PN"
F 5 "22-23-2021" H 8150 1500 60  0001 L CNN "MPN"
F 6 "Connectors, Interconnects" H 8150 1600 60  0001 L CNN "Category"
F 7 "Rectangular Connectors - Headers, Male Pins" H 8150 1700 60  0001 L CNN "Family"
F 8 "https://media.digikey.com/pdf/Data%20Sheets/Molex%20PDFs/A-6373-N_Series_Dwg_2010-12-03.pdf" H 8150 1800 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/molex/22-23-2021/WM4200-ND/26667" H 8150 1900 60  0001 L CNN "DK_Detail_Page"
F 10 "CONN HEADER VERT 2POS 2.54MM" H 8150 2000 60  0001 L CNN "Description"
F 11 "Molex" H 8150 2100 60  0001 L CNN "Manufacturer"
F 12 "Active" H 8150 2200 60  0001 L CNN "Status"
	1    7950 1000
	-1   0    0    1   
$EndComp
Text GLabel 7000 1350 3    50   Input ~ 0
+12V
Text GLabel 6900 1350 3    50   Input ~ 0
GND
Wire Wire Line
	7000 1350 7000 1100
Wire Wire Line
	6900 1350 6900 1100
Text GLabel 7950 1350 3    50   Input ~ 0
+12V
Wire Wire Line
	7950 1350 7950 1100
Text GLabel 7850 1350 3    50   Input ~ 0
GND
Wire Wire Line
	7850 1350 7850 1100
Wire Notes Line
	6650 700  6650 1650
Wire Notes Line
	6650 1650 8700 1650
Wire Notes Line
	8700 1650 8700 700 
Wire Notes Line
	8700 700  6650 700 
Text Notes 8300 1550 0    50   ~ 0
Power\nMotors
Wire Notes Line
	2200 700  2200 2100
Wire Notes Line
	2200 2100 3900 2100
Wire Notes Line
	3900 2100 3900 700 
Wire Notes Line
	3900 700  2200 700 
Text Notes 3350 1950 0    50   ~ 0
Power 12V
Wire Notes Line
	4100 700  6450 700 
Wire Notes Line
	6450 700  6450 2900
Wire Notes Line
	6450 2900 4100 2900
Wire Notes Line
	4100 2900 4100 700 
Text Notes 5950 2700 0    50   ~ 0
Power 5V
Wire Notes Line
	2150 5750 7300 5750
Wire Notes Line
	7300 5750 7300 3550
Wire Notes Line
	7300 3550 2150 3550
Wire Notes Line
	2150 3550 2150 5750
Text Notes 6450 5600 0    50   ~ 0
Controller Board
$Comp
L Connector_Generic:Conn_01x03 J7
U 1 1 6075F9C4
P 7150 2150
F 0 "J7" H 7230 2192 50  0000 L CNN
F 1 "M1" H 7230 2101 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 7150 2150 50  0001 C CNN
F 3 "~" H 7150 2150 50  0001 C CNN
	1    7150 2150
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J8
U 1 1 6075FA63
P 8200 2150
F 0 "J8" H 8280 2192 50  0000 L CNN
F 1 "M2" H 8280 2101 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 8200 2150 50  0001 C CNN
F 3 "~" H 8200 2150 50  0001 C CNN
	1    8200 2150
	1    0    0    -1  
$EndComp
Wire Notes Line
	6500 1850 6500 2500
Wire Notes Line
	6500 2500 8550 2500
Wire Notes Line
	8550 2500 8550 1850
Wire Notes Line
	8550 1850 6500 1850
Text Notes 7850 2450 0    50   ~ 0
Motors Control
Wire Wire Line
	6800 2050 6950 2050
Wire Wire Line
	7800 2050 8000 2050
Wire Wire Line
	6800 2150 6950 2150
Wire Wire Line
	7800 2150 8000 2150
Text GLabel 6900 4600 2    50   Output ~ 0
M1S
Wire Wire Line
	6800 4600 6900 4600
Text GLabel 6800 2250 0    50   Input ~ 0
M1S
Wire Wire Line
	6800 2250 6950 2250
Text GLabel 7800 2250 0    50   Input ~ 0
M2S
Wire Wire Line
	7800 2250 8000 2250
Text GLabel 6200 4700 0    50   Output ~ 0
M2S
Wire Wire Line
	6300 4700 6200 4700
Wire Wire Line
	4850 2150 4850 2500
Wire Wire Line
	4950 2550 5000 2550
Wire Wire Line
	4950 2150 4950 2550
Text GLabel 6900 4300 2    50   Input ~ 0
RCS
Wire Wire Line
	6900 4300 6800 4300
Text GLabel 2650 2650 0    50   Input ~ 0
RCS
Wire Notes Line
	2200 2250 2200 2800
Wire Notes Line
	2200 2800 3850 2800
Wire Notes Line
	3850 2800 3850 2250
Wire Notes Line
	3850 2250 2200 2250
Text Notes 3600 2700 0    50   ~ 0
RC
$Comp
L Connector_Generic:Conn_01x03 J9
U 1 1 60762012
P 3050 2550
F 0 "J9" H 3130 2592 50  0000 L CNN
F 1 "RC-PPM" H 3130 2501 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 3050 2550 50  0001 C CNN
F 3 "~" H 3050 2550 50  0001 C CNN
	1    3050 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 2650 2850 2650
Text Label 2600 2450 2    50   ~ 0
GND
Wire Wire Line
	2600 2450 2850 2450
Text GLabel 2650 2550 0    50   Input ~ 0
+5V
Wire Wire Line
	2650 2550 2850 2550
$Comp
L axis2-symbols:axis2-frame U3
U 1 1 6076B53D
P 7250 2900
F 0 "U3" H 7428 3046 50  0000 L CNN
F 1 "axis2-frame" H 7428 2955 50  0000 L CNN
F 2 "axis2:axis2-footprint" H 7250 2900 50  0001 C CNN
F 3 "" H 7250 2900 50  0001 C CNN
	1    7250 2900
	1    0    0    -1  
$EndComp
NoConn ~ 6800 2050
NoConn ~ 6800 2150
NoConn ~ 7800 2050
NoConn ~ 7800 2150
NoConn ~ 5400 2450
NoConn ~ 4850 2550
$EndSCHEMATC
