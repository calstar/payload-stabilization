EESchema Schematic File Version 4
EELAYER 30 0
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
L Relay_SolidState:TLP3543 U?
U 1 1 5EAC0E5E
P 2600 3500
F 0 "U?" H 2600 3825 50  0001 C CNN
F 1 "TLP3543" H 2600 3734 50  0000 C CNN
F 2 "Package_DIP:DIP-6_W7.62mm" H 2600 3200 50  0001 C CNN
F 3 "https://toshiba.semicon-storage.com/info/docget.jsp?did=12660&prodName=TLP3543" H 2600 3500 50  0001 C CNN
	1    2600 3500
	-1   0    0    1   
$EndComp
$Comp
L Device:R R
U 1 1 5EAC2D4E
P 3150 3400
F 0 "R" V 2943 3400 50  0000 C CNN
F 1 "336 Ohms" V 3034 3400 50  0000 C CNN
F 2 "" V 3080 3400 50  0001 C CNN
F 3 "~" H 3150 3400 50  0001 C CNN
	1    3150 3400
	0    1    1    0   
$EndComp
$Comp
L Device:Battery_Cell LiPo_Battery_Single_Cell
U 1 1 5EAC5DDB
P 1300 7600
F 0 "LiPo_Battery_Single_Cell" V 1555 7650 50  0000 C CNN
F 1 "3.7V" V 1464 7650 50  0000 C CNN
F 2 "" V 1300 7660 50  0001 C CNN
F 3 "~" V 1300 7660 50  0001 C CNN
	1    1300 7600
	0    -1   -1   0   
$EndComp
$Comp
L Motor:Motor_Servo M?
U 1 1 5EAC9006
P 3350 1900
F 0 "M?" H 3682 1965 50  0001 L CNN
F 1 "Motor_Servo" V 3682 1874 50  0000 C CNN
F 2 "" H 3350 1710 50  0001 C CNN
F 3 "http://forums.parallax.com/uploads/attachments/46831/74481.png" H 3350 1710 50  0001 C CNN
	1    3350 1900
	0    1    1    0   
$EndComp
$Comp
L Motor:Motor_Servo M?
U 1 1 5EACA814
P 2600 1900
F 0 "M?" H 2932 1965 50  0001 L CNN
F 1 "Motor_Servo" V 2932 1874 50  0000 C CNN
F 2 "" H 2600 1710 50  0001 C CNN
F 3 "http://forums.parallax.com/uploads/attachments/46831/74481.png" H 2600 1710 50  0001 C CNN
	1    2600 1900
	0    1    1    0   
$EndComp
$Comp
L Motor:Motor_Servo M?
U 1 1 5EACBB6C
P 1850 1900
F 0 "M?" H 2182 1965 50  0001 L CNN
F 1 "Motor_Servo" V 2182 1874 50  0000 C CNN
F 2 "" H 1850 1710 50  0001 C CNN
F 3 "http://forums.parallax.com/uploads/attachments/46831/74481.png" H 1850 1710 50  0001 C CNN
	1    1850 1900
	0    1    1    0   
$EndComp
Wire Wire Line
	1100 7600 1000 7600
Wire Wire Line
	1400 7600 1450 7600
Wire Wire Line
	1000 7600 1000 7250
Wire Wire Line
	1450 7600 1450 7100
Wire Wire Line
	1000 1000 1850 1000
Wire Wire Line
	1000 1000 1000 5750
Wire Wire Line
	9750 1000 9750 1550
Wire Wire Line
	8050 1550 8050 1000
Connection ~ 8050 1000
Wire Wire Line
	8050 1000 9750 1000
Wire Wire Line
	6050 1550 6050 1000
Connection ~ 6050 1000
Wire Wire Line
	6050 1000 7450 1000
Wire Wire Line
	9750 1850 9150 1850
Wire Wire Line
	9150 1850 9150 3300
Wire Wire Line
	9150 3300 5400 3300
Wire Wire Line
	9400 3900 9400 1950
Wire Wire Line
	9400 1950 9750 1950
Wire Wire Line
	9550 4050 9550 2050
Wire Wire Line
	9550 2050 9750 2050
Wire Wire Line
	9700 4200 9700 2150
Wire Wire Line
	9700 2150 9750 2150
Wire Wire Line
	9750 1750 9000 1750
Wire Wire Line
	9000 1750 9000 5000
Wire Wire Line
	5400 2800 5600 2800
Wire Wire Line
	5600 2800 5600 1850
Wire Wire Line
	5600 1850 6050 1850
Wire Wire Line
	5600 2800 7550 2800
Wire Wire Line
	7550 2800 7550 1850
Wire Wire Line
	7550 1850 8050 1850
Connection ~ 5600 2800
Wire Wire Line
	5400 2900 5850 2900
Wire Wire Line
	5850 2900 5850 1750
Wire Wire Line
	5850 1750 6050 1750
Wire Wire Line
	5850 2900 7850 2900
Wire Wire Line
	7850 2900 7850 1750
Wire Wire Line
	7850 1750 8050 1750
Connection ~ 5850 2900
Wire Wire Line
	6050 1650 5950 1650
Wire Wire Line
	8050 1650 7950 1650
Wire Wire Line
	7950 1650 7950 5000
Wire Wire Line
	5950 1650 5950 2150
Wire Wire Line
	1000 7250 750  7250
Wire Wire Line
	750  7250 750  750 
Wire Wire Line
	750  750  5500 750 
Wire Wire Line
	5500 750  5500 2400
Wire Wire Line
	5500 2400 5400 2400
Connection ~ 1000 7250
Wire Wire Line
	1000 7250 1000 7100
Wire Wire Line
	1850 1000 1850 1600
Connection ~ 1850 1000
Wire Wire Line
	1850 1000 2600 1000
Wire Wire Line
	2600 1000 2600 1600
Connection ~ 2600 1000
Wire Wire Line
	2600 1000 3350 1000
Wire Wire Line
	3350 1000 3350 1600
Connection ~ 3350 1000
Wire Wire Line
	3350 1000 4800 1000
Wire Wire Line
	1750 1600 1550 1600
Wire Wire Line
	1550 1600 1550 2500
Wire Wire Line
	2500 1600 2300 1600
Wire Wire Line
	2300 1600 2300 2500
Wire Wire Line
	3250 1600 3050 1600
Wire Wire Line
	3050 1600 3050 2500
Wire Wire Line
	1450 5750 1450 5000
Wire Wire Line
	1450 5000 2000 5000
Wire Wire Line
	2300 3500 2000 3500
Wire Wire Line
	2000 3500 2000 5000
Wire Wire Line
	3050 2500 2300 2500
Connection ~ 2300 2500
Wire Wire Line
	2300 2500 1550 2500
Wire Wire Line
	1550 2500 1550 3400
Wire Wire Line
	1550 3600 2300 3600
Connection ~ 1550 2500
Wire Wire Line
	2300 3400 1550 3400
Connection ~ 1550 3400
Wire Wire Line
	1550 3400 1550 3600
Wire Wire Line
	4000 3600 2900 3600
Wire Wire Line
	2900 3400 3000 3400
Wire Wire Line
	1950 1600 1950 1400
Wire Wire Line
	2700 1600 2700 1500
Wire Wire Line
	3450 1600 3600 1600
Wire Wire Line
	2700 1500 3700 1500
Wire Wire Line
	1950 1400 3800 1400
Wire Wire Line
	9000 5000 7950 5000
Connection ~ 2000 5000
Connection ~ 5000 5000
Wire Wire Line
	5000 5000 3400 5000
Connection ~ 5950 5000
Wire Wire Line
	5950 5000 5000 5000
Connection ~ 7950 5000
Wire Wire Line
	3400 3400 3400 5000
Connection ~ 3400 5000
Wire Wire Line
	2000 5000 3400 5000
Wire Wire Line
	3300 3400 3400 3400
Wire Wire Line
	4800 1000 4800 1400
Connection ~ 4800 1000
Wire Wire Line
	4800 1000 6050 1000
$Comp
L arduino_nano_stab:Arduino_Nano_v3.x_copy A?
U 1 1 604F3D0A
P 4900 2400
F 0 "A?" H 4900 1311 50  0000 C CNN
F 1 "Arduino_Nano_v3.x_copy" H 4900 1220 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 4900 2400 50  0001 C CIN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 4900 2400 50  0001 C CNN
	1    4900 2400
	1    0    0    -1  
$EndComp
Connection ~ 4800 1400
Wire Wire Line
	4800 1400 4800 1500
$Comp
L arduino_nano_stab:GY-521_MPU-6050 U?
U 1 1 604FD407
P 6450 2350
F 0 "U?" H 6778 2851 50  0001 L CNN
F 1 "GY-521_MPU-6050" V 6778 2760 50  0000 C CNN
F 2 "" H 6450 2350 50  0001 C CNN
F 3 "" H 6450 2350 50  0001 C CNN
	1    6450 2350
	1    0    0    -1  
$EndComp
$Comp
L arduino_nano_stab:GY-521_MPU-6050 U?
U 1 1 604FE090
P 8450 2350
F 0 "U?" H 8778 2851 50  0001 L CNN
F 1 "GY-521_MPU-6050" V 8778 2760 50  0000 C CNN
F 2 "" H 8450 2350 50  0001 C CNN
F 3 "" H 8450 2350 50  0001 C CNN
	1    8450 2350
	1    0    0    -1  
$EndComp
$Comp
L arduino_nano_stab:Micro-SD_Breakout U?
U 1 1 605046FA
P 10350 2350
F 0 "U?" H 10878 2851 50  0000 L CNN
F 1 "Micro-SD_Breakout" H 10878 2760 50  0000 L CNN
F 2 "" H 10350 2350 50  0001 C CNN
F 3 "" H 10350 2350 50  0001 C CNN
	1    10350 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 2250 5700 2250
Wire Wire Line
	5700 2250 5700 1300
Wire Wire Line
	5700 1300 4300 1300
Wire Wire Line
	4300 1300 4300 2000
Wire Wire Line
	4300 2000 4400 2000
Wire Wire Line
	4400 2100 4200 2100
Wire Wire Line
	4200 2100 4200 1200
Wire Wire Line
	4200 1200 7750 1200
Wire Wire Line
	7750 1200 7750 2250
Wire Wire Line
	7750 2250 8050 2250
$Comp
L arduino_nano_stab:Booster_Converter U?
U 1 1 6051CE62
P 1500 6450
F 0 "U?" V 1571 6472 50  0000 R CNN
F 1 "Booster_Converter" V 1480 6472 50  0000 R CNN
F 2 "" H 1500 6450 50  0001 C CNN
F 3 "" H 1500 6450 50  0001 C CNN
	1    1500 6450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1000 7100 1050 7100
Wire Wire Line
	1400 7100 1450 7100
Wire Wire Line
	1450 5750 1400 5750
Wire Wire Line
	1000 5750 1050 5750
Wire Wire Line
	5000 3400 5000 5000
Wire Wire Line
	4400 3100 4400 3500
Wire Wire Line
	4400 3500 5400 3500
Wire Wire Line
	5400 3500 5400 3300
Wire Wire Line
	4400 3000 4300 3000
Wire Wire Line
	4300 3000 4300 3900
Wire Wire Line
	4300 3900 9400 3900
Wire Wire Line
	4400 2900 4200 2900
Wire Wire Line
	4200 2900 4200 4050
Wire Wire Line
	4200 4050 9550 4050
Wire Wire Line
	4100 4200 4100 2800
Wire Wire Line
	4100 2800 4400 2800
Wire Wire Line
	4100 4200 9700 4200
Wire Wire Line
	4000 3600 4000 2700
Wire Wire Line
	4000 2700 4400 2700
Wire Wire Line
	3800 1400 3800 2200
Wire Wire Line
	3800 2200 4400 2200
Wire Wire Line
	3700 1500 3700 2300
Wire Wire Line
	3700 2300 4400 2300
Wire Wire Line
	3600 1600 3600 2400
Wire Wire Line
	3600 2400 4400 2400
Wire Wire Line
	7950 5000 5950 5000
Wire Wire Line
	6050 2150 5950 2150
Connection ~ 5950 2150
Wire Wire Line
	5950 2150 5950 5000
Wire Wire Line
	8050 2150 7450 2150
Wire Wire Line
	7450 2150 7450 1000
Connection ~ 7450 1000
Wire Wire Line
	7450 1000 8050 1000
$EndSCHEMATC
