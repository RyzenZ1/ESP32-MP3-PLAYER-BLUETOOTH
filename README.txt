This repo contains the code for a ESP32 MP3 BLUETOOTH PLAYER using a custom lib. You can edit the file names to whatever your file name is. The components are mentioned below along with the circuit connections
COMPONENTS:
1.ESP32 DevKit (ESP32‑WROOM‑32)
2.1.8″ ST7735 TFT display (SPI)
3.Analog joystick KY‑023
4.MicroSD card module (SPI, 3.3V)
5.MicroSD card (FAT32)

CONNECTIONS:
ESP32 GPIO	Component
2	        TFT DC
4	        TFT RST
5	        SD CS
13	        SD MOSI
14	        SD SCK
15	        TFT CS
18	        TFT SCK
22	        TFT LED
23	        TFT MOSI
25	        Joystick SW
27        	SD MISO
34        	Joystick VRx
35        	Joystick VRy

🔧 Code Pin Defines
// TFT
#define TFT_CS    15
#define TFT_RST    4
#define TFT_DC     2
#define TFT_LED   22

// SD card
#define SD_CS      5
#define SD_MOSI   13
#define SD_MISO   27
#define SD_SCK    14

// Joystick
#define JOY_X     34
#define JOY_Y     35
#define JOY_SW    25
