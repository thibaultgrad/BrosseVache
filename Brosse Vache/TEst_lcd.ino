// include the library code:
#include <Adafruit_SSD1306.h>
//#include <EEPROM.h>
#include <Wire.h>
//#include <avr/wdt.h>

// initialize the library with the numbers of the interface pins

int temps_brossage = 0;
int state_vue = 0;
int pinbouton = 6;
bool state_bouton = false;
bool prev_state_bouton = false;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
	pinMode(pinbouton, INPUT);
	// Print a message to the LCD.
	// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
	if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {

		for (;;); // Don't proceed, loop forever
	}
	display.display();
	delay(2000);
	display.clearDisplay();
	
}

void loop() {
	// set the cursor to column 0, line 1
	// (note: line 1 is the second row, since counting begins with 0):
	//state_bouton = digitalRead(pinbouton);
	//if (state_bouton == HIGH)
	//{
	//	state_vue += 1;
	//	prev_state_bouton = state_bouton;
	//}
	//switch (state_vue)
	//{
	//	case 0:
	//		lcd.clear;
	//		temps_brossage = analogRead(0);
	//	lcd.print("Temps brossage");
	//	lcd.setCursor(0, 1);
	//	// print the number of seconds since reset:
	//	lcd.print(millis() / 1000);
	//	lcd.setCursor(5, 1);
	//	lcd.print(temps_brossage);
	//	delay(1000);
	//	case 1:
	//		temps_brossage = analogRead(0);
	//		lcd.print("Angle inclinaison");
	//		lcd.setCursor(0, 1);
	//		// print the number of seconds since reset:
	//		lcd.print(millis() / 1000);
	//		lcd.setCursor(5, 1);
	//		lcd.print(temps_brossage);
	//		delay(1000);
	//}

}
