
//Gyro - Arduino UNO R3
//VCC  -  5V
//GND  -  GND
//SDA  -  A4
//SCL  -  A5
//INT - port-2


//#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <Wire.h>
#include <avr/wdt.h>

//Declaring some global variables
int gyro_x, gyro_y, gyro_z;

boolean set_gyro_angles;

long acc_x, acc_y, acc_z;
int angle_brut = 0;


long loop_timer;
int temp;

// ----------------------

long t_debut_etat;
long duree_etat;
long temps_I_MAX=0;
long temps_I_MIN=0;
int I_MAX=0;
int I_MIN=0;
long angle_mesure;
boolean etat_moteur;
#define pin_moteur_relais1 9
#define pin_moteur_relais2 10
#define pin_moteur_inversion1 11
#define pin_moteur_inversion2 12

bool SensRotation = true;

//#define pin_seuil_inclinaison A1
//#define pin_seuil_courant A2
#define pinCourant A2
#define pinaAdjustValueY A0
#define pinaAdjustValueX A1

//int angle_declenchement = 20;
boolean etat_courant;
// periode d'échantillonnage en ms
#define MS_PERIOD_ECH  2

// temps de moyennage du courant
#define MS_MOYENNE    70
//#define MS_BROSSAGE 1000000 // en microseconde pour test
//int MS_BROSSAGE = 120000000; //en microseconde
#define MS_SURCOURANT 3000000 //en microseconde
#define MS_DEMARRAGE_MOTEUR 1500000 //en microseconde

int i = 0;
#define TAILLE_TABLEAU_ECHANTILLONS   (MS_MOYENNE / MS_PERIOD_ECH)
//int echantillon_angle[TAILLE_TABLEAU_ECHANTILLONS] = { 0 };
//
//int indexEchantillonsCourant = 0;
int tension = 0;
//int courant_max = 1300;
enum Etats
{
	Attentedemarrage=0,
	Demarrage,
	Brossage,
	Arret,
};

int buttonpin = 4;
bool buttonstate = false;
bool prevbuttonstate = false;
int vue_affichage = 0;
int prev_vue = 1;
bool issetting = false;
bool initialised = false;
Etats etat = Attentedemarrage; //0=attente demarrge , 1= demarrage, 2=moteur tourne, 3=arret
//unsigned int temps_total_brossage = 0; // en minutes


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/** Le nombre magique et le numéro de version actuelle */
static const unsigned long STRUCT_MAGIC = 123456789;
static const byte STRUCT_VERSION = 2;

struct SavedDatasStruct
{
	unsigned long magic;
	byte struct_version;
	unsigned int temps_total_brossage;
	int angle_declenchement;
	int courant_max;
	unsigned long MS_BROSSAGE;
};
struct SerialExchangeData
{
	byte type;
	unsigned int temps_total_brossage;
	int angle_declenchement;
	int courant_max;
	unsigned long MS_BROSSAGE;
	unsigned int angle_mesure;
	int courant;
};

SavedDatasStruct SavedDatas;
SerialExchangeData ExchangedDatas;
int indexechantilon = 0;
uint8_t echantillon_angle[TAILLE_TABLEAU_ECHANTILLONS] = { 0 };

float gravity = 9.81f;

void setup()
{
	Serial.begin(74880);

	wdt_disable();
	wdt_enable(WDTO_8S);
	pinMode(pin_moteur_relais1, OUTPUT);
	pinMode(pin_moteur_relais2, OUTPUT);
	pinMode(buttonpin, INPUT_PULLUP);
	pinMode(pinaAdjustValueX, INPUT);
	pinMode(pinaAdjustValueY, INPUT);
	pinMode(pinCourant, INPUT);
	digitalWrite(pin_moteur_relais1, HIGH);
	digitalWrite(pin_moteur_relais2, HIGH);
	analogReference(DEFAULT);

	// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
	if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
		
		for (;;); // Don't proceed, loop forever
	}
	display.display();
	//delay(2000);
	display.clearDisplay();
	display.setTextColor(WHITE);
	display.setTextSize(1);
	display.cp437(true);


	t_debut_etat = micros();
	etat_moteur = 0;
	Wire.begin();                                                        //Start I2C as master
	setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 

	Serial.begin(74880);
	loop_timer = micros();                                               //Reset the loop timer
	vue_affichage = 5;
}

void loop()
{
	bool raz = false;
	if (!initialised) {
		while (digitalRead(buttonpin) && loop_timer + 30000000l > micros())
		{
			display.clearDisplay();
			display.setCursor(0, 0);
			display.print("Vider la memoire ?");
			int val = map(analogRead(pinaAdjustValueY), 0, 1023, -1000, 1000);
			if (val > 500) raz = true;
			if (val < -500) raz = false;
			display.setCursor(0, 16);
			display.print(raz ? "Oui" : "Non");
			display.display();
			delay(5);
			wdt_reset();
		}
		buttonstate = digitalRead(buttonpin);
		prevbuttonstate = buttonstate;
		if (raz) {
			for (int i = 0; i < EEPROM.length(); i++) {
				EEPROM.write(i, 0);
			}
		}
		chargeEEPROM();
		loop_timer = millis();
		while (digitalRead(buttonpin) && loop_timer + 30000000l > micros())
		{
			display.clearDisplay();
			display.setCursor(0, 0);
			display.print("Prise de zero angle ?");
			display.setCursor(1, 0);
			display.print("Cliquer pour prise zero");
			display.display();
			delay(5);
			read_mpu_6050_data();
			wdt_reset();
		}
		gravity = acc_z;

		initialised = true;
	}
//#pragma region SerialCom
//	unsigned long uBufSize = sizeof(SavedDatas);
//	char pBuffer[uBufSize];
//
//	memcpy(pBuffer, &SavedDatas, uBufSize);
//	for (int i = 0; i < uBufSize; i++) {
//		Serial.print(pBuffer[i]);
//	}
//#pragma endregion



	read_mpu_6050_data();

	angle_brut = acos((float)acc_z / gravity) * 57.296;       //Calculate the pitch angle

	Echantillonnageangle();
	duree_etat = abs(micros() - t_debut_etat);
	switch (etat)
	{
		case 0:
		if (angle_mesure >= SavedDatas.angle_declenchement)
		{
			etat = Demarrage;
			t_debut_etat = micros();
			issetting = 0;
		}
		break;
		case 1:
		if (etat_moteur == 0) {MoteursOn();}
		//echantillonnagecourant();
		//if (tension > SavedDatas.courant_max) {
		//	etat = 3;
		//	t_debut_etat = micros();
		//}
		if ((duree_etat) > MS_DEMARRAGE_MOTEUR)
		{
			etat =Brossage;
			t_debut_etat = micros();
		}
		break;
		case 2:
			echantillonnagecourant();
			if (tension > SavedDatas.courant_max || ((duree_etat > SavedDatas.MS_BROSSAGE) && angle_mesure < SavedDatas.angle_declenchement))
			{
				MoteursOff();
				etat =Arret;
				t_debut_etat = micros();
				ajout_temps_brossage();
			}
			else if ((duree_etat > SavedDatas.MS_BROSSAGE) && angle_mesure >= SavedDatas.angle_declenchement) {
				ajout_temps_brossage();
				t_debut_etat = micros();
			}
			break;
		case 3:
			if (duree_etat > MS_SURCOURANT) { 
				etat = Attentedemarrage;
				vue_affichage = 5;
			}
			break;
	default:
		break;
	}

	buttonstate = digitalRead(buttonpin);
	digitalWrite(13, buttonstate);

	if (prevbuttonstate != buttonstate){
		if (buttonstate == false){
			vue_affichage += 1;
			issetting = 1;
		}
	}
	prevbuttonstate = buttonstate;
	gestion_affichage(map(analogRead(pinaAdjustValueY), 0, 1023, -1000, 1000));
	while (abs(micros() - loop_timer) < 4000);                                //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
	if (abs(duree_etat / 1000000) > 600) t_debut_etat = micros();
	loop_timer = micros();//Reset the loop timer
	//HandleSerial();
	wdt_reset();
}
void HandleSerial()
{
	if (Serial.available() > sizeof(ExchangedDatas))
	{

		Serial.readBytes((byte*)&ExchangedDatas, sizeof(ExchangedDatas));
		if (ExchangedDatas.type == 'G')
		{
			ExchangedDatas.angle_declenchement = SavedDatas.angle_declenchement;
			ExchangedDatas.courant_max = SavedDatas.courant_max;
			ExchangedDatas.MS_BROSSAGE = SavedDatas.MS_BROSSAGE;
			ExchangedDatas.temps_total_brossage = SavedDatas.temps_total_brossage;
			ExchangedDatas.angle_mesure = angle_mesure;
			ExchangedDatas.courant = tension;
			Serial.write((byte*)&ExchangedDatas, sizeof(ExchangedDatas));
		}
		else if (ExchangedDatas.type == 'P')
		{
			SavedDatas.angle_declenchement = ExchangedDatas.angle_declenchement;
			SavedDatas.courant_max = ExchangedDatas.courant_max;
			SavedDatas.MS_BROSSAGE = ExchangedDatas.MS_BROSSAGE;
			sauvegardeEEPROM();
		}
	}
}

void MoteursOff()
{
	t_debut_etat = micros();
	digitalWrite(pin_moteur_relais1, HIGH);
	delayMicroseconds(40000);
	digitalWrite(pin_moteur_relais2, HIGH);

	etat_moteur = 0;
	//echantillon_angle[TAILLE_TABLEAU_ECHANTILLONS] = { 0 };
	tension = 0;
}
void MoteursOn()
{
	InversionSensRotation();
	t_debut_etat = micros();
	vue_affichage = 6;
	etat_moteur = 1;
	digitalWrite(pin_moteur_relais1, LOW);
	delayMicroseconds(40000);
	digitalWrite(pin_moteur_relais2, LOW);
}
void InversionSensRotation()
{
	SensRotation = !SensRotation;
	digitalWrite(pin_moteur_inversion1, SensRotation);
	delay(2);
	digitalWrite(pin_moteur_inversion2, SensRotation);
}
void echantillonnagecourant()
{
	int mesure = map(analogRead(pinCourant), 0, 1023, 0, 5000);
	if (mesure > I_MAX||temps_I_MAX+micros()>20000) {
			I_MAX = mesure;
		temps_I_MAX = micros();
	}
	else if (mesure < I_MIN || temps_I_MIN + micros()>20000) {
		I_MIN = mesure;
		temps_I_MIN = micros();
	}
	tension = (I_MAX - I_MIN) / 2;
	//echantillon_angle[indexEchantillonsCourant] = map(analogRead(pinCourant), 0, 1023, 0, 5000);
	//for (i = 0; i < TAILLE_TABLEAU_ECHANTILLONS; i++)
	//{
	//	tension += echantillon_angle[i] * echantillon_angle[i];
	//}
	//tension = sqrt(tension / TAILLE_TABLEAU_ECHANTILLONS);
	////Serial.println(tension);
	//indexEchantillonsCourant++;
	//if (indexEchantillonsCourant == TAILLE_TABLEAU_ECHANTILLONS)
	//{
	//	indexEchantillonsCourant = 0;
	//}
}
void Echantillonnageangle()
{
	echantillon_angle[indexechantilon] =angle_brut;
	for (i = 0; i < TAILLE_TABLEAU_ECHANTILLONS; i++)
	{
		angle_mesure += echantillon_angle[i];
	}
	angle_mesure = angle_mesure / TAILLE_TABLEAU_ECHANTILLONS;
	indexechantilon++;
	if (indexechantilon == TAILLE_TABLEAU_ECHANTILLONS)
	{
		indexechantilon = 0;
	}
}
void ajout_temps_brossage()
{
	SavedDatas.temps_total_brossage += (duree_etat) / (1000000 * 60);
	sauvegardeEEPROM();
}
void setup_mpu_6050_registers() {
	//Activate the MPU-6050
	Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
	Wire.write(0x6B);                                                    //Send the requested starting register
	Wire.write(0x00);                                                    //Set the requested starting register
	Wire.endTransmission();
	//Configure the accelerometer (+/-8g)
	Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
	Wire.write(0x1C);                                                    //Send the requested starting register
	Wire.write(0x10);                                                    //Set the requested starting register
	Wire.endTransmission();
	//Configure the gyro (500dps full scale)
	Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
	Wire.write(0x1B);                                                    //Send the requested starting register
	Wire.write(0x08);                                                    //Set the requested starting register
	Wire.endTransmission();
}

void read_mpu_6050_data() {                                             //Subroutine for reading the raw gyro and accelerometer data
	Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
	Wire.write(0x3B);                                                    //Send the requested starting register
	Wire.endTransmission();                                              //End the transmission
	Wire.requestFrom(0x68, 14);                                           //Request 14 bytes from the MPU-6050
	while (Wire.available() < 14);                                        //Wait until all the bytes are received
	acc_x = Wire.read() << 8 | Wire.read();
	acc_y = Wire.read() << 8 | Wire.read();
	acc_z = Wire.read() << 8 | Wire.read();
	temp = Wire.read() << 8 | Wire.read();
	gyro_x = Wire.read() << 8 | Wire.read();
	gyro_y = Wire.read() << 8 | Wire.read();
	gyro_z = Wire.read() << 8 | Wire.read();
}
void gestion_affichage(long analogvalue)
{
	prev_vue = vue_affichage;
	if (abs(analogvalue) < 300)
	{
		analogvalue = 0;
	}
	switch (vue_affichage)
	{
	case 0:
		SavedDatas.angle_declenchement = SavedDatas.angle_declenchement + analogvalue / 500;
		SavedDatas.angle_declenchement = constrain(SavedDatas.angle_declenchement, 0, 90);
		Affichage("Angle declenchement", String(SavedDatas.angle_declenchement), 248);
		break;
	case 1:
		SavedDatas.courant_max = SavedDatas.courant_max + analogvalue / 200;
		SavedDatas.courant_max = constrain(SavedDatas.courant_max, 0, 5000);
		Affichage("Courant de coupure ", String(SavedDatas.courant_max), 0);
		break;
	case 2:
		SavedDatas.MS_BROSSAGE = SavedDatas.MS_BROSSAGE + analogvalue * 1000;
		SavedDatas.MS_BROSSAGE = constrain(SavedDatas.MS_BROSSAGE, 30000000, 400000000);		
		Affichage("Temps de brossage :", String(SavedDatas.MS_BROSSAGE/1000000) + " sec", 0);
		break;
	case 3:
		Affichage("Tps total brossage :", (String)SavedDatas.temps_total_brossage + " min", 0);
		break;
	case 4:
		Affichage("Tps total brossage :", String((float)(SavedDatas.temps_total_brossage / 60)) + " h", 0);
		break;
	case 5:
		Affichage("Angle mesure:" + String(angle_mesure), "Angle declenchement : " + String(SavedDatas.angle_declenchement), 248);
		break;
	case 6:
		Affichage("Courant mesure : " + String(tension),"Restant brossage:"+String((SavedDatas.MS_BROSSAGE-duree_etat)/1000000) + "s", 0);
		break;
	case 7:
		Affichage("Fin reglage, donneees sauvegardees", "Re-clique pour regler", 0);
		vue_affichage = -1;
		issetting = 0;
		sauvegardeEEPROM();
		break;
	default:
		vue_affichage = -1;
		issetting = 0;
		break;
	}
	display.display();
	
}
void Affichage(String ligne1, String ligne2, int caracspec)
{
	display.clearDisplay();
	display.setCursor(0, 0);
	display.print(ligne1);
	display.setCursor(0, 16);
	display.print(ligne2);
	if (caracspec != 0)
	{
		display.write(caracspec);
	}
}
void sauvegardeEEPROM() {
	SavedDatas.magic = STRUCT_MAGIC;
	SavedDatas.struct_version = STRUCT_VERSION;
	EEPROM.put(0, SavedDatas);
}

void chargeEEPROM() {

	EEPROM.get(0, SavedDatas);

	// Détection d'une mémoire non initialisée
	byte erreur = SavedDatas.magic != STRUCT_MAGIC;

	// Valeurs par défaut struct_version == 0
	if (erreur) {

		// Valeurs par défaut pour les variables de la version 0
		SavedDatas.MS_BROSSAGE = (unsigned long)120000000;
		SavedDatas.courant_max = 1300;
		SavedDatas.angle_declenchement = 20;
		SavedDatas.temps_total_brossage = 0;
	}

	// Valeurs par défaut struct_version == 1
	//if (SavedDatas.struct_version < 1 || erreur) {

	//	// Valeurs par défaut pour les variables de la version 1
	//	SavedDatas.valeur_2 = 13.37;
	//}

	//// Valeurs par défaut pour struct_version == 2
	//if (SavedDatas.struct_version < 2 || erreur) {

	//	// Valeurs par défaut pour les variables de la version 2
	//	strcpy(SavedDatas.valeur_3, "Hello World!");
	//}

	// Sauvegarde les nouvelles données
	sauvegardeEEPROM();
}