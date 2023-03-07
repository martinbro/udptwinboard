/****************************************************************************************************************************
	ESP8266-Minimal-Client: Minimal ESP8266 Websockets Client
	This sketch:
				0. Connects to a twin board.
				1. Connects to a WiFi network
				2. Connects to a Websockets server
					 a. Send Calculated val to a Webserver using Websockets
					 b. Get Waypoints info& PID values using Websockets
				3. Connects to BNO055
				4. Connects to GPS
				5. Connects to Servo
				6. Connects to ESP8866
				7. Calculate course using datafusion
				8. Calculate position and speed using dead reckoning (not implemented)
				9. Calculate rudder position using PID  only in AUTO MODE
				10. Stores Data in External storige (not implementet)
		NOTE:
		The sketch dosen't check or indicate about errors while connecting to
		WiFi or to the websockets server. For full example you might want
		to try the example named "ESP8266-Client".
	Hardware:
				ESP8266 board.
				GPS
				BNO055
				Servo (rudder)
	Created  : 2021
	Author   : Martin Bro Mikkelsen
*****************************************************************************************************************************/

#include <ESP8266WiFi.h>
#include <TinyGPS++.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Soft-AP
const char *ssid_softAP = "ESPap";
const char *password_softAP = "thereisnospoon";
IPAddress local_IP(192, 168, 4, 122); // 192.168.137.188:4210
IPAddress gateway(192, 168, 4, 121);
IPAddress subnet(255, 255, 255, 0);

// uint16_t PORT = 4110 ;//SKIB1
uint16_t PORT = 4210 ;//SKIB2
//uint16_t PORT = 4310 ;//SKIB3
// uint16_t PORT = 4410 ;//SKIB4

// Station
const char *ssid_sta = "SKIB2";
const char *password_sta = "marnavfablab";
unsigned int localUdpPort = 8081;					  // local port
char incomingPacket[255];							  // buffer for incoming packets

// Namespaces
WiFiUDP Udp;
// static const int RORpwm = 14; //D5 som udlæg output
// char PORT_NR[5] = "/ws1";
static const uint8_t SCL_gyro = 5, SDA_gyro = 4; // Gyro: D2 (SCL), D1(SDA)
static const uint8_t RXPin = 15, TXPin = 13;	 // GPS: D8 (RXin=TDXgps), D7(TDXin=RDX(gps))
static const uint16_t GPSBaud = 9600;

/* Globale variable ************************************************ */

uint8_t BNO055_SAMPLERATE_DELAY_MS = 11;
uint8_t ROR_UPDATE_DELAY_MS = 100; // uint8_t: [0 .. 255]
uint16_t WiFi_DELAY_MS = 100;	   // uint16_t: [0 .. 65,535]
static uint32_t t0 = 0;			   //, t0_main_loop = 0 ;
static uint32_t t_ror = 0;		   //, t0_main_loop = 0 ;
static uint32_t t1 = 0;			   //, t1_main_loop = 0; //, tWiFi = 0;
boolean updated = false;
float floatVal;
char charVal;

// experimentel
static uint32_t timer1 = 0;

// Performanceforbedring ved at anvende simple variable frem for structs

float gx, gy, gz = 0.0;
float dt = 0.0;
float kurs, kursRaw, roll, rollRaw, pitch, pitchRaw = 0.0;

float K = 0.99;
float MISVISNING = 3.75; // NB https://www.magnetic-declination.com/
float kursGyroStabiliseret;

uint16_t kalibVal = 0; // status for kalibreret værdi (gyro)

float lg[] = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0}; // måske flytte over på dea anden chip
float br[] = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0}; // måske flytte over på dea anden chip
float r = 5.0;															   // Succefuld afstand til way point - måske flytte over på dea anden chip
uint8_t antalWP = 0;													   // måske flytte over på dea anden chip
uint8_t activeWP = 0;													   // måske flytte over på dea anden chip

double brGps = -1.0;
double lgGps = -1.0;
double tiGps = -1.0;

float e = 0.0;
float sum_e = 0.0;
float P = 1;
float I = 0;
float D = 0;
int ROR_KALIB = 90; // neutral rorudlæg

/* Initialiserer moduler ******************************************* */

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
Adafruit_BNO055 bno = Adafruit_BNO055(); //(id,address)

void setup()
{
	Wire.begin(5, 4);
	Serial.begin(115200);
	while (!Serial)
	{
		;
	}
	//Forbinder til GO-server i land
	WiFi.mode(WIFI_STA);//WiFi.mode(WIFI_AP_STA) //burde udskiftes?
	Serial.printf("Connecting to %s ", ssid_sta);
	// WiFi.config();
	WiFi.begin(ssid_sta, password_sta);

	if (WiFi.waitForConnectResult(1000) != WL_CONNECTED)
	{
		Serial.println("WiFi Failed");
		// delay(500);
	}
	//Kontakt til andre ESP'er
	Serial.print("Setting soft-AP configuration ... ");
	Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");

	Serial.print("Setting soft-AP ... ");
	Serial.println(WiFi.softAP(ssid_softAP, password_softAP) ? "Ready" : "Failed!");

	Serial.print("Soft-AP IP address = ");
	Serial.println(WiFi.softAPIP());

	delay(500);
	IPAddress myIP = WiFi.softAPIP();
	Serial.print("AP IP address: ");
	Serial.println(myIP);
	Serial.println("HTTP server started");

	Udp.begin(localUdpPort); // lytter på port 8081
	Udp.begin(PORT);		 // lytter på port 4210    *********************TROR DEN SKAL SLETTES!!!!!!!!

	ss.begin(GPSBaud);
	initBNO055(bno);
}

int i = 0;
unsigned long tWiFi = millis();
// unsigned long tPing = millis();

void loop()
{
	if (updated)
	{ // begrænser netværkstrafik
		sendEkko();
		updated = false;
	}

	if (getBNO055val())
	{
		if ((millis() - tWiFi) > 17)
		{ // begrænser netværkstrafik
			sendBNOdata();

			tWiFi = millis();
		}
		if (sendRorUdlaeg())
			Serial.println("Rorudlæg send");
	}

	while (ss.available() > 0)//ss: software serial - dvs læser fra Serial Port
	{
		if (gps.encode(ss.read()))
		{
			i++;
			if (gps.location.isValid())
			{
				if (i % 9 == 0)
				{ // Primitiv netværks begrænsning

					if (ss.overflow())
					{
						Serial.println("SoftwareSerial overflow!");
					}
					i = 0;
					// brGps = gps.location.lat();
					// lgGps = gps.location.lng();
					// tiGps = gps.time.hour() * 3600 + gps.time.minute() * 60 + gps.time.second() + gps.time.centisecond() / 100;
					sendGPSdata(gps);
				}
			}
		}
	}
	// receive incoming UDP packets
	int packetSize = Udp.parsePacket();
	if (packetSize > 0)
	{
		Serial.printf("Received %d bytes from %s, port %d ", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
		int len = Udp.read(incomingPacket, packetSize + 1);

		if (len > 0) // Dvs. Udp.read() har ikke virket- eller der er lutter blanktegn?
		{
			incomingPacket[len] = '\0'; // C-style endelse
			// Serial.printf("UDP packet contents: %s, %d\n", incomingPacket, len);
			if (len > 4)
			{
				process(incomingPacket, len);
			}
		}
	}
}
void process(char *dat, int len)
{
	char nr = dat[4];
	charVal = nr;
	// Serial.printf("process linje 228: %s %i \n", dat, len);

	int startBit = 5;

	floatVal = atof(&dat[startBit]);

	{
		/**
		 *  besked er af formen:
		 * 1 "a54.8743326:10.469032,54.8743326:10.469032," NB slutter med komma
		 * (1.a "a-1:-1," fjerner alle wp)
		 * 2. "b123.456"
		 **/
		int startnr = 1; // bruges i case a
		int midt = 0;	 // bruges i case a
		int pos_nr = 0;	 // bruges i case a

		int pos = 0;
		boolean skift = false;
		switch (nr)
		{
		case 'a': // waypoint
			// antalWP=0;

			for (int i = 5; i < len; i++)
			{
				if (dat[i] == ':')
				{
					skift = !skift;
					dat[i] = '\0';

					lg[pos] = atof(&dat[startBit]);
					startBit = i + 1;
				}
				else if (dat[i] == ',')
				{
					skift = !skift;
					dat[i] = '\0';

					br[pos] = atof(&dat[startBit]);
					pos++;
					antalWP = pos;
					startBit = i + 1;
				}
			}
			for (int i = antalWP; i < 10; i++)
			{
				lg[i] = -1;
				br[i] = -1;
			}
			updated = true;
			break;

		case 'c': // K-val
			if (floatVal < 0.0)
				K = 0.0;
			if (floatVal > 1.0)
				K = 1.0;
			K = floatVal;
			updated = true;		
			break;

		case 'd': // Misvisning
			MISVISNING = floatVal;
			updated = true;
			break;

		case 'e': // Radius til waypoint
			r = floatVal;
			updated = true;
			break;

		case 'f': // P værdi til styring
			P = floatVal;
			updated = true;
			break;

		case 'g': // I værdi til styring
			I = floatVal;
			updated = true;
			break;

		case 'h': // D værdi til styring
			D = floatVal;
			updated = true;
			break;

		case 'i': // Ror kalibrering
			ROR_KALIB = floatVal;
			updated = true;
			break;
		case 'j'://Speed auto
			char fstr[16];
			sprintf(fstr, "b%d", (int)floatVal); // gps.course.isValid() ? gps.course.deg() : NAN,gps.speed.isValid() ? gps.speed.kmph() : NAN);
			// sender til ESP-lokal (static IP)
				Serial.print("FSTR: ");
				Serial.println(fstr);
				Serial.print("floatVal: ");
				Serial.println(floatVal);
			Udp.beginPacket("192.168.4.123", PORT);
				Udp.write(fstr);
				delay(5);
				Udp.write(fstr);
				delay(5);
				Udp.write(fstr);
			Udp.endPacket();
			updated = true;
		break;

		default:
			Serial.println("FEJL i modtagelse");
			updated = false;
			break;
		}
	}
}

/* ** FUNKTIONER ** FUNKTIONER ** FUNKTIONER ** FUNKTIONER ** */

void sendEkko()//til bekræftigelse af modtaget v
{
	if (charVal == 'a')
	{
		char buffer[40];
		for (int i = 0; i < 10; i++)
		{
			// Serial.printf("lg[i]%10f br[i]%10f", i, lg[i], i, br[i]);
			sprintf(buffer, "pin,%c(%i)%11f:%11f", charVal, i, lg[i], br[i]);
			Udp.beginPacket("192.168.137.1", 8083); // 192.168.137.1:8083 - mobilt hotspot
			Udp.write(buffer);
			Udp.endPacket();
			delay(20);
		}
	}
	else
	{
		char buffer[20];
		sprintf(buffer, "pin,%c%8f", charVal, floatVal);
		Udp.beginPacket("192.168.137.1", 8083); // 192.168.137.1:8083 - mobilt hotspot
		Udp.write(buffer);
		Udp.endPacket();
	}
}
//////////////////////Kompas///////////////////////////////
void initBNO055(Adafruit_BNO055 bno)
{
	delay(1000);
	if (!bno.begin())
	{ /* Initialise the sensor */
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		// client.send("fejl, Ingen forbindelse til gyrokompas");
		delay(1000);
		ESP.restart();
	}
	Serial.print("Succesfuld forbindelse til BNO055");

	delay(1000);
	bno.setExtCrystalUse(true); /* Bruger microprocessorens  clock */
	bno.printSensorDetails();
}
bool getBNO055val()
{
	/* Get a new sensor vector*/
	if ((millis() - t0) < BNO055_SAMPLERATE_DELAY_MS)
	{
		delay(1);
		return false;
	}
	/* Beregner tidsdifferencen dt mellem læsninger */
	t1 = millis();
	if (t0 == 0)
	{
		t0 = t1;
	} // initialiserer
	dt = (t1 - t0);
	dt = dt / 1000.0; /* i sek. */
	t0 = t1;

	imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	imu::Vector<3> m = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
	imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

	/* Henter data fra BNO055 */
	/* NB! akser fra 'euler'-koordinater til 'fly-koordinater' (dvs. x-akse frem & z-akse NEDAD!) dvs y- og z- akser skifter fortegn)
	 * Årsag: for at få pos ROT om z-akse i samme retn. som kompassets pos. retn)
	 * NB! acc er neg da BNO055 måler reaktionskraft.
	 */
	float rotx = g[0];
	float roty = -g[1];
	float rotz = -g[2];

	float ax = -a[0];
	float ay = -(-a[1]);
	float az = -(-a[2]);

	float mx = m[0];
	float my = -m[1];
	float mz = -m[2];

	// Beregner gyroens kurs fra Rate Of Turn
	//  float gx, gy, gx = 0.0;
	gx = gx + rotx * dt;
	gy = gy + roty * dt;
	gz = gz + rotz * dt;

	// Roll, Pitch og Yaw (kurs) beregnes - bare trigonometri
	float RollRaw = atan2(ay, az); // rollRaw i radianer
	rollRaw = RollRaw * 180 / PI;
	pitchRaw = atan2(-ax, (ay * sin(RollRaw) + az * cos(RollRaw))) * 180 / PI;
	kursRaw = atan2(-my, mx) * 180 / PI;

	// Comperatorfilter på roll, og pitch, 99% gyro, 1% acc
	float k = 0.99; // procent gyro
	roll = (roll + rotx * dt) * k + rollRaw * (1 - k);
	pitch = (pitch + roty * dt) * k + pitchRaw * (1 - k);

	//'Gyrostabiliserede' værdier
	// roll & pitch i radianer
	float Roll = roll * PI / 180;
	float Pitch = pitch * PI / 180;

	// tilt kompenseret kurs.(Jeg kan vise dig beregningen hvis du er interesseret Jørgen - fås direkte ud fra rotationsmatriserne for roll og pitch)
	//(Findes også mange steder på nettet! Pas dog på wikipiedia - der har de byttet om på roll, pitch og yaw... Hmmm det ligner dem ellers ikke...)
	//  NB! her anvendes de gode! værdier for roll og pitch i radianer!
	float X = mx * cos(Pitch) + mz * sin(Pitch);
	float Y = mx * sin(Roll) * sin(Pitch) + my * cos(Roll) - mz * sin(Roll) * cos(Pitch);
	kursGyroStabiliseret = (atan2(-Y, X) * 180 / PI);
	float gyrokurs = kurs + rotz * dt;

	// Beregner hastighedsændringer fra accelerometret
	//  dvx = (ax*cos(Pitch) + az*sin(Pitch))*dt;
	//  dvy = (ax*sin(Roll)*sin(Pitch) + ay*cos(Roll) - az * sin(Roll)*cos(Pitch))*dt;

	// løser et fjollet diskontinuitetsproblem mellem gyro og mag. (Første del)
	// får mag til at være kontinuært over 180
	if (gyrokurs - kursGyroStabiliseret < -180)
	{
		while (gyrokurs - kursGyroStabiliseret < -180)
		{
			kursGyroStabiliseret = kursGyroStabiliseret - 360.0;
		}
	}
	else if (gyrokurs - kursGyroStabiliseret > 180)
	{
		while (gyrokurs - kursGyroStabiliseret > 180)
		{
			kursGyroStabiliseret = kursGyroStabiliseret + 360.0;
		}
	}
	kurs = (K * (gyrokurs) + (1 - K) * kursGyroStabiliseret);
	getKalibrering();

	return true;
}

void getKalibrering()
{
	uint8_t systemC, gyroC, accelC, magC = 0;
	// Auto kalibreringens status: (integer) 0=lavest niveau (forkast data), 3=højste niveau (fuldt kalibreret data)
	bno.getCalibration(&systemC, &gyroC, &accelC, &magC);

	kalibVal = gyroC / 1.0 * 100 + magC / 1.0 * 10 + accelC / 1.0; // systemC
}

bool sendRorUdlaeg()
{
	// KUN HVIS GPS SIGNALER
	if (brGps < 0 || br[0] < 0)
	{
		return false;
	}
	if (t_ror < 1)
	{
		t_ror = millis();
	} // initialiserer
	if ((millis() - t_ror) < ROR_UPDATE_DELAY_MS)
	{
		// delay(1);
		return false;
	}

	t_ror = millis(); // opdaterer

	// Bestikregning
	//  float dN += cos(-kurs*PI/180)*dt; // pos
	//  float dE += -sin(-kurs*PI/180)*cos(brGps)*dt;
	//  Serial.print("Bestik: "); Serial.print(dN); Serial.print(", "); Serial.println(dE);

	float br_forandring = br[activeWP] - brGps;
	float afvigning = (lg[activeWP] - lgGps) * cos(brGps * PI / 180); // bruger bredde i stedet for middelbredde

	float sp_kurs = atan2(afvigning, br_forandring) * 180 / PI;
	float afstandWP = sqrt(br_forandring * br_forandring + afvigning * afvigning) * 60 * 1852;

	/*Beregner X-Track Error - kun påbegyndt*/
	// float xte = 0;
	// if (activeWP > 0)
	// { // sejler på et ben
	// 	int affWP = activeWP - 1;
	// 	float br_forandring1 = br[activeWP] - br[affWP];
	// }

	/* opdaterer aktivt WayPoint*/
	if (afstandWP < r)
	{
		//TO DO 
		//	tilføj en betingelse: if(afstand til waypoint er blevet mindre){ så vent lidt}
		// 	eller se på COG/kurs og SOG og prædikter TCPA = time stamp før vi skifter WP
		activeWP++;
		if (activeWP > 9)
			activeWP = 0; // starter forfra vers 1
		if (br[activeWP] < 0.0 && activeWP > 0)
			activeWP = 0; // starter forfra vers 2
	}

	float e = sp_kurs - (kurs + MISVISNING);
	while (e > 180)
	{
		e = e - 360;
	};
	while (e < -180)
	{
		e = e + 360;
	};

	if (I > 0)
	{

		if (abs(e) < 10)
		{
			sum_e += e * dt / 100.0; // 10.0*1/100=0,1 grad pr.sek
		}
		else if (e > 9.999)
		{
			sum_e += 10.0 * dt / 100.0;
		}
		else if (e < -9.999)
		{
			sum_e += -10.0 * dt / 100.0;
		}
	}
	else
	{
		sum_e = 0;
	}
	if (sum_e * I > 15)
	{
		sum_e = 15.0 / I;
	}
	if (sum_e * I < -15)
	{
		sum_e = -15.0 / I;
	}
	float ror = P * e + I * sum_e;
	if (ror < -35)
	{
		ror = -35;
	};
	if (ror > 35)
	{
		ror = 35;
	}; // ror begrænsning

	int udlg = int(round(ror));
	int num = int(round(udlg + ROR_KALIB));
	Serial.printf(" udl: %i ", udlg);
	char cstr[16];
	sprintf(cstr, "a%d", num); // gps.course.isValid() ? gps.course.deg() : NAN,gps.speed.isValid() ? gps.speed.kmph() : NAN);
	 // sender til ESP-lokal (static IP)
    Udp.beginPacket("192.168.4.123", PORT);//192.168.4.123 ESP lokalnet
    Udp.write(cstr);
    Udp.endPacket();
	sendRorData(num, afstandWP, sp_kurs, ror);

	return true;
}
void sendRorData(int num, float afstandWP, float sp_kurs, float ror)
{
	char buffer[40];
	sprintf(buffer, "udl,%d,%5.1f,%d,%4.2f,%2.1f,%d",
			num,
			afstandWP,
			activeWP,
			sp_kurs,
			ror,
			ESP.getFreeHeap());
	// client.send(buffer);
	Udp.beginPacket("192.168.137.1", 8084);
	Udp.write(buffer);
	Udp.endPacket();

}
float formatKurs(float tal, int precision)
{
	float f = round((tal)*precision) / precision;
	while (f < 0)
	{
		f += 360.0;
	}
	while (f > 360)
	{
		f -= 360.0;
	}
	return f;
}
void sendBNOdata()
{
	// værdier sendes som semikolonsepereret streng - csv-stil
	char buffer[60];
	sprintf(buffer, "bno,%0.1f,%0.2f,%0.2f,%0.1f,%d,%0.1f,%0.1f,%d",
			//		buffer,"bno,360.0,-180.12,-180.12"
			formatKurs(kurs + MISVISNING, 10),
			roll,
			pitch,
			(dt * 1000),
			kalibVal,
			formatKurs(kursRaw + MISVISNING, 10),
			formatKurs(kursGyroStabiliseret + MISVISNING, 10), ESP.getFreeHeap());
	// client.send(buffer);
	Udp.beginPacket("192.168.137.1", 8081);
	Udp.write(buffer);
	Udp.endPacket();
}
////////////////////// GPS ////////////////////////////////
void sendGPSdata(TinyGPSPlus gps)
{
	if (gps.location.isValid() && gps.location.isUpdated())
	{
		brGps = gps.location.lat();
		lgGps = gps.location.lng();
		tiGps = gps.time.hour() * 3600 + gps.time.minute() * 60 + gps.time.second() ;//+ gps.time.centisecond() / 100;
		char buffer[60];
		sprintf(buffer, "gps,%.8f,%.8f,%.1f,%d,%.1f,%.1f,%i",
			brGps,
			lgGps,
			gps.hdop.hdop(),
			gps.satellites.value(),
			gps.course.deg(),
			gps.speed.mps(),
			int(tiGps));
		Serial.println(buffer);
		Udp.beginPacket("192.168.137.1", 8082);
		Udp.write(buffer);
		Udp.endPacket();
		// Udp.beginPacket("192.168.4.123", 8081);//192.168.4.123 ESP lokalnet , port til skib 1
		// Udp.write(buffer);
		// Udp.endPacket();
		// Udp.beginPacket("192.168.4.123", 8082);//192.168.4.123 ESP lokalnet , port til skib 2
		// Udp.write(buffer);
		// Udp.endPacket();
		// Udp.beginPacket("192.168.4.123", 8083);//192.168.4.123 ESP lokalnet , port til skib 3
		// Udp.write(buffer);
		// Udp.endPacket();
		// Udp.beginPacket("192.168.4.123", 8084);//192.168.4.123 ESP lokalnet , port til skib 4
		// Udp.write(buffer);
		// Udp.endPacket();
	}
}
////////////////////// WiFi + websoket2 //////////////////////////
//bool initConnectToWifi()
//{
	// // ESP8266 Connect to wifi

	// delay(5000);
	// for (byte j = 0; j < 3; j++){
	// 	WiFi.begin(sta_ssid, sta_password);
	// 	Serial.println("Prøver at forbinde til LAPTOP-RJIJOOL9 4301");

	// 	for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++){
	// 		Serial.print(".");
	// 		delay(1000);
	// 	}
	// 	if (WiFi.status() == WL_CONNECTED){
	// 		Serial.println("Succesfuld forbindelse til WiFi");
	// 		return true;
	// 	}
	// }
	// // Check if connected to wifi
	// if (WiFi.status() != WL_CONNECTED){
	// 	Serial.println("No Wifi!");
	// 	return false;
	// }
	// WiFi.setAutoReconnect(true);
	// WiFi.persistent(true);
//}