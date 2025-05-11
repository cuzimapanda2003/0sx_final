
#include <HCSR04.h>
#include <LCD_I2C.h>
#include <U8g2lib.h>
#include <AccelStepper.h>
#include <PubSubClient.h>
#include <WiFiEspAT.h>
#include <DHT.h>
#include "Alarm.h"
#include "ViseurAutomatique.h"

#define MOTOR_INTERFACE_TYPE 4

#define IN_1 31
#define IN_2 33
#define IN_3 35
#define IN_4 37


#define CLK_PIN 30
#define DIN_PIN 34
#define CS_PIN 32  // Chip Select

U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(
  U8G2_R0,             // rotation
  /* clock=*/CLK_PIN,  // pin Arduino reliée à CLK (horloge)
  /* data=*/DIN_PIN,   // pin Arduino reliée à DIN (données)
  /* cs=*/CS_PIN,      // pin Arduino reliée à CS (chip select)
  /* dc=*/U8X8_PIN_NONE,
  /* reset=*/U8X8_PIN_NONE);

#define TRIGGER_PIN 3
#define ECHO_PIN 2

#define lum "A0"
int lumiere;

int buzzerPin = 4;
int frequence = 1;

int redPin = 6;
int bluePin = 7;
int greenPin = 8;

LCD_I2C lcd(0x27, 16, 2);

HCSR04 hc(TRIGGER_PIN, ECHO_PIN);

float distance;

Alarm alarm(redPin, greenPin, bluePin, buzzerPin, distance);
ViseurAutomatique viseur(IN_1, IN_2, IN_3, IN_4, distance);

int minStepper;
int maxStepper;

unsigned long currentTime = 0;

unsigned long tempsAffichageSymbole = 0;
bool symboleActif = false;


#define HAS_SECRETS 1

#if HAS_SECRETS
#include "arduino_secrets.h"
/////// SVP par soucis de sécurité, mettez vos informations dans le fichier arduino_secrets.h

// Nom et mot de passe du réseau wifi
const char ssid[] = SECRET_SSID;
const char pass[] = SECRET_PASS;

#else
const char ssid[] = "TechniquesInformatique-Etudiant";  // your network SSID (name)
const char pass[] = "shawi123";                         // your network password (use for WPA, or use as key for WEP)

#endif

#if HAS_SECRETS
#define DEVICE_NAME "2168637"
#else
#define DEVICE_NAME "2168637"
#endif

#define MQTT_PORT 1883
#define MQTT_USER "etdshawi"
#define MQTT_PASS "shawi123"

#define AT_BAUD_RATE 115200

// Serveur MQTT du prof
const char* mqttServer = "216.128.180.194";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

unsigned long lastWifiAttempt = 0;
const unsigned long wifiRetryInterval = 10000;

void wifiInit() {
  while (!Serial)
    ;

  Serial1.begin(AT_BAUD_RATE);
  WiFi.init(Serial1);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println();
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }

  WiFi.disconnect();  // to clear the way. not persistent

  WiFi.setPersistent();  // set the following WiFi connection as persistent

  WiFi.endAP();  // to disable default automatic start of persistent AP at startup

  //  uncomment this lines for persistent static IP. set addresses valid for your network
  //  IPAddress ip(192, 168, 1, 9);
  //  IPAddress gw(192, 168, 1, 1);
  //  IPAddress nm(255, 255, 255, 0);
  //  WiFi.config(ip, gw, gw, nm);

  Serial.println();
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);

  //  use following lines if you want to connect with bssid
  //  const byte bssid[] = {0x8A, 0x2F, 0xC3, 0xE9, 0x25, 0xC0};
  //  int status = WiFi.begin(ssid, pass, bssid);

  int status = WiFi.begin(ssid, pass);

  if (status == WL_CONNECTED) {
    Serial.println();
    Serial.println("Connected to WiFi network.");
    printWifiStatus();
  } else {
    WiFi.disconnect();  // remove the WiFi connection
    Serial.println();
    Serial.println("Connection to WiFi network failed.");
  }
}
void printWifiStatus() {

  // print the SSID of the network you're attached to:
  char ssid[33];
  WiFi.SSID(ssid);
  Serial.print("SSID: ");
  Serial.println(ssid);

  // print the BSSID of the network you're attached to:
  uint8_t bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  printMacAddress(mac);

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}

void toggleMoteurOff() {
  viseur.desactiver();
}
void toggleMoteurOn() {
  viseur.activer();
}

// Gestion des messages reçues de la part du serveur MQTT
void mqttEvent(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message recu [");
  Serial.print(topic);
  Serial.print("] ");

  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.println(message);

  if (strcmp(topic, "etd/20/motor") == 0) {
    if (message.indexOf("\"motor\":1") != -1) {
      toggleMoteurOn();
      Serial.println("Moteur ON");
    } else if (message.indexOf("\"motor\":0") != -1) {
      toggleMoteurOff();
      Serial.println("Moteur OFF");
    } else {
      Serial.println("Message inconnu pour le moteur");
    }
  }
}



void periodicTask() {
  static unsigned long lastTime = 0;
  const unsigned int rate = 2500;  // toutes les 2.5 secondes

  if (currentTime - lastTime < rate) return;
  lastTime = currentTime;

  unsigned long uptime = millis() / 1000;
  int dist = static_cast<int>(distance);
  int angle = viseur.getAngle();
  lumiere = map(analogRead(lum), 0, 1023, 0, 100);

  const int MAX_CHAR = 300;
  char message[MAX_CHAR];

  // Ajout des clés "name" et "number"
  sprintf(message,
          "{\"name\":\"marc\", \"number\":7, \"uptime\":%lu, \"dist\":%d, \"angle\":%d, \"lum\":%d}",
          uptime, dist, angle, lumiere);

  Serial.print("Envoi MQTT : ");
  Serial.println(message);

  if (!client.publish("etd/20/data", message)) {
    reconnect();
    Serial.println("Échec d'envoi !");
  } else {
    Serial.println("Message envoyé");
  }
}
void periodicTaskLCD() {
  static unsigned long lastTime = 0;
  const unsigned int rate = 1100;
  if (currentTime - lastTime < rate) return;
  lastTime = currentTime;

  char distStr[10];
  dtostrf(distance, 1, 0, distStr);


  char line1[20];
  char line2[20];
  snprintf(line1, sizeof(line1), "Dist : %s cm", distStr);
  snprintf(line2, sizeof(line2), "Obj  : %s", viseur.getEtatTexte());

  char message[200];
  snprintf(message, sizeof(message),
           "{\"line1\": \"%s\", \"line2\": \"%s\"}", line1, line2);

  Serial.print("Envoi LCD MQTT : ");
  Serial.println(message);

  if (!client.publish("etd/20/data", message)) {
    reconnect();
    Serial.println("Échec d'envoi LCD !");
  } else {
    Serial.println("Message LCD envoyé.");
  }
}
bool reconnect() {
  bool result = client.connect(DEVICE_NAME, MQTT_USER, MQTT_PASS);
  if (!result) {
    Serial.println("Incapable de se connecter sur le serveur MQTT");
  }
  return result;
}
void ecranSetup() {
  u8g2.begin();
  u8g2.setContrast(5);
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}
void lcdstart() {
  lcd.print("2168637");
  lcd.setCursor(0, 1);
  lcd.print("laboFinal");
  delay(2000);
}
void setup() {
  Serial.begin(115200);
  lcd.begin();
  lcd.backlight();

  minStepper = viseur.getMinStep();
  maxStepper = viseur.getMaxStep();
  alarm.setDistance(15);
  alarm.setColourA(1, 0, 0);
  alarm.setColourB(0, 0, 1);
  viseur.setAngleMin(10);
  viseur.setAngleMax(170);
  alarm.turnOn();
  viseur.activer();

  wifiInit();

  client.setServer(mqttServer, MQTT_PORT);
  client.setCallback(mqttEvent);

  if (!client.connect(DEVICE_NAME, MQTT_USER, MQTT_PASS)) {
    Serial.println("Incapable de se connecter sur le serveur MQTT");
    Serial.print("client.state : ");
    Serial.println(client.state());
  } else {
    Serial.println("Connecté sur le serveur MQTT");
  }
  client.subscribe("etd/20/motor", 0);


  ecranSetup();
  lcdstart();
}
void loop() {
  currentTime = millis();
  chercherDistance();
  ecranLCD(currentTime);
  alarm.update();
  viseur.update();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (WiFi.status() != WL_CONNECTED) {
    if (currentTime - lastWifiAttempt >= wifiRetryInterval) {
      Serial.println("WiFi perdu. Tentative de reconnexion...");
      WiFi.begin(ssid, pass);
      lastWifiAttempt = currentTime;
    }
  }
  periodicTask();
  periodicTaskLCD();


  commande();
}
void ecranLCD(unsigned long time) {
  static unsigned long lastTime = 0;
  int rate = 100;

  if (time - lastTime >= rate) {
    lcd.setCursor(0, 0);
    lcd.print("Dist : ");
    lcd.print(distance);
    lcd.print(" cm  ");
    lcd.setCursor(0, 1);
    lcd.print("Obj  : ");
    lcd.print(viseur.getEtatTexte());

    lastTime = time;
  }
}
void chercherDistance() {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  int interval = 100;

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    distance = hc.dist();
  }
}
void verifierSymbole() {
  if (symboleActif && millis() - tempsAffichageSymbole >= 3000) {
    u8g2.clearBuffer();
    u8g2.sendBuffer();
    symboleActif = false;
  }
}
void dessinInterdit() {

  u8g2.clearBuffer();
  u8g2.drawCircle(3, 4, 3);
  u8g2.drawLine(0, 0, 8, 8);
  u8g2.sendBuffer();
  symboleActif = true;
  tempsAffichageSymbole = millis();
}
void dessinX() {
  u8g2.clearBuffer();
  u8g2.drawLine(7, 0, 0, 7);
  u8g2.drawLine(0, 0, 8, 8);
  u8g2.sendBuffer();
  symboleActif = true;
  tempsAffichageSymbole = millis();
}
void dessinWeGood() {
  u8g2.clearBuffer();
  u8g2.drawLine(4, 4, 2, 6);
  u8g2.drawLine(1, 1, 4, 4);
  u8g2.sendBuffer();
  symboleActif = true;
  tempsAffichageSymbole = millis();
}
void analyserCommande(const String& tampon, String& commande, String& arg1, String& arg2) {
  commande = "";
  arg1 = "";
  arg2 = "";

  int firstSep = tampon.indexOf(';');
  int secondSep = tampon.indexOf(';', firstSep + 1);

  if (firstSep == -1) {
    // Pas de point-virgule, c'est peut-être "stop" ou autre commande sans paramètre
    commande = tampon;
    return;
  }

  // Extraire la commande
  commande = tampon.substring(0, firstSep);

  // Extraire arg1
  if (secondSep != -1) {
    arg1 = tampon.substring(firstSep + 1, secondSep);

    arg2 = tampon.substring(secondSep + 1);
  } else {
    // Il y a une seule valeur après la commande
    arg1 = tampon.substring(firstSep + 1);
  }
}
void commande() {
  if (!Serial.available()) {
    return;
  }
  String tampon = Serial.readStringUntil('\n');

  String commande;
  String arg1, arg2;
  analyserCommande(tampon, commande, arg1, arg2);

  bool commandeValide = false;

  if (commande == "g_dist") {
    Serial.println(distance);
    dessinWeGood();
    commandeValide = true;
  } else if (commande == "cfg" && arg1 == "alm") {
    alarm.setDistance(arg2.toInt());
    dessinWeGood();
    commandeValide = true;
  } else if (commande == "cfg" && arg1 == "lim_inf") {
    if (arg2.toInt() > viseur.getDistanceMaxSuivi()) {
      Serial.println("erreur 🚫");
      dessinInterdit();
    } else {
      viseur.setDistanceMinSuivi(arg2.toInt());
      dessinWeGood();
    }
    commandeValide = true;
  } else if (commande == "cfg" && arg1 == "lim_sup") {
    if (arg2.toInt() < viseur.getDistanceMinSuivi()) {
      Serial.println("erreur 🚫");
      dessinInterdit();
    } else {
      viseur.setDistanceMaxSuivi(arg2.toInt());
      dessinWeGood();
    }
    commandeValide = true;
  }

  if (!commandeValide && tampon != "") {
    dessinX();
  }
}
