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


#define HAS_SECRETS 0

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
#define DEVICE_NAME "2168637Maison"
#else
#define DEVICE_NAME "2168637"
#endif

#define MQTT_PORT 1883
#define MQTT_USER "etdshawi"
#define MQTT_PASS "shawi123"

// Serveur MQTT du prof
const char* mqttServer = "216.128.180.194";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

unsigned long lastWifiAttempt = 0;
const unsigned long wifiRetryInterval = 10000;

void wifiInit() {
  while (!Serial)
    ;

  Serial1.begin(115200);
  WiFi.init(&Serial1);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println();
    Serial.println("La communication avec le module WiFi a échoué!");
  }

  WiFi.disconnect();  // pour effacer le chemin. non persistant

  WiFi.setPersistent();  // définir la connexion WiFi suivante comme persistante

  WiFi.endAP();  // pour désactiver le démarrage automatique persistant AP par défaut au démarrage

  Serial.println();
  Serial.print("Tentative de connexion à SSID: ");
  Serial.println(ssid);

  int status = WiFi.begin(ssid, pass);

  if (status == WL_CONNECTED) {
    Serial.println();
    Serial.println("Connecté au réseau WiFi.");
  } else {
    WiFi.disconnect();
    Serial.println();
    Serial.println("La connexion au réseau WiFi a échoué.");
  }
}

void toggleMoteur() {
}

// Gestion des messages reçues de la part du serveur MQTT
void mqttEvent(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message recu [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strcmp(topic, "moteur") == 0) {
    toggleMoteur();
  }
}

void periodicTask() {
  static unsigned long lastTime = 0;
  const unsigned int rate = 10000;

  static char message[100] = "";
  // static char szTemp[6];
  // static char szHum[6];
  // static float temp = 0;
  // static float hum = 0;

  if (currentTime - lastTime < rate) return;

  lastTime = currentTime;

  // temp = dht.readTemperature();
  //hum = dht.readHumidity();

  // On convertit les valeurs en chaîne de caractères
  // dtostrf(temp, 4, 1, szTemp);
  //dtostrf(hum, 4, 1, szHum);

#if HOME
  // sprintf(message, "{\"name\":%s, \"temp\" : %s, \"hum\":%s, \"millis\":%lu }", "\"profHome\"", szTemp, szHum, currentTime / 1000);
#else
                                                        // sprintf(message, "{\"name\":%s, \"temp\" : %s, \"hum\":%s, \"millis\":%lu }", "\"Le prof\"", szTemp, szHum, currentTime / 1000);
#endif

  // Serial.print("Envoie : ");
  // Serial.println(message);


  // Changer le topic pour celui qui vous concerne.
  if (!client.publish("etd/20/data", message)) {
    reconnect();
    Serial.println("Incapable d'envoyer le message!");
  } else {
    Serial.println("Message envoyé");
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
  lcd.print("labo5");
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
