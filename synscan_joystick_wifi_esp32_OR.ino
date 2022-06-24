// Librairies
#include <WiFi.h>
#include <WiFiUdp.h>

typedef enum {
  Stopped,
  Moving,
  Undefined
} motor_state_t;

// Constantes & variables
const char* ip_serveur = "192.168.4.1";      // Adresse IP du téléescope (serveur)
const char* ssid = "SynScanDuino";           // Nom du réseau
const char* password = "123456789";          // mot de passe du réseau
int local_port = 11880;                      // port de communication pour l'échange UDP
#define bufferSize 8192
uint8_t udpBuffer[bufferSize];

#define PIN_JOY_V   34
#define PIN_JOY_H   35
#define PIN_JOY_BTN 32

int center_V = 1905;
int center_H = 1910;

int new_val_H = 0; int new_val_V = 0;
int old_val_H = 0; int old_val_V = 0;
boolean mode_vitesse = 1;

motor_state_t mot_busy = Undefined;

#define LED_ON 26
#define LED_WIFI 25
#define LED_MODE 27

// Instances
WiFiUDP udp;


void setup() {
  pinMode(PIN_JOY_H, INPUT);                 //Init Joystick vertical
  pinMode(PIN_JOY_V, INPUT);                 //Init Joystick horizontal
  pinMode(PIN_JOY_BTN, INPUT_PULLUP);        //Init Joystick switch
  pinMode(LED_ON, OUTPUT);                   //Init des LEDS
  pinMode(LED_MODE, OUTPUT);
  pinMode(LED_WIFI, OUTPUT);

  digitalWrite(LED_ON, HIGH);

  WiFi.disconnect();
  Serial.begin(115200);                     // INITIALISATION COMMUNICATION AVEC PC

  Serial.print("Connexion au téléscope en cours");
  delay(1000);

  WiFi.mode(WIFI_AP);                       // INITIALISATION WIFI
  IPAddress ip(192, 168, 43, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(ip, ip, subnet);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {   // Tant que la connexion n'est pas établie, on boucle
    delay(500);
    Serial.print(".");
  }
  digitalWrite(LED_WIFI, HIGH);
  Serial.println(""); Serial.println("Connexion au téléscope établie");

  Serial.println("Démarrage udp");
  udp.begin(local_port);             // initialisation connexion UDP
  delay(1000);

  digitalWrite(LED_MODE, HIGH);
}

void loop() {
  lectureJoystick();  
  if (new_val_V != old_val_V) {      // si la nouvelle valeur est identique à l'ancienne, on ne renvoit pas la même commande
    arret_mot_Alt();                 //"Envoi commande :K"
    arret_mot_AZ();                  // stop moteur en cas de mvt diagonal
    if (new_val_V > 0 and old_val_V < 0 and mode_vitesse == 1 ) {Serial.println("tempo"); delay(1400);} 
    if (new_val_V < 0 and old_val_V > 0 and mode_vitesse == 1) {Serial.println("tempo"); delay(1400);} 
    if (abs(old_val_V) == 2 and new_val_V == 0  and mode_vitesse == 1) {Serial.println("tempo"); delay(1400);} 
    
    if (new_val_V != 0) {
      wait_mot_Alt();                //"Envoi commande :f"
      sens_mot_Alt();                //"Envoi commande :G"
      vitesse_mot_Alt();             //"Envoi commande :I"
      depart_mot_Alt();              //"Envoi commande :J"
    }
    old_val_V = new_val_V ;
  }

  if (new_val_H != old_val_H) {      // si la nouvelle valeur est identique à l'ancienne, on ne renvoit pas la même commande
    arret_mot_AZ();                  //"Envoi commande :K"
    arret_mot_Alt();                 // en cas de mvt diagonal
    if (new_val_H > 0 and old_val_H < 0 and mode_vitesse == 1) {Serial.println("tempo"); delay(1400);} 
    if (new_val_H < 0 and old_val_H > 0 and mode_vitesse == 1 ) {Serial.println("tempo"); delay(1400);} 
    if (abs(old_val_H) == 2 and new_val_H == 0 and mode_vitesse == 1) {Serial.println("tempo"); delay(1400);} 
    
    if (new_val_H != 0) {
      wait_mot_AZ();                 //"Envoi commande :f"
      sens_mot_AZ();                 //"Envoi commande :G"
      vitesse_mot_AZ();              //"Envoi commande :I"
      depart_mot_AZ();               //"Envoi commande :J"
      old_val_H = new_val_H ;
    }
    old_val_H = new_val_H ;
  }
  delay(100);
}

void lectureJoystick() {
  //lecture joystick
  int joyVal_V = analogRead(PIN_JOY_V) - center_V; // lecture vertical
  new_val_V = map(joyVal_V, -2047, 2048, -2, 2);
  int joyVal_H = analogRead(PIN_JOY_H) - center_H; // lecture horizontal
  new_val_H = map(joyVal_H, -2047, 2048, -2, 2);

  byte joyBtn = digitalRead(PIN_JOY_BTN);          // lecture btn
  if (joyBtn == 0) {
       delay(2500);
       byte joyBtn = digitalRead(PIN_JOY_BTN);  
       if (joyBtn == 0) {mode_vitesse = not(mode_vitesse);}
    }
    if (mode_vitesse == 1) {digitalWrite(LED_MODE, HIGH);}
    if (mode_vitesse == 0) {digitalWrite(LED_MODE, LOW);}                // mode 0 = lent ; mode 1 = rapide
}


// ---------------------------------------------------------------
// INQUIRE MOTEUR     -     INQUIRE MOTEUR    -    INQUIRE MOTEUR
// ---------------------------------------------------------------
void wait_mot_AZ()
{
   commandeTelescope(":f1\r");
}
void wait_mot_Alt()
{
  commandeTelescope(":f2\r");
}

// ---------------------------------------------------------
// ARRET MOTEUR     -     ARRET MOTEUR    -    ARRET MOTEUR
// ---------------------------------------------------------
void arret_mot_AZ()
{
  commandeTelescope(":K1\r");
}
void arret_mot_Alt()
{
  commandeTelescope(":K2\r");
}

// ------------------------------------------------------------
// DEPART MOTEUR     -     DEPART MOTEUR    -    DEPART MOTEUR
// ------------------------------------------------------------
void depart_mot_AZ()
{
  commandeTelescope(":J1\r");
}
void depart_mot_Alt()
{
  commandeTelescope(":J2\r");
}

// ---------------------------------------------------------------
// VITESSE MOTEUR     -     VITESSE MOTEUR    -    VITESSE MOTEUR
// ---------------------------------------------------------------
void vitesse_mot_AZ()
{
  if (abs(new_val_H) == 2) {
    if (mode_vitesse == 1) {commandeTelescope(":I1280400\r");}
    if (mode_vitesse == 0) {commandeTelescope(":I1F13300\r");}
  }
  if (abs(new_val_H) == 1) {
    if (mode_vitesse == 1) {commandeTelescope(":I1500800\r");}
    if (mode_vitesse == 0) {commandeTelescope(":I1C4CF00\r");}
  }
}
void vitesse_mot_Alt()
{
  if (abs(new_val_V) == 2) {
    if (mode_vitesse == 1) {commandeTelescope(":I2280400\r");}
    if (mode_vitesse == 0) {commandeTelescope(":I2F13300\r");}
  }
  if (abs(new_val_V) == 1) {
    if (mode_vitesse == 1) {commandeTelescope(":I2500800\r");}
    if (mode_vitesse == 0) {commandeTelescope(":I2C4CF00\r");}
  }
}

// ---------------------------------------------------------------
// SENS MOTEUR     -     SENS MOTEUR    -    SENS MOTEUR
// ---------------------------------------------------------------
void sens_mot_AZ()
{
  if (new_val_H > 0) {commandeTelescope(":G130\r");}
  if (new_val_H < 0) {commandeTelescope(":G131\r");}
}
void sens_mot_Alt()
{
  if (new_val_V > 0) {commandeTelescope(":G230\r");}
  if (new_val_V < 0) {commandeTelescope(":G231\r");}
}

void commandeTelescope(const char* cmd) {
  udp.beginPacket(ip_serveur, local_port);
  udp.print(cmd);
  udp.endPacket();
  delay(50);
}

// ---------------------------------------------------------------
// AFFICHAGE SUR REPONSE TELESCOPE SUR PC
// ---------------------------------------------------------------
bool isReponseTelescope()
{
  udp.flush();
  int packetSize = udp.parsePacket();
  if (packetSize) {
  //  Serial.printf("Reçus %d bytes de %s, port %d\n", packetSize, udp.remoteIP().toString().c_str(), udp.remotePort());
    int len = udp.read(udpBuffer, 255);
    if (len >= 2) { // au moins 2 caractères pédondus
      udpBuffer[len] = '\0';
      Serial.printf("Contenu UDP: %s\n", udpBuffer);
     // Serial.println(udpBuffer[2]);
      if (udpBuffer[2] == '1') {mot_busy = Moving;}
      if (udpBuffer[2] == '0') {mot_busy = Stopped;}
      return true; // on a reçu un commande
    }
  }
  return false; // aucune réponse ou pas la bonne
}
