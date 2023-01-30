/******************* Βιβλιοθήκες *******************/

#include <RadioLib.h> // Βιβλιοθήκη Lora.
#include <Servo.h>    // Βιβλιοθήκη σερβοκινητήρα.
#include <TinyGPS++.h> // Βιβιοθήκη GPS.

/******************* Ορισμός Servo-Lora *******************/

Servo claw; // Ονομάζουμε τον ένα σερβοκινητήρα claw.
Servo food; // Ονομάζουμε τον άλλον σερβοκινητήρα food.
Servo turn; // Ονομάζουμε τον άλλον σερβοκινητήρα turn.
Servo motor;// Ονομάζουμε τον άλλον σερβοκινητήρα motor.

SX1278 radio = new Module(10, 2, 9); //Δηλώνω στο digital 10 του atmel συνδέεται το NSS του LORA, digital 2 το DIO0 (Interrupt), digital 9 το RESET.
TinyGPSPlus gps;  // gps object
/******************* Flags/Interrupts *******************/

bool transmitFlag = false;             // Flag που μας δείχνει αν είναι αποστολή η λήψη.
volatile bool enableInterrupt = true;  // Αυτή η μεταβλητή ενεργοποιεί και απενεργοποιεί την διακοπή.
volatile bool operationDone = false;   // Αυτή η μεταβλητή μας δείχνει πότε στάλθηκε/λήφθηκε ένα πακέτο.

boolean validGPS = false; // Μεταβλητή για να δείχνει αν είναι έγκυρα τα gps data.
static unsigned int flag = 0; // Σημαία που δείχνει αν πρέπει να αποθηκεύσει τις πρώτες συντεταγμενές η τις συνεχείς.

/******************* Μεταβλητές *******************/

typedef struct remote // Για να γλιτώσουμε τα περιττά bit χρησιμοποιούμε το struct.
{
  uint8_t Motor;              // Τιμές 0-180.
  uint8_t Pin;                // Κωδικός 231.
  uint8_t PinSend = 213;       // Kωδικός 213.
  uint8_t Led = 5;            // Τιμές 0-255.
  uint8_t satellite;
  uint8_t Turn           : 3; // Τιμές 0-4.
  uint8_t LedW           : 2; // Τιμές 0-3.
  uint8_t JoyButton      : 1; // Τιμές 0 ή 1.
  uint8_t Food           : 1; // Τιμές 0 ή 1.
  uint8_t Claw           : 1; // Τιμές 0 ή 1.
  uint8_t RedLed = A2;   : 1; // Τιμές 0 ή 1.
  uint8_t GreenLed = A0; : 1; // Τιμές 0 ή 1.
};

struct remote Control;

unsigned long SendTime = 1000;         // Η μεταβλητή αυτή θα ελέγχει κάθε πότε θα στέλνει δεδομένα.
static unsigned long previousSend = 0; // Κρατάει την τιμή της προηγούμενης αποστολής.
unsigned long GpsTime = 4000;          // Κάθε πότε θα διαβάζει δεδομένα το gps.
static unsigned long previousGps = 0;  // Κρατάει την τιμή απο την προηγούμενη φορά που διάβασε δεδομένα το gps.

double Lat;
double Lng;
double FirstLat;
double FirstLng;
double Run = 0;
double distanceToDestination = 0;
double courseToDestination;
int courseChangeNeeded;

int TraState = ERR_NONE;   // Αυτή η μεταβλητή ελέγχει την επικοινωνία της αποστολής.
int ReceState = ERR_NONE;  // Αυτή η μεταβλητή ελέγχει την επικοινωνία δέκτη.

uint16_t recePacket[7]; // Αυτός ο πίνακας θα αποθηκεύει τις μεταβλητές που θα λαμβάνει.
double sendPacket[7];    // Αυτός ο πίνακας θα αποθηκεύει τις μεταβλητές που θα στέλνει.

/******************* Έλεγχος Interrupt *******************/

void setFlag(void)
{
  if (!enableInterrupt)
    // Έλεγχος αν το interrupt είναι ενεργοποιημένο.
  {
    return;
  }
  operationDone = true; // Στείλαμε η λάβαμε πακέτο, κάνε true το flag.
}

/******************* Ξεκινάει το πρόγραμμα *******************/

void setup()
{
  Serial.begin(9600);

  pinMode(Control.RedLed, OUTPUT);   // Δηλώνουμε τo pin που ειναι συνδεδεμένο το κόκκινο led έξοδο.
  pinMode(Control.GreenLed, OUTPUT); // Δηλώνουμε τo pin που ειναι συνδεδεμένο το πράσινο led έξοδο.

  radio.begin(433.5,500,8,7,18,14,20,0);

  while (radio.begin() != ERR_NONE)
  {
    digitalWrite(Control.GreenLed, LOW);  // Κλείνει το πράσινο led.
    digitalWrite(Control.RedLed, HIGH);   // Ανάβει το κόκκινο led.
  }
  if (radio.setCurrentLimit(100) == ERR_INVALID_CURRENT_LIMIT)
  {
    digitalWrite(Control.RedLed, HIGH); // Ανάβει το κόκκινο led.
    digitalWrite(Control.GreenLed, LOW);  // Κλείνει το πράσινο led.
  }
  if (radio.setCRC(true) == ERR_INVALID_CRC_CONFIGURATION)  // Ενεργοποιούμε το CRC.
  {
    digitalWrite(Control.GreenLed, LOW);  // Κλείνει το πράσινο led.
    digitalWrite(Control.RedLed, HIGH);   // Ανοίγει το κόκκινο led.
  }
  digitalWrite(Control.GreenLed, HIGH); // Ανάβει το πράσινο led.
  digitalWrite(Control.RedLed, LOW);    // Κλείνει το κόκκινο led.

  claw.attach(A4);
  claw.write(180);
  food.attach(A5);
  food.write(0);
  turn.attach(7);  // Δηλώνω πως ο σερβοκινητήρας που θα γυρνάει το πλοίο θα παιρνει απο το πόδι Α5  το σήμα PWM.
  turn.write(70);
  motor.attach(6, 1000, 2000); // Δηλώνω στο πόδι digital 6 Θα παίρνει ο κινητήρας το σήμα και πως ο ελάχιστος παλμός που θα δέχεται είναι 1000 και ο μέγιστος 2000.

  radio.setDio0Action(setFlag); // Δηλώνω το ποδαράκι του lora Dio0 να δηιαβάζει την fuction setFlag.

#if defined(INITIATING_NODE)
  // send the first packet on this node
  Serial.print(F("[SX1262] Sending first packet ... "));
  TraState = radio.startTransmit((byte*)&sendPacket, 7 * sizeof(double));    transmitFlag = true;
#else
  // start listening for LoRa packets on this node
  Serial.print(F("[SX1262] Starting to listen ... "));
  ReceState = radio.startReceive();
  if (ReceState == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(ReceState);
    while (true);
  }
#endif
}

void loop()
{
  unsigned long currentMillis = millis();

  while (Serial.available()) // Ακούει τα data του gps.
  {
    validGPS = gps.encode(Serial.read()); //Ανάγνωση και κωδικοποίηση δεδομένων gps.
  }

  if (validGPS && gps.location.isUpdated())
  {
    if ( flag == 0)
    {
      FirstLat = gps.location.lat(); // Κρατάνε τις πρώτες συντεταγμένες
      FirstLng = gps.location.lng(); // Κρατάνε τις πρώτες συντεταγμένες
      flag = 1;
    }
    else if (flag == 1)
    {
      Lat = gps.location.lat(); // Κρατάει την τελευταία συντεταγμένη.
      Lng = gps.location.lng(); // Κρατάει την τελευταία συντεταγμένη.
      Control.satellite = gps.satellites.value(); // Κρατάει τον αριθμό των δορυφόρων.
      Run = gps.speed.kmph(); // Αυτή η μεταβλητή θα δείχνει την ταχύτητα του σκάφους.
    }
  }
  if (operationDone)
  {
    enableInterrupt = false;
    operationDone = false;
    if (transmitFlag)
    {
      if (TraState == ERR_NONE)
      {
        digitalWrite(Control.GreenLed, HIGH);  // Ανάβει το πράσινο led.
        digitalWrite(Control.RedLed, LOW);   // Κλείνει το κόκκινο led.
      }
      else
      {
        digitalWrite(Control.GreenLed, LOW);  // Ανάβει το πράσινο led.
        digitalWrite(Control.RedLed, LOW);   // Κλείνει το κόκκινο led.
      }
      radio.startReceive();
      Serial.println("rece");
      transmitFlag = false;
    }
    else
    {
      ReceState = radio.readData((uint8_t*)recePacket, sizeof(recePacket));
      Serial.println("read");
      if (ReceState == ERR_NONE)
      {
        Control.Pin   = recePacket[0];
        Control.Claw  = recePacket[1];
        Control.Food  = recePacket[2];
        Control.LedW  = recePacket[3];
        Control.Turn  = recePacket[4];
        Control.Motor = recePacket[5];
        Control.JoyButton = recePacket[6];

        digitalWrite(Control.GreenLed, HIGH); // Ανάβει το πράσινο led.
        digitalWrite(Control.RedLed, LOW);   // Κλείνει το κόκκινο led.
      }
      else if (ReceState == ERR_CRC_MISMATCH)
      {
        digitalWrite(Control.GreenLed, HIGH); // Ανάβει το πράσινο led.
        digitalWrite(Control.RedLed, HIGH);   // Ανάβει το κόκκινο led.
      }
      else
      {
        digitalWrite(Control.GreenLed, LOW);  // Κλείνει το πράσινο led.
        digitalWrite(Control.RedLed, HIGH);   // Ανάβει το κόκκινο led.
      }
      sendPacket[0] = Control.PinSend;
      sendPacket[1] = Battery();
      sendPacket[2] = Lat;
      sendPacket[3] = Lng;
      sendPacket[4] = Control.satellite;
      sendPacket[5] = Run;
      sendPacket[6] = distanceToDestination;
      TraState = radio.startTransmit((byte*)&sendPacket, 7 * sizeof(double));
      Serial.println("send");
      transmitFlag = true;
    }
    enableInterrupt = true;
  }
  if (Control.Pin == 231)
  {
    CLAW();
    FOOD();
    LEDW();
    TURN();
    motor.write(Control.Motor);
  }

  if (currentMillis - previousGps >= GpsTime)
  {
    previousGps = currentMillis;
    distanceToDestination = TinyGPSPlus::distanceBetween(Lat, Lng, FirstLat, FirstLng); // Απόσταση
    courseToDestination = TinyGPSPlus::courseTo(Lat, Lng, FirstLat, FirstLng);
    const char *directionToDestination = TinyGPSPlus::cardinal(courseToDestination);
    courseChangeNeeded = (int)(360 + courseToDestination - gps.course.deg()) % 360;
  }
}

/******************* Έλεγχος δαγκάνας *******************/

void CLAW()
{
  if (Control.Claw == 0)
  {
    claw.write(180);
  }
  else if (Control.Claw == 1)
  {
    claw.write(0);
  }
}

/******************* Έλεγχος μαλάγρα *******************/

void FOOD()
{
  if (Control.Food == 0)
  {
    food.write(0);
  }
  else if (Control.Food == 1)
  {
    food.write(180);
  }
}

/******************* Έλεγχος led *******************/

void LEDW()
{
  if (Control.LedW == 0)
  {
    analogWrite(Control.Led, 0);
  }
  else if (Control.LedW == 1)
  {
    analogWrite(Control.Led, 85);
  }
  else if (Control.LedW == 2)
  {
    analogWrite(Control.Led, 170);
  }
  else if (Control.LedW == 3)
  {
    analogWrite(Control.Led, 255);
  }
}

/******************* Στροφή πλοίου *******************/

void TURN()
{
  static int go = 0;


  if (Control.Turn == 0)
  {
    turn.write(41);
    go = 0;
  }
  else if (Control.Turn == 1)
  {
    turn.write(55);
    go = 0;
  }
  else if (Control.Turn == 2)
  {
    if (go == 0)
    {
      turn.write(68);
    }
    else if (go == 1)
    {
      turn.write(63);
    }
  }
  else if (Control.Turn == 3)
  {
    turn.write(75);
    go = 1;
  }
  else if (Control.Turn == 4)
  {
    turn.write(90);
    go = 1;
  }
}

/******************* Τάση μπαταρίας  *******************/

int Battery()
{
  static int BatteryVolt = 0;
  int readAnalogVolt = analogRead(A1);

  float Value = ((readAnalogVolt * 5) / 1024) + 0.7 ;
  float Volt = Value / 0.394;

  if (Volt >= 12.5)
  {
    BatteryVolt = 100;
  }

  else if ((Volt < 12.5) && (Volt >= 12.2))
  {
    BatteryVolt = 90;
  }
  else if ((Volt < 12.2) && (Volt >= 11.9))
  {
    BatteryVolt = 80;
  }

  else if ((Volt < 11.9) && (Volt >= 11.6))
  {
    BatteryVolt = 70;
  }

  else if ((Volt < 11.6) && (Volt >= 11.3))
  {
    BatteryVolt = 60;
  }
  else if ((Volt < 11.3) && (Volt >= 11))
  {
    BatteryVolt = 50;
  }

  else if ((Volt < 11) && (Volt >= 10.8))
  {
    BatteryVolt = 40;
  }

  else if  ((Volt < 10.8) && (Volt >= 10.6))
  {
    BatteryVolt = 30;
  }

  else if (Volt < 10.6 && Volt >= 10.5)
  {
    BatteryVolt = 20;
  }

  else if (Volt < 10.5 && Volt >= 10.3)
  {
    BatteryVolt = 10;
  }
  else
  {
    BatteryVolt = 0;
  }

  return BatteryVolt;
}
