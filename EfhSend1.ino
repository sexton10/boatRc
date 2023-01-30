/******************* Βιβλιοθήκες *******************/

#include <RadioLib.h>           // Βιβλιοθήκη για το Lora.
#include <LiquidCrystal_I2C.h>  // Βιβλιοθήκη για την LCD.
#include <Wire.h>               // Βιβλιοθήκη για το I2C

/******************* Ορισμός LCD-Lora *******************/

LiquidCrystal_I2C lcd(0x27, 20, 4);   //Δηλώνω  πως η LCD ειναι 20 χαρακτήρων 4 σειρών. Το 0χ27 υποδηλώνει την διεύθυνση του I2C.
SX1278 radio = new Module(10, 2, 9);  //Δηλώνω στο digital 10 του atmel συνδέεται το NSS του LORA, digital 2 το DIO0 (Interrupt), digital 9 το RESET.

/******************* Σχηματισμός στην LCD *******************/

byte fish[8] =   // Δημιουργεί ένα ψάρι.
{
  B00000,
  B00000,
  B10110,
  B11001,
  B11001,
  B10110,
  B00000,
  B00000
};

byte lock[8] =  // Δημιουργεί μία κλειδαριά .
{
  B01110,
  B10001,
  B10001,
  B11111,
  B11011,
  B11011,
  B11111,
  B11111
};

byte LockOpen[8] // Δημιουργεί μία ανοιχτή κλειδαριά.
{
  B00110,
  B00001,
  B00001,
  B11111,
  B11011,
  B11011,
  B11111,
  B11111
};

byte ArrowRight[8] = // Δημιουργεί ένα βέλος δεξιά.
{
  B00000,
  B00100,
  B00010,
  B11111,
  B00010,
  B00100,
  B00000,
  B00000
};

byte ArrowLeft[8] =  // Δημιουργεί ένα βέλος αριστερά.
{
  B00000,
  B00100,
  B01000,
  B11111,
  B01000,
  B00100,
  B00000,
  B00000
};

byte Ret[8] = // Δημιουργεί ένα σήμα που δείχνει οτι επιστρέφει.
{
  B00100,
  B01000,
  B01111,
  B01001,
  B10101,
  B10001,
  B11111,
  B00000
};
/******************* Flags/Interrupts *******************/

bool transmitFlag = false;             // Flag που μας δείχνει αν είναι αποστολή η λήψη.
volatile bool enableInterrupt = true;  // Αυτή η μεταβλητή ενεργοποιεί και απενεργοποιεί την διακοπή.
volatile bool operationDone = false;   // Αυτή η μεταβλητή μας δείχνει πότε στάλθηκε/λήφθηκε ένα πακέτο.

/******************* Μεταβλητές *******************/

typedef struct remote   // Για να γλιτώσουμε τα περιττά bit χρησιμοποιούμε το struct.
{
  uint16_t Pot              : 10;   // Τιμές 0-1023.
  uint16_t JoystickY        : 10;   // Τιμές 0-1023.
  uint16_t JoystickX        : 10;   // Τιμές 0-1023.
  uint8_t  motor;                   // Τιμές 0-180.
  uint8_t  Degrees          : 3;    // Τιμές 0-4.
  uint8_t  LedW = 0;        : 2;    // Τιμές 0-3.
  uint8_t  Meter = 0;       : 2;    // Τιμές 0-3.
  uint8_t satellite;
  uint8_t  Pin = 231;
  uint8_t PinSend;
  uint8_t  Food             : 1;    // Τιμές 0 ή 1.
  uint8_t  ReadFood         : 1;    // Τιμές 0 ή 1.
  uint8_t  Dagana;          : 1;    // Τιμές 0 ή 1.
  uint8_t  ReadDagana       : 1;    // Τιμές 0 ή 1.
  uint8_t RedLed = 6;       : 1;    // Τιμές 0 ή 1. Επίσης δηλώνω το κόκκινο ποδαράκι είναι το Digital 6.
  uint8_t GreenLed = 7;     : 1;    // Τιμές 0 ή 1. Επίσης δηλώνω το κόκκινο ποδαράκι είναι το Digital 7.
  uint8_t Button            : 1;
};

struct remote Control;

double Lng;     // Μεταβλητή για το lng.
double Lat;     // Μεταβλητή για το Lat.
double Distance =0;
int Run;
int timh = 0;  // Mεταβλητή για την μπαταρία.

uint16_t sendPacket[7]; // Αυτός ο πίνακας θα αποθηκεύει τις μεταβλητές που θα στέλνει.
double recePacket[7];    // Αυτός ο πίνακας θα αποθηκεύει τις μεταβλητές που θα λαμβάνει.

unsigned long SendTime = 1000;
unsigned long previousSend = 0;
unsigned long LcdTime = 4000;
unsigned long previousLcd = 0;

int TraState = ERR_NONE;   // Αυτή η μεταβλητή ελέγχει την επικοινωνία της αποστολής.
int ReceState = ERR_NONE;  // Αυτή η μεταβλητή ελέγχει την επικοινωνία δέκτη.

/******************* Έλεγχος Interrupt *******************/

void setFlag(void)
{
  // Έλεγχος αν το interrupt είναι ενεργοποιημένο.
  if (!enableInterrupt)
  {
    return;
  }
  operationDone = true;  // Στείλαμε η λάβαμε πακέτο, κάνε true το flag.
}

/******************* Ξεκινάει το πρόγραμμα *******************/

void setup()
{
  Serial.begin(9600);
  lcd.init();                // Αρχικοποιήση LCD

  radio.begin( );
  //Frequency        : 433.5 MHz
  //Bandwidth        : 500 kHz
  //Spreading factor : 8
  //Coding rate      : 7
  //Sync word        : 0x18
  //Output power     : 14
  //Preamble length  : 20
  //Amplifier gain   : 0

  pinMode(Control.RedLed, OUTPUT);   // Δηλώνουμε τo pin που ειναι συνδεδεμένο το κόκκινο led έξοδο.
  pinMode(Control.GreenLed, OUTPUT); // Δηλώνουμε τo pin που ειναι συνδεδεμένο το πράσινο led έξοδο.
  pinMode(5, INPUT);                 // Δηλώνουμε το digital 5 είσοδο.
  pinMode(4, INPUT);                 // Δηλώνουμε το digital 4 είσοδο.
  pinMode(A1, INPUT);
  
  /**************** ΞΕΚΙΝΑΕΙ Η LCD ****************/


  lcd.backlight();               // Ανοίγει το backlight της οθόνης.
  lcd.setCursor(8, 0);           // Γράφει στην LCD στην πρώτη σειρά έκτη στήλη.
  lcd.print("E.F.H");            // Εκτύπωση μηνύματος.
  lcd.createChar(0, fish); // Φτιάχνει τον χαρακτήρα.

  for ( int i = 0; i < 19; i++) // Θα μετακινήσει το ψάρι που βρίσκεται στην δεύτερη σειρά.
  {
    lcd.setCursor(i, 1);         // Αλλάζει την στήλη.
    lcd.write(((byte)0));        // Εμφανίζεται στην κενούργια στήλη ο χαρακτήρας.
    delay(100);
    lcd.setCursor(i, 1);
    lcd.write(' ');              // Διαγράφει τον χαρακτήρα.
  }

  /**************** ΕΛΕΓΧΟΣ ΑΝ ΥΠΑΡΧΕΙ ΣΦΑΛΜΑ ΣΤΟ lORA  ****************/
  int j = 0;

  while (radio.begin() != ERR_NONE)    // Σε περίπτωση που το Lora.begin έχει κάποια σφάλμα μπαίνει σε αυτή την while.
  {
    for ( int i = 0; i < 15 ; i++)   // Σε αυτή την for θα μετακινεί το ψάρι στην οθόνη όσο κάνει έλεγχο την Lora.begin.
    {
      lcd.setCursor(i, 1);  // Ελέγχει σε ποια θέση θα είναι το ψάρι.
      lcd.write(((byte)0)); // Δημιουργεί το ψάρι.
      delay(100);
      lcd.setCursor(i, 1);
      lcd.write(' ');      // Διαγράφει το ψάρι.
    }
    delay(100);
    j++;
    digitalWrite(Control.GreenLed, LOW);  // Κλείνει το πράσινο led.
    digitalWrite(Control.RedLed, HIGH);   // Ανάβει το κόκκινο led.

    if ( j > 10)    // Όταν κάνει 10 φορές τον έλεγχο αν δεν διορθωθεί θα μπεί σε αυτη την for και θα μας δείξει η lcd
    { // για να κάνουμε restart το τηλεχειριστήριο.
      lcd.clear();
      lcd.setCursor(4, 0);

      for (;;)
      {
        lcd.print("restart");
        digitalWrite(Control.RedLed, HIGH); // Ανοίγει το κόκκινο Led.
      }
    }

  }//end while
  lcd.clear();  //Καθαρίζει την LCD.
  if (radio.setCurrentLimit(100) == ERR_INVALID_CURRENT_LIMIT) // Ορίζουμε 100mA ρεύμα.
  { 
    digitalWrite(Control.GreenLed, LOW);  // Κλείνει το πράσινο led.
    digitalWrite(Control.RedLed, HIGH);   // Ανοίγει το κόκκινο led.
  }
  if (radio.setCRC(true) == ERR_INVALID_CRC_CONFIGURATION)  // Ενεργοποιούμε το CRC.
  { 
    digitalWrite(Control.GreenLed, LOW);  // Κλείνει το πράσινο led.
    digitalWrite(Control.RedLed, HIGH);   // Ανοίγει το κόκκινο led.
  }
  digitalWrite(Control.GreenLed, HIGH);   // Αν δεν έχει κάποιο το Lora.begin τότε ανάβει το πράσινο led/
  digitalWrite(Control.RedLed, LOW);      // Κλείνει το κόκκινο led.
  
  lcd.setCursor(15, 1);   // Oρίζουμε θέση.
  lcd.print("LED:");      // Εκτύπωνει LED:
  lcd.setCursor(0, 0);
  lcd.print(F("LAT:"));
  lcd.setCursor(0, 1);
  lcd.print(F("LNG:"));
  lcd.setCursor(0, 2);
  lcd.print(F("SPEED:"));
  lcd.setCursor(0, 3);
  lcd.print("DISTANCE:");
  delay(2000);           // Καθυστέρησε 2 δευτερόλεπτα.


  /**************** Τέλος ελέγχου ****************/
  radio.setDio0Action(setFlag);   // Δηλώνω το ποδαράκι του lora Dio0 να δηιαβάζει την fuction setFlag.

  TraState = radio.startTransmit((uint8_t*)sendPacket, sizeof(sendPacket)); //Στέλνουμε το πρώτο πακέτο.
  transmitFlag = true; // Μόλις στείλουμε το πρώτο πακέτο το κάνουμε true για να μπει στην λειτουργία λήψης.
}

void loop()
{
  unsigned long currentMillis = millis(); // Αυτή η μεταβλητή θα μετράει μόνιμα.
 
  sendPacket[0] = Control.Pin;  // Δηλώνουμε κάθε θέση του πίνακα να παίρνει μία μεταβλητή.
  sendPacket[1] = claw();
  sendPacket[2] = food();
  sendPacket[4] = turn();
  sendPacket[5] = DcMotor();

  if ( currentMillis - previousSend >= SendTime)
  {
    previousSend = currentMillis;
    sendPacket[3] = led();   // Η θέση τρία του πίνακα θα παίρνει την τιμή του led Κάθε ένα δευτερόλεπτο.
    sendPacket[6] = JoyButton();
    TraState = radio.startTransmit((uint8_t*)sendPacket, sizeof(sendPacket));
    transmitFlag = true;     // Μόλις στείλουμε το πρώτο πακέτο το κάνουμε true για να μπει στην λειτουργία λήψης.
    Serial.println("send");
  }

  if (operationDone)  //Στείλαμε η λάβαμε πακέτο άρα γίνεται true και μπαίνει στο if.
  {
    enableInterrupt = false; // Κλείνουμε το Interrupt μέχρι να κάνουμε την δουλειά που χρειάζεται.
    operationDone = false;   // Κάνουμε reset το flag

    if (transmitFlag)        // Αν είχαμε στείλει πριν πακέτο και είναι η σημαία true μπαίνει στο if.
      // Αν ήταν στο receive mode τότε συνεχίζουμε τον κώδικα χωρίς να μπούμε σε αυτό το if.
    {
      if (TraState == ERR_NONE)
      {
        digitalWrite(Control.GreenLed, HIGH);   // Αν στάλθηκε με επιτυχία  τότε ανάβει το πράσινο led.
        digitalWrite(Control.RedLed, LOW);     // Κλείνει το κόκκινο.
      }
      else
      {
        digitalWrite(Control.GreenLed, LOW);   // Αν υπήρξε σφάλμα κλείνει το πράσινο LED.
        digitalWrite(Control.RedLed, HIGH);    // Ανάβει το κόκκινο.
      }
      radio.startReceive();                    // Πάμε στο Listen mode
      Serial.println("rece");
      transmitFlag = false;                    // Όταν είμαστε στο listen mode το trasmitFlag πρέπει να είναι false.
    }

    else
    {
     ReceState = radio.readData((byte*)&recePacket,7*sizeof(double)); // Έρχεται ειδοποιήση απο το interrupt οτι δέχτηκε η στάλθηκε πακέτο,τότε ελέγχει ο μικροελεγκτής την σημαία transmitFlag αν είναι false ξέρει οτι ήρθε πακέτο και πρέπει να διαβάσει τα δεδομένα. 
       Serial.println("read");
     if (ReceState == ERR_NONE)  // Αν το πακέτο είναι εντάξει τότε μπές στο If αλλιώς τρέξε το else. 
     { 
       Battery();       // Καλούμε την function.
       Control.PinSend = recePacket[0]; // Λέμε στις μεταβλητές να διαβάσουν τις τιμές απο το πακέτο που λάβαμε.
       timh = recePacket[1];  
       Lat = recePacket[2];   
       Lng = recePacket[3];      
       Control.satellite= recePacket[4];  
       Run = recePacket[5];
       Distance = recePacket[6];                                                                                                         
       digitalWrite(Control.GreenLed, HIGH);
       digitalWrite(Control.RedLed, LOW);
     }
      else if(ReceState == ERR_CRC_MISMATCH)
     {
      digitalWrite(Control.GreenLed, HIGH); // Ανάβει το πράσινο led.
      digitalWrite(Control.RedLed, HIGH);   // Ανάβει το κόκκινο led. 
     }
     else
     {
      timh = 0;
      digitalWrite(Control.GreenLed, LOW); // Κλείνει το πράσινο led. 
      digitalWrite(Control.RedLed, HIGH);  // Ανάβει το κόκκινο led. 
     }
    }
    enableInterrupt = true;    // Ενεργοποιούμε το interrupt.
  }

  
  
 if(currentMillis - previousLcd >= LcdTime)
 {
   
  static double prevLat=0;
  static double prevLng=0;
  static double prevsatellite=0;
  static float prevRun=0;
  static int prevtimh=0;
  static double prevDestance = 0;
   previousLcd = currentMillis;
  if(Control.PinSend == 213)
  {
   if(timh!=prevtimh)
   {
    lcd.setCursor(15, 0);  // Στην πρώτη σειρά στην θέση 16.
    lcd.print(" ");
    lcd.setCursor(15, 0);  // Στην πρώτη σειρά στην θέση 16.
    lcd.print(timh);       // Εκτυπώνουμε την μεταβλητή.
    lcd.print(F("%"));
    prevtimh =timh;
   }
   if(Lat!=prevLat)
   {
    lcd.setCursor(4, 0);  // Στην πρώτη σειρά στην θέση 16.
    lcd.print(Lat,6);       // Εκτυπώνουμε την μεταβλητή. 
   lcd.print(" ");
    prevLat = Lat;   
   }
   if(Lng!=prevLng)
   {
    lcd.setCursor(4,1);
    lcd.print(Lng,6);
   lcd.print(" ");
    prevLng = Lng;
   }
   if(Control.satellite!=prevsatellite)
   {
    lcd.setCursor(18,3);
    lcd.print(Control.satellite);  
      lcd.print(" ");
   }
   if(Run!=prevRun)
   {
    if(Run<= 7)
    {
     Run=0;
     lcd.setCursor(6,2);
     lcd.print(Run);
     prevRun=Run;
    }
    else
    {
    lcd.setCursor(6,2);
    lcd.print(Run);
    lcd.print(" ");
    prevRun = Run;
    }
   }
   if(Distance!=prevDestance)
   {
   lcd.setCursor(9,3);
   lcd.print(Distance/1000,3);
   lcd.print("   ");
   prevDestance = Distance;
   }
  }
 }
}

/******************* Ρίχνει το αγκίστρι *******************/

int claw()              
{ 
  static unsigned int Lock = 0; // Χρησιμοποιούμαι αυτή την μεταβλητή ώστε να μην γράφουμε όλη την ώρα στην οθόνη το ίδιο σύμβολο.
  static unsigned int Open = 0; // Χρησιμοποιούμαι αυτή την μεταβλητή ώστε να μην γράφουμε όλη την ώρα στην οθόνη το ίδιο σύμβολο.
  Control.ReadDagana = digitalRead(5);   // Σε αυτή την function Θα ελέγχει το ποδαράκι digital 5 το button αν είναι low η high.       

  if ( Control.ReadDagana == LOW ) // Αν είναι low επιστρέφτει την τιμή 0 και εκτυπώνει μία κλειδαριά στην LCD.
  { 
     Control.Dagana = 0;  
  
    if( Lock == 0)
    {
           
   lcd.createChar(5 , lock);
   lcd.setCursor(17, 2);
   lcd.write(byte(5));
   }
    Lock = 1;
    Open = 0;
  }
  else
  {
    Control.Dagana = 1;  // Αν είναι High τότε επιστρέφει την τιμή 1 και  εκτυπώνει μία ανοιχτεί κλειδαριά.
    if(Open == 0)
    {
     lcd.createChar(6 , LockOpen);
     lcd.setCursor(17, 2);
     lcd.write(byte(6));
    }
    Lock = 0;
    Open = 1;
  }
  return Control.Dagana;
}

/******************* Ρίχνει την μαλάγρα *******************/

int food()
{
  static unsigned int Lock = 0; // Χρησιμοποιούμαι αυτή την μεταβλητή ώστε να μην γράφουμε όλη την ώρα στην οθόνη το ίδιο σύμβολο. 
  static unsigned int Open = 0; // Χρησιμοποιούμαι αυτή την μεταβλητή ώστε να μην γράφουμε όλη την ώρα στην οθόνη το ίδιο σύμβολο.
  Control.ReadFood = digitalRead(4);  // Ελέγχει το button αν ειναι high η low μέσο του ποδαράκι digital 4.

  if ( Control.ReadFood == LOW ) // Αν είναι low επιστρέφτει την τιμή 0 και εκτυπώνει μία κλειδαριά στην LCD.
  {
    Control.Food = 0;
    if( Lock == 0)
    {
      lcd.createChar(5 , lock);
      lcd.setCursor(18, 2);
      lcd.write(byte(5));
    }
    Lock = 1;
    Open = 0;
  }
  else                     // Αν είναι High τότε επιστρέφει την τιμή 1 και  εκτυπώνει μία ανοιχτεί κλειδαριά.
  {
    Control.Food = 1;
    if(Open == 0)
    {
      lcd.createChar(6 , LockOpen);
      lcd.setCursor(18, 2);
      lcd.write(byte(6));
    }
    Open = 1;
    Lock = 0;
  }
  return Control.Food;
}

/******************* Έλεγχος led *******************/

int led()           
{

  Control.JoystickX  = analogRead(A2); // Η αναλογική πόρτα Α2 διαβάζει τον Χ άξονα του joystick.
  Control.JoystickY = analogRead(A3); // Η αναλογική πόρτα Α2 διαβάζει τον Y άξονα του joystick.

  if ((Control.JoystickY > 800) && (Control.JoystickX > 400 ) && ( Control.JoystickX < 750)) // Αν πάει πάνω το joystick ανέβασε την φωτεινότητα +1 αν φτάσει το 3 τότε stop.
  {
    if (Control.Meter == 0) 
    {
      Control.LedW = 0;
      Control.Meter = 1;
    }
    else if (Control.Meter  == 1)
    {
      Control.LedW = 1;
      Control.Meter = 2;
    }
    else if (Control.Meter  == 2)
    {
      Control.LedW = 2;
      Control.Meter = 3;
    }
    else if (Control.Meter  == 3)
    {
      Control.LedW = 3;
    }
  }// Τέλος if++

  else if ((Control.JoystickY < 300) && (Control.JoystickX > 400 ) && ( Control.JoystickX < 750)) // Αν πάει κάτω το Joystick κατέβασε την φωτεινότητα -1 αν φτάσει 0 τότε stop. 
  {
    if (Control.Meter == 0)
    {
      Control.LedW = 0;
    }
    else if (Control.Meter  == 1)
    {
      Control.LedW = 1;
      Control.Meter = 0;
    }
    else if (Control.Meter == 2)
    {
      Control.LedW = 2;
      Control.Meter = 1;
    }
    else if (Control.Meter  == 3)
    {
      Control.LedW = 3;
      Control.Meter = 2;
    }
  }// Τέλος if --
  lcd.setCursor(19, 1);        // Εκτύπωσε το επίπεδο φωτεινότητας στην θέσει 19 στην δεύτερη σειρά. 
  lcd.print(Control.LedW);

  return Control.LedW;
}

/******************* Στροφή πλοίου *******************/

int turn()
{
  /* Eλέγχει αν είναι το joystick δεξιά, αριστερά η στην μέση. Ανάλογα με την θέση του πάει στο ανάλογο If.
   *  Αν ειναι δεξιά στέλνει έναν αριθμό και σχηματίζει ενα βελάκι δεξιά. 
   */
   static unsigned int right =0 ;
   static unsigned int left = 0;
  Control.JoystickX  = analogRead(A2); // Η αναλογική πόρτα Α2 διαβάζει τον Χ άξονα του joystick.
  Control.JoystickY  = analogRead(A3); // H αναλογική πόρτα A3 διαβάζει τον Υ άξονα του joystick.

 if ((Control.JoystickX <= 1023 ) && ( Control.JoystickX > 950) && ( Control.JoystickY > 400) && ( Control.JoystickY < 800)) 
  {
    Control.Degrees = 0;   
   if(right == 0)
   {                    
    lcd.createChar(2 , ArrowRight);
    lcd.setCursor(19, 2);
    lcd.write(byte(2));
   }
  right = 1;
  left = 0;
 }

 else if((Control.JoystickX <= 950 ) && ( Control.JoystickX >= 750) && ( Control.JoystickY > 400) && ( Control.JoystickY < 800))
 {
   Control.Degrees = 1;   
   if(right == 0)
   {                    
    lcd.createChar(2 , ArrowRight);
    lcd.setCursor(19, 2);
    lcd.write(byte(2));
   }
   right = 1;
   left = 0;
  }
  else if ((Control.JoystickX > 400 ) && ( Control.JoystickX < 750) && ( Control.JoystickY > 400) && ( Control.JoystickY < 800))
  {
    Control.Degrees = 2;
    lcd.setCursor(19, 2);
    lcd.print(" ");
    lcd.setCursor(16, 2);
    lcd.print(" ");
    right  = 0;
    left = 0;   
  }
  else if ((Control.JoystickX <= 400 ) && ( Control.JoystickX > 200) && ( Control.JoystickY > 400) && ( Control.JoystickY < 800) )
  {
    Control.Degrees = 3; 
    if (left == 0)
    {
      lcd.createChar(3 , ArrowLeft);
      lcd.setCursor(16, 2);
      lcd.write(byte(3));
    }
    left = 1;
    right = 0;
  }

   else if ((Control.JoystickX <= 200 ) && ( Control.JoystickX >= 0) && ( Control.JoystickY > 400) && ( Control.JoystickY < 800) )
  {
    Control.Degrees = 4; 
    if (left == 0)
    {
      lcd.createChar(3 , ArrowLeft);
      lcd.setCursor(16, 2);
      lcd.write(byte(3));
    }
    left = 1;
    right = 0;
  }
  return Control.Degrees;
}


/******************* Έλεγχος στροφή κινητήρα *******************/

int DcMotor()
{

  Control.Pot = analogRead(A0);                          // H αναλογική είσοδος A0 διαβάζει την τάση που θα δέχεται απο το ποτενσιόμετρο.
  Control.motor = map(Control.Pot , 0 , 1023 , 0 , 180); // Μετατρέπουμε τις τιμές 0-1023 σε 0-180 μοίρες.

  return  Control.motor;                              // Επιστρέφει την τιμή της μεγαλητής.
}

/******************* Σχηματίζει την μπαταρία ανάλογα με την τάση  *******************/
void Battery()
{

  if (timh <= 100 && timh > 90) // Βλέπει τι αριθμός θα φτάσει απο το σκάφος και το τοποθετεί στο ανάλογο If και σχεδιάζει την μπαταρία με την κατάλληλη τάση.
  {
    byte batlevel[8] =
    {
      B01110,
      B11111,
      B10101,
      B10001,
      B11011,
      B11011,
      B11111,
      B11111,
    };
    lcd.createChar(1 , batlevel);
    lcd.setCursor(19, 0);
    lcd.write(byte(1));
  }
  if (timh <= 90 && timh > 80)
  {
    byte batlevel[8] =
    {
      B01110,
      B11111,
      B11111,
      B11111,
      B11111,
      B11111,
      B11111,
      B11111,
    };
    lcd.createChar(1 , batlevel);
    lcd.setCursor(19, 0);
    lcd.write(byte(1));
  }
  if (timh <= 80 && timh > 70)
  {
    byte batlevel[8] =
    {
      B01110,
      B10001,
      B11111,
      B11111,
      B11111,
      B11111,
      B11111,
      B11111,
    };
    lcd.createChar(1 , batlevel);
    lcd.setCursor(19, 0);
    lcd.write(byte(1));
  }
  if (timh <= 70 && timh > 60)
  {
    byte batlevel[8] = {
      B01110,
      B10001,
      B10001,
      B11111,
      B11111,
      B11111,
      B11111,
      B11111,
    };
    lcd.createChar(1 , batlevel);
    lcd.setCursor(19, 0);
    lcd.write(byte(1));
  }
  if (timh <= 60 && timh > 40)
  {
    byte batlevel[8] =
    {
      B01110,
      B10001,
      B10001,
      B10001,
      B11111,
      B11111,
      B11111,
      B11111,
    };
    lcd.createChar(1 , batlevel);
    lcd.setCursor(19, 0);
    lcd.write(byte(1));
  }
  if (timh <= 40 && timh > 30)
  {
    byte batlevel[8] =
    {
      B01110,
      B10001,
      B10001,
      B10001,
      B10001,
      B11111,
      B11111,
      B11111,
    };
    lcd.createChar(1 , batlevel);
    lcd.setCursor(19, 0);
    lcd.write(byte(1));
  }
  if (timh <= 30 && timh > 10)
  {
    byte batlevel[8] = {
      B01110,
      B10001,
      B10001,
      B10001,
      B10001,
      B10001,
      B11111,
      B11111,
    };
   // digitalWrite(Control.GreenLed, HIGH); // Ανάβει το πράσινο led. 
   // digitalWrite(Control.RedLed, HIGH);  // Ανάβει το κόκκινο led.
    lcd.createChar(1 , batlevel);
    lcd.setCursor(19, 0);
    lcd.write(byte(1));
  }
  if (timh <= 10)
  {
    byte batlevel[8] = {
      B01110,
      B10001,
      B10001,
      B10001,
      B10001,
      B10001,
      B10001,
      B11111,
    };
 //   digitalWrite(Control.GreenLed, HIGH); // Ανάβει το πράσινο led. 
   // digitalWrite(Control.RedLed, HIGH);  // Ανάβει το κόκκινο led.
    lcd.createChar(1 , batlevel);
    lcd.setCursor(19, 0);
    lcd.write(byte(1));
  }
}


int JoyButton()
{ 
  static int JoyButton = 0;
  static int Return = 0;
  Control.Button = digitalRead(A1);

  if ( Control.Button == 0)
  {
   if(JoyButton == 0)
   {
    JoyButton = 1;
    if(Return == 0)
    {
      lcd.createChar(9 , Ret);
      lcd.setCursor(15, 2);
      lcd.write(byte(9));   
    }
    Return = 1;
  }
   else if(JoyButton == 1)
    {
     JoyButton = 0;
     lcd.setCursor(15, 2);  // Στην πρώτη σειρά στην θέση 16.
     lcd.print(" ");
     Return = 0;
    }    
  }
return JoyButton;
}



  
