
#define LDR1 2
#define LDR2 3

#define DATA 9   // DS
#define LATCH 8  // ST_CP
#define CLOCK 7  // SH_CP

#define DIGIT_4 10
#define DIGIT_3 11
#define DIGIT_2 12
#define DIGIT_1 13

#define GREEN_LED1 5
#define RED_LED1 6
#define GREEN_LED2 7
#define RED_LED2 4

// 7-Seg Display Variables
unsigned char gTable[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x00 };


float gstartTime = 0;
float Time = 0;
unsigned long gSpeed = 0;

volatile bool gLDR1_ISR_Flag = 0;
volatile bool gLDR2_ISR_Flag = 0;

bool startFlag = 0;
double gDist = 1.5;

volatile bool gbutton1_ISR_Flag = 0;
volatile bool gbutton2_ISR_Flag = 0;

byte gCurrentDigit;

unsigned int gReloadTimer1 = 62500;
byte gReloadTimer2 = 10;  


void setup() {

  Serial.begin(9600);
  // put your setup code here, to run once
  pinMode(LDR1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LDR1), LDR1_ISR, FALLING);

  pinMode(LDR2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LDR2), LDR2_ISR, FALLING);

  pinMode(GREEN_LED1, OUTPUT);
  pinMode(RED_LED1, OUTPUT);
  pinMode(GREEN_LED2, OUTPUT);
  pinMode(RED_LED2, OUTPUT);

  pinMode(DIGIT_1, OUTPUT);
  pinMode(DIGIT_2, OUTPUT);
  pinMode(DIGIT_3, OUTPUT);
  pinMode(DIGIT_4, OUTPUT);

  // Shift Register Pins
  pinMode(LATCH, OUTPUT);
  pinMode(CLOCK, OUTPUT);
  pinMode(DATA, OUTPUT);


  noInterrupts();  // disable all interrupts
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  OCR2A = 10;            // compare match register 16MHz/256
  TCCR2A |= (1 << WGM22);   // CTC mode
  TCCR2B = (1 << CS22)| (1<<CS21) | (1<<CS20);    // 256 prescaler
  TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt


  TCCR1B = 0x00; // clock frequency / 1024
  //OCR2B = 0x00;  // Output compare
  TCNT1= 0;     // Set counter 2 to zero
  TIMSK1 = 0x01; // Enable overflow interrupt
  interrupts();
}



void startCheck() {
  while (digitalRead(LDR1) == HIGH && digitalRead(LDR2) == HIGH) {
    if (digitalRead(LDR1) == LOW) {
      digitalWrite(GREEN_LED1, HIGH);
    } else {
      digitalWrite(GREEN_LED1, LOW);
    }
    if (digitalRead(LDR2) == LOW) {
      digitalWrite(GREEN_LED2, HIGH);
    } else {
      digitalWrite(GREEN_LED2, LOW);
    }
  } 
}

//start button
void button1ISR() {
  // Set ISR Flag
  gbutton1_ISR_Flag = 1;
  if(digitalRead(LDR1) == HIGH && digitalRead(LDR2) == HIGH){
    startFlag = 1;
  }
  
}

//reset button (clears interupt flags and resets the)
void button2ISR() {
  // Set ISR Flag
  gbutton2_ISR_Flag = 1;

  gSpeed = 0;  

  if (gLDR1_ISR_Flag == 1) {
    gLDR1_ISR_Flag = 0;
  }
  if (gLDR2_ISR_Flag == 1) {
    gLDR2_ISR_Flag = 0;
  }
}

ISR(TIMER1_OVF_vect) {}

//ISR that switches through the digits for the 7 seg display
ISR(TIMER2_COMPA_vect)  // Timer2 interrupt service routine (ISR)
{
  dispOff();  // turn off the display

  switch (gCurrentDigit) {
    case 1:                                     //0x:xx
      display(((gSpeed / 1000)) % 10, 0);  // prepare to display digit 1 (most left)
      digitalWrite(DIGIT_1, LOW);               // turn on digit 1
      break;

    case 2:                               //x0:xx
      display((gSpeed / 100) % 10, 0);  // prepare to display digit 2
      digitalWrite(DIGIT_2, LOW);         // turn on digit 2
      break;

    case 3:                           //xx:0x
      display((gSpeed / 10) % 10, 1);  // prepare to display digit 3
      digitalWrite(DIGIT_3, LOW);     // turn on digit 3
      break;

    case 4:                        //xx:x0
      display(gSpeed % 10, 0);     // prepare to display digit 4 (most right)
      digitalWrite(DIGIT_4, LOW);  // turn on digit 4
      break;

    default:
      break;
  }

  gCurrentDigit = (gCurrentDigit % 4) + 1;
}

//Displays numbers on the 7 segmant display
void display(unsigned char num, unsigned char dp) {
  digitalWrite(LATCH, LOW);
  shiftOut(DATA, CLOCK, MSBFIRST, gTable[num] | (dp << 7));
  digitalWrite(LATCH, HIGH);
}

//turns the display off
void dispOff() {
  digitalWrite(DIGIT_1, HIGH);
  digitalWrite(DIGIT_2, HIGH);
  digitalWrite(DIGIT_3, HIGH);
  digitalWrite(DIGIT_4, HIGH);
}

//ISR for the first LDR
void LDR1_ISR() {
  // Serial.println("LDR1");
  // if (gLDR1_ISR_Flag == 0) {
  //   gLDR1_ISR_Flag = 1;
  gLDR1_ISR_Flag = 1;

    gstartTime = micros();
  }


//ISR for the second LDR 
void LDR2_ISR() {
  // Serial.println("LDR2");
  gLDR2_ISR_Flag = 0;
  if(gLDR1_ISR_Flag == 1){
    Time = micros() - gstartTime;
   gSpeed = (gDist * .000157828) / (Time * 0.00000000027778);
   Serial.print(gSpeed);
   Serial.println(" MPH");
  }
  delay(3000);
  gSpeed = 0;
  Time = 0;
  gstartTime = 0;
}

void loop() {
 //clears ISR flags


  if (gbutton1_ISR_Flag == 1) {
    gbutton1_ISR_Flag = 0;
  }
  if (gbutton2_ISR_Flag == 1) {
    gbutton2_ISR_Flag = 0;
  }
  
  //waits for sensors to be alligned and the start button to be pressed
  // while(startFlag == 0){
  //   startCheck();
  // }  
}

