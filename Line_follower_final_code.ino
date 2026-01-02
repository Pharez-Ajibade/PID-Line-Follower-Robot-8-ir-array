//  Motor A connections--------------------------------------------------------- left motor
const int Motor_A_pwm = 19;
const int Ain2 = 18;
const int Ain1 = 5;

// Motor B connections-----------------------------------------------------------
const int Motor_B_pwm = 17;
const int Bin1 = 16;
const int Bin2 = 4;

// PWM SETUP-----------------------------------------------------------------------
const int PWM_CHANNEL_0 = 0;     // ESP32 has 16 channels which can generate 16 independent waveforms - high speed from 8 - 15
const int PWM_CHANNEL_1 = 1;     // ESP32 has 16 channels which can generate 16 independent waveforms - high speed from 8 - 15

const int PWM_FREQ = 490;    // in Hz - Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz i
const int PWM_RESOLUTION = 8;  // We'll use same resolution as Uno (8 bits, 0-255) but ESP32 can go up to 16 bits

// The max duty cycle value based on PWM resolution (will be 255 if resolution is 8 bits)
const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION) - 1);


// analog input for 8 ir array 

int DS1 = 36; // left most 
int DS2 = 39;
int DS3 = 34;
int DS4 = 35;
int DS5 = 32;
int DS6 = 33;
int DS7 = 25;
int DS8 = 26; /// rightmost 

// 8 ir array sensor val
 int S1 = 0 ;
 int S2 = 0 ;
 int S3 = 0 ;
 int S4 = 0 ;
 int S5 = 0 ;
 int S6 = 0 ;
 int S7 = 0 ;
 int S8 = 0 ;

 // 8 IR ARRAY SUM VARIABLES
 int  S1_sum = 0; 
 int  S2_sum = 0; 
 int  S3_sum = 0; 
 int  S4_sum = 0; 
 int  S5_sum = 0; 
 int  S6_sum = 0; 
 int  S7_sum = 0; 
 int  S8_sum = 0; 

 // 8 IR AVERAGE VARIABLES
 int S1_avg [2]; // [ 0 is max average , 1 is min average ]
 int S2_avg [2];
 int S3_avg [2];
 int S4_avg [2];
 int S5_avg [2];
 int S6_avg [2];
 int S7_avg [2];
 int S8_avg [2];
 

// DECLARING WEIGHTS

int weights[8] = {-40, -30, -20, -10, 10, 20, 30, 40};
 // midpoint and desired setpoint
double midpoint ;
double setpoint ;
double stoppoint;

// Samplin rate timer
 unsigned long Sampling_rate = 50; // milliseconds = 4 milliseconds = 0.004 seconds = 250hz 
 double Sampling_rate_seconds = Sampling_rate / 1000.0;
 unsigned long Current_timer ;
 unsigned long Current_timer2 ;
 unsigned long Start_timer ;
 unsigned long Start_timer2 ;
 unsigned long Stop_duration = 2000 ;

//------------------------------PID VALUES ----------------------
double KP = 15.5; // intrgral constant - TUNE THI

double Ki =  0.0100; // intrgral constant - TUNE THIS
double Integral;

double Kd = 0.5; // Derivative constant- TUNE THIS
double Derivative ;

double error ;
double error_previous = 0;
double PID_output ;

//------------------------------PID FUNCTION ----------------------
void PID(){
error = setpoint - midpoint ;
Integral  = Integral + (error * Sampling_rate_seconds);
Derivative = (error - error_previous) / Sampling_rate_seconds ;

PID_output = (KP  * error) + (Ki * Integral) + (Kd * Derivative);

error_previous = error ; /// save previous error time

}

void setup()
{
 
 Start_timer = millis();
 Start_timer2 = millis();

 Serial.begin(115200);

   // Set all the motor control pins to outputs
  pinMode(Ain1, OUTPUT);
  pinMode(Ain2, OUTPUT);
  pinMode(Bin1, OUTPUT);
  pinMode(Bin2, OUTPUT);
  
  ledcAttachChannel(Motor_A_pwm, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_0 );
  ledcAttachChannel(Motor_B_pwm, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_1 );

  // Turn off motors - Initial state
  digitalWrite(Ain1, LOW);
  digitalWrite(Ain2, LOW);
  digitalWrite(Bin1, LOW);
  digitalWrite(Bin2, LOW);

  pinMode(DS1,INPUT);
  pinMode(DS2,INPUT);
  pinMode(DS3,INPUT);
  pinMode(DS4,INPUT);
  pinMode(DS5,INPUT);
  pinMode(DS6,INPUT);
  pinMode(DS7,INPUT);
  pinMode(DS8,INPUT);

 
 // place the robot on the black calibration section  red light comes on 

  int R_Led_pin = 23;
  int Y_Led_pin = 21;
  int G_Led_pin = 22;
  int Button_pin = 15;

  pinMode( R_Led_pin , OUTPUT);  
  pinMode( Y_Led_pin , OUTPUT);
  pinMode( G_Led_pin , OUTPUT);
  pinMode( Button_pin, INPUT_PULLUP);

  digitalWrite(R_Led_pin, HIGH);
  digitalWrite(Y_Led_pin, LOW); 
  digitalWrite(G_Led_pin, LOW);

  Serial.println("RED LIGHT ON, PRESS BUTTON TO START CALIBRATION");
  
  int button_state = 1;
  
  // if button is pushed Red light goes off, YELLOW COMES ON and then calibartion starts

  while( button_state == 1 ) {

   button_state = digitalRead(Button_pin);
   
   delay (300);
   
   button_state = digitalRead(Button_pin);

   }

  digitalWrite(R_Led_pin, LOW);
  digitalWrite(Y_Led_pin, HIGH); 
  digitalWrite(G_Led_pin, LOW);



 // place the robot on the black calibration section  red light comes on takes reading 1000 average for seconds store as max 
    int avg_read_no = 1000 ;
    for ( int i = 0; i < avg_read_no ; i++){
    
     S1 = analogRead(DS1);
     S2 = analogRead(DS2);
     S3 = analogRead(DS3);
     S4 = analogRead(DS4);
     S5 = analogRead(DS5);
     S6 = analogRead(DS6);
     S7 = analogRead(DS7);
     S8 = analogRead(DS8);

     S1_sum += S1 ; 
     S2_sum += S2 ;
     S3_sum += S3 ;
     S4_sum += S4 ;
     S5_sum += S5 ;
     S6_sum += S6 ;
     S7_sum += S7 ;
     S8_sum += S8 ;
    
    // commmment out 
    Serial.print(" S1: "); Serial.print(S1); Serial.print(" S2: "); Serial.print(S2); Serial.print(" S3: "); Serial.print(S3);  Serial.print(" S4: "); Serial.print(S4); Serial.print(" S5: "); Serial.print(S5); Serial.print(" S6: "); Serial.print(S6); Serial.print(" S7: "); Serial.print(S7); Serial.print(" S8: "); Serial.print(S8); Serial.println(" ");
    
    }

  S1_avg [0] = S1_sum / avg_read_no ; 
  S2_avg [0] = S2_sum / avg_read_no ;
  S3_avg [0] = S3_sum / avg_read_no ;
  S4_avg [0] = S4_sum / avg_read_no ;
  S5_avg [0] = S5_sum / avg_read_no ;
  S6_avg [0] = S6_sum / avg_read_no ;
  S7_avg [0] = S7_sum / avg_read_no ;
  S8_avg [0] = S8_sum / avg_read_no ;

 // Red light comes on once the scanning is done and ot ortates to scan the other part

  digitalWrite(R_Led_pin, HIGH);
  digitalWrite(Y_Led_pin, LOW); 
  digitalWrite(G_Led_pin, LOW);


 // robot turns righ to white section turn on led
   Serial.println(" Turning to white side for calibration");

   ledcWriteChannel(PWM_CHANNEL_0, 150);
   ledcWriteChannel(PWM_CHANNEL_1, 150);

   // SPIN RIGHT 
   digitalWrite(Ain1, HIGH);
   digitalWrite(Ain2, LOW);
   digitalWrite(Bin1, LOW);
   digitalWrite(Bin2, HIGH);
   Serial.print("SPINNING TO WHITE SIDE "); 

   delay(500);

   // STOP
   digitalWrite(Ain1, LOW);
   digitalWrite(Ain2, LOW);
   digitalWrite(Bin1, LOW);
   digitalWrite(Bin2, LOW);
   Serial.print("STOP MOVING "); 
 
 // red light goes off and scanning of white section begins \  digitalWrite(R_Led_pin, HIGH);
  digitalWrite(R_Led_pin, LOW);
  digitalWrite(Y_Led_pin, HIGH); 
  digitalWrite(G_Led_pin, LOW);

// RESET SUM
S1_sum = S2_sum = S3_sum = S4_sum = S5_sum = S6_sum = S7_sum = S8_sum = 0; 

 // rtakes reading 1000 average readings store as min 
  for ( int i = 0; i < avg_read_no ; i++){
    
  S1 = analogRead(DS1);
  S2 = analogRead(DS2);
  S3 = analogRead(DS3);
  S4 = analogRead(DS4);
  S5 = analogRead(DS5);
  S6 = analogRead(DS6);
  S7 = analogRead(DS7);
  S8 = analogRead(DS8);

  S1_sum += S1 ; 
  S2_sum += S2 ;
  S3_sum += S3 ;
  S4_sum += S4 ;
  S5_sum += S5 ;
  S6_sum += S6 ;
  S7_sum += S7 ;
  S8_sum += S8 ;

  // comment out 
  Serial.print(" S1: "); Serial.print(S1); Serial.print(" S2: "); Serial.print(S2); Serial.print(" S3: "); Serial.print(S3);  Serial.print(" S4: "); Serial.print(S4); Serial.print(" S5: "); Serial.print(S5); Serial.print(" S6: "); Serial.print(S6); Serial.print(" S7: "); Serial.print(S7); Serial.print(" S8: "); Serial.print(S8); Serial.println(" ");

  }

  S1_avg [1] = S1_sum / avg_read_no ; 
  S2_avg [1] = S2_sum / avg_read_no ;
  S3_avg [1] = S3_sum / avg_read_no ;
  S4_avg [1] = S4_sum / avg_read_no ;
  S5_avg [1] = S5_sum / avg_read_no ;
  S6_avg [1] = S6_sum / avg_read_no ;
  S7_avg [1] = S7_sum / avg_read_no ;
  S8_avg [1] = S8_sum / avg_read_no ;

 /// determins midpoint - s4 and 5 are on black  via // Line Position = Σ(SensorValue × Weight) / Σ(SensorValue) using S1 to S3 on WHITE, S4 and S5 on black, and S6 to S8 on white

midpoint = (
  (weights[0] * S1_avg[1]) +
  (weights[1] * S2_avg[1]) +
  (weights[2] * S3_avg[1]) +
  (weights[3] * S4_avg[0]) +
  (weights[4] * S5_avg[0]) +
  (weights[5] * S6_avg[1]) +
  (weights[6] * S7_avg[1]) +
  (weights[7] * S8_avg[1])
) / double( S1_avg[1] + S2_avg[1] + S3_avg[1] + S4_avg[0] + S5_avg[0] + S6_avg[1] + S7_avg[1] + S8_avg[1] );

setpoint = midpoint;

Serial.print("Calibrated Setpoint [-40 to 40 scale]: ");
Serial.println(setpoint);


 // red AND GREEN  light comes on scanning calibration done 
  digitalWrite(R_Led_pin, HIGH);
  digitalWrite(Y_Led_pin, LOW); 
  digitalWrite(G_Led_pin, HIGH);

 // IF BUTTON IS PRESSED START LINE FOLLLOWINF PID ALGORITHM 
  
  button_state = 1 ;

  while( button_state == 1 ) {

   digitalWrite(G_Led_pin, HIGH);

   button_state = digitalRead(Button_pin);
   
   delay (300);
   
   button_state = digitalRead(Button_pin);

   digitalWrite(G_Led_pin, LOW);

  }

  digitalWrite(R_Led_pin, LOW);
  digitalWrite(Y_Led_pin, LOW); 
  digitalWrite(G_Led_pin, HIGH);

  delay(2500);

}




void loop()
{
  getmidpoint();

  // PWM SPEEED DECLARATION 
   int dutyCycle_L ;
   int dutyCycle_R ;

  
  Current_timer = millis(); 

  if ( Current_timer - Start_timer >= Sampling_rate ){
   
   PID();

 //   Serial.print("Error ;") ;   Serial.println (error ) ;
 //   Serial.print("PID OUTPUT: "); Serial.println(PID_output);
   
  Start_timer = Current_timer ;
  }
 
  int base_speed_pwm = 65 ;

  int steering_val = round (PID_output) ;

  int steering_val_max  =  255 - base_speed_pwm ;
  
  steering_val = constrain (steering_val, -steering_val_max, steering_val_max);
   
   dutyCycle_L = base_speed_pwm - steering_val;
   dutyCycle_R = base_speed_pwm + steering_val;

   dutyCycle_L = constrain(dutyCycle_L , 0, 255);
   dutyCycle_R = constrain(dutyCycle_R , 0, 255);

   
 //   Serial.print("Moving forward at "); Serial.print(" L PWM ");  Serial.print (dutyCycle_L); Serial.print(" R PWM ");  Serial.println(dutyCycle_R); 

// lse{ dutyCycle_L = 0; dutyCycle_R = 0; }

   ledcWriteChannel(PWM_CHANNEL_0, dutyCycle_L);
   ledcWriteChannel(PWM_CHANNEL_1, dutyCycle_R);

   // FORWARD 
   digitalWrite(Ain1, HIGH);
   digitalWrite(Ain2, LOW);
   digitalWrite(Bin1, HIGH);
   digitalWrite(Bin2, LOW);

   // ALL BLACK STOP 

   // BUTTON TO STOP 

}

void getmidpoint(){
  
 // get sensor readings

  S1 = analogRead(DS1);
  S2 = analogRead(DS2);
  S3 = analogRead(DS3);
  S4 = analogRead(DS4);
  S5 = analogRead(DS5);
  S6 = analogRead(DS6);
  S7 = analogRead(DS7);
  S8 = analogRead(DS8);

  // comment out 

//  Serial.print(" S1: "); Serial.print(S1); Serial.print(" S2: "); Serial.print(S2); Serial.print(" S3: "); Serial.print(S3);  Serial.print(" S4: "); Serial.print(S4); Serial.print(" S5: "); Serial.print(S5); Serial.print(" S6: "); Serial.print(S6); Serial.print(" S7: "); Serial.print(S7); Serial.print(" S8: "); Serial.print(S8); Serial.println(" ");
  
midpoint = (
   (weights[0] * S1) +
   (weights[1] * S2) +
   (weights[2] * S3) +
   (weights[3] * S4) +
   (weights[4] * S5) +
   (weights[5] * S6) +
   (weights[6] * S7) +
   (weights[7] * S8)
) / double( S1 + S2 + S3 + S4 + S5 + S6 + S7 + S8 );
  
//  Serial.print("currrent position on midpoint scale -40 to 0 to 4 ;") ;   Serial.println (midpoint) ; 

}
