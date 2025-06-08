#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h> 
#define DHT_PIN A0     
#define DHT_TYPE DHT11 

LiquidCrystal_I2C lcd(0x27, 16, 2);

DHT dht(DHT_PIN, DHT_TYPE);
float currentTemp = 0;         
float currentHumidity = 0;     
unsigned long lastDHTread = 0; 

// Button pins - users press these to control the AC
#define BUTTON_MODE 9  // Changes between OFF/MANUAL/AUTO modes
#define BUTTON_INC  10 // Makes target temperature higher
#define BUTTON_DEC  11 // Makes target temperature lower

#define BUZZER      8  

#define MOTOR_EN    6   
#define MOTOR_IN1   4   
#define MOTOR_IN2   5  

#define LED_GREEN   2   
#define LED_RED     3   

#define MOTION_SENSOR 7  

#define TRIG_PIN    12   
#define ECHO_PIN    13   

// Safety distances 
#define WARNING_DISTANCE 50  // Show warning when person is this close
#define DANGER_DISTANCE  20  // Stop motor when person is this close

// Fan speed limits
const int MIN_SPEED = 200;  
const int MAX_SPEED = 255;   


int currentMode = 0;         // 0=OFF, 1=MANUAL, 2=AUTO
int targetTemp = 20;         /
bool acRunning = false;      
bool motionDetected = false; 
bool tooClose = false;       //person too close to motor
bool ultrasonicActive = true; //distance sensor is working

// Time tracking variables (used to control when things happen)
unsigned long tempDisplayTime = 0;    
unsigned long lastMotionTime = 0;      
unsigned long noMotionTimeout = 3000;   // Turn off AC after 3 seconds of no motion just for the experience
unsigned long lastDistanceCheck = 0;   
unsigned long distanceDisplayTime = 0; 
unsigned long safetyResetTime = 0;     
unsigned long lastSafetyCheck = 0;     


void setup() {
  buzz(2); // sound when Arduino start


  pinMode(BUTTON_MODE, INPUT_PULLUP);
  pinMode(BUTTON_INC,  INPUT_PULLUP);
  pinMode(BUTTON_DEC,  INPUT_PULLUP);
  pinMode(MOTION_SENSOR, INPUT);  
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(BUZZER,    OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED,   OUTPUT);
  pinMode(MOTOR_EN,  OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);


  motorStop(); 
  digitalWrite(LED_GREEN, LOW);  
  digitalWrite(LED_RED,   HIGH); 

  dht.begin();

  lcd.init();
  lcd.backlight();
  updateDisplay();
  Serial.begin(9600);

  readDHTSensor();
}

void loop() {
  
  // Read temperature sensor every 2 seconds
  if (millis() - lastDHTread > 2000) {
    readDHTSensor();
    updateDisplay(); // for Show new temperature on screen
  }

  // for Check if people are too close
  if (millis() - lastDistanceCheck > 200 && currentMode != 0 && ultrasonicActive && acRunning) {
    checkDistance();
    lastDistanceCheck = millis();
  }
  
  // If someone was too close, check if they moved away
  if (tooClose && (millis() - lastSafetyCheck > 1000)) {
    float distance = getDistance();
    if (distance > DANGER_DISTANCE) {
      // Person moved away - it is safe now
      tooClose = false;
      ultrasonicActive = true;
      
      // Turn AC back on if it should be running
      if (currentMode == 1 || (currentMode == 2 && motionDetected)) {
        acRunning = true;
        updateACStatus();
      }
      
      updateDisplay();
      Serial.println("Safe distance detected - automatically reactivating system");
      buzz(1); 
    }
    lastSafetyCheck = millis();
  }
  
  // Stop showing distance warning after 3 seconds
  if (distanceDisplayTime != 0 && millis() - distanceDisplayTime > 3000 && !tooClose) {
    updateDisplay();
    distanceDisplayTime = 0;
  }

  // In AUTO mode, check if people are in the room
  if (currentMode == 2) {
    checkMotionSensor();
  }

  // Check if MODE button is pressed 
  if (digitalRead(BUTTON_MODE) == LOW) {
    if (currentMode == 2) buzz(3);          // Special sound when leaving AUTO mode
    delay(200);
    currentMode = (currentMode + 1) % 3;    // Change to next mode
    buzz(1);                                

    // Reset safety warnings when changing modes
    if (tooClose) {
      tooClose = false;
      ultrasonicActive = true;
    }

    // Handle what happens in each mode
    if (currentMode == 0) {
      // OFF mode - stop everything
      acRunning = false;
      ultrasonicActive = false;
      buzz(3);
    } else if (currentMode == 1) {
      // MANUAL mode motor runs all the time
      acRunning = true;
      ultrasonicActive = true;
      buzz(2);
    } else if (currentMode == 2) {
      // AUTO mode motor only runs when people are detected
      ultrasonicActive = true;
      checkMotionSensor(); // Check for people right away
    }

    updateACStatus(); 
    updateDisplay();  
    while (digitalRead(BUTTON_MODE) == LOW);
  }

  // Check if UP button is pressed makes target temperature higher
  if (digitalRead(BUTTON_INC) == LOW && targetTemp < 30 && currentMode != 0) {
    delay(200);
    targetTemp++; 
    targetTemp = constrain(targetTemp, 16, 30); 
    buzz(4); 
    showTempChange(); 
    while (digitalRead(BUTTON_INC) == LOW); 
  }
  

  if (digitalRead(BUTTON_DEC) == LOW && targetTemp > 16 && currentMode != 0) {
    delay(200);
    targetTemp--; 
    targetTemp = constrain(targetTemp, 16, 30); 
    buzz(4); 
    showTempChange(); // Show new temperature on screen
    while (digitalRead(BUTTON_DEC) == LOW); 
  }

  // Go back to normal screen after showing temperature change for 3 seconds
  if (tempDisplayTime != 0 && millis() - tempDisplayTime > 3000) {
    updateDisplay();
    tempDisplayTime = 0;
  }

  // Control fan speed based on how hot the room is
  static int prevSpeed = -1; 
  if (acRunning && !tooClose) {
    // Calculate how much hotter the room is than target temperature
    int tempDiff = max(0, currentTemp - targetTemp);
     
    
    int speed = map(tempDiff, 0, 10, MIN_SPEED, MAX_SPEED);
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);
    
    // Only change fan speed if it is different from before
    if (speed != prevSpeed) {
      digitalWrite(MOTOR_IN1, HIGH); 
      digitalWrite(MOTOR_IN2, LOW);
      analogWrite(MOTOR_EN, speed); 
      prevSpeed = speed;
      Serial.print("Motor speed: ");
      Serial.println(speed);
    }
  } else {
    // motor is not running so stop the fan
    if (prevSpeed != 0) {
      motorStop();
      prevSpeed = 0;
      Serial.println("Motor stopped");
    }
  }

  delay(50); 
}


void readDHTSensor() {
  currentHumidity = dht.readHumidity();
  currentTemp = dht.readTemperature();

  // Check if sensor is working properly
  if (isnan(currentHumidity) || isnan(currentTemp)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Humidity: ");
  Serial.print(currentHumidity);
  Serial.print("% Temperature: ");
  Serial.print(currentTemp);
  Serial.println("°C");
  
  lastDHTread = millis();
}

// Measure how far away objects are using ultrasonic sensor
float getDistance() {
  
  if (!ultrasonicActive) {
    return 999.0; 
  }
  
  // Send sound wave and measure how long it takes to come back
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate distance in centimeters
  float distance = duration * 0.0343 / 2;
  
  return distance;
}

// Check if someone is too close to motor and stop it if dangerous
void checkDistance() {
  float distance = getDistance();
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance < DANGER_DISTANCE && !tooClose) {
    tooClose = true;
    
    motorStop(); 
    
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SAFETY SHUTDOWN");
    lcd.setCursor(0, 1);
    lcd.print("MOVE AWAY!");
    
    tone(BUZZER, 2000, 1500);  // Make loud alarm sound
    
    
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);
    
    Serial.println("DANGER: Too close - safety shutdown activated");
    Serial.println("System will auto-restart when safe distance detected");
  }
  // WARNING: Person is getting close but not dangerous yet
  else if (distance < WARNING_DISTANCE && distance >= DANGER_DISTANCE && !tooClose && distanceDisplayTime == 0) {
    // Show warning on screen
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WARNING:");
    lcd.setCursor(0, 1);
    lcd.print("KEEP SAFE DIST!");
    
    tone(BUZZER, 1500, 300); // Make warning beep
    
    distanceDisplayTime = millis(); // Remember when we showed warning
    
    Serial.println("WARNING: Approaching minimum safe distance");
  }
}

// Check if people are in the room (only used in AUTO mode)
void checkMotionSensor() {
  int motionValue = digitalRead(MOTION_SENSOR);
  
  if (motionValue == HIGH) {
    // Motion detected - person is in room
    motionDetected = true;
    lastMotionTime = millis();
    
    // Turn on AC if it's not already running
    if (!acRunning && currentMode == 2 && !tooClose) {
      acRunning = true;
      ultrasonicActive = true;
      updateACStatus();
      buzz(2);
      Serial.println("Motion detected - turning AC ON");
    }
  } else {
    // No motion detected - check if we should turn off AC
    if (motionDetected && millis() - lastMotionTime > noMotionTimeout) {
      motionDetected = false;
      
      // Turn off AC after timeout period
      if (acRunning && currentMode == 2) {
        acRunning = false;
        updateACStatus();
        buzz(3);
        Serial.println("No motion for timeout period - turning AC OFF");
      }
    }
  }
}

// Make different beep sounds for different situations
void buzz(int type) {
  int dur = 100; 
  if      (type == 1) dur = 150; // Normal beep
  else if (type == 2) dur = 500; // Long beep (AC turning on)
  else if (type == 3) dur = 350; // Medium beep (AC turning off)
  else if (type == 4) dur =  50; // Short beep (temperature change)
  tone(BUZZER, 1000, dur);
}

// Show current information on LCD screen
void updateDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Mode: ");
       if (currentMode == 0) lcd.print("OFF");
  else if (currentMode == 1) lcd.print("MANUAL");
  else                    lcd.print("AUTO");
  
 
  lcd.setCursor(0, 1);
  if (tooClose) {
    lcd.print("SAFETY SHUTDOWN");
  } else {
    lcd.print("Tem:");
    lcd.print(currentTemp, 1);
    lcd.print("C Tg:");
    lcd.print(targetTemp);
    lcd.print("C");
  }
}


void showTempChange() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Target temp:");
  lcd.setCursor(0, 1);
  lcd.print("Tar:");
  lcd.print(targetTemp);
  lcd.print("C ");
  
  tempDisplayTime = millis(); 
}


void motorStop() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_EN, 0);
}

// Update LED lights to show if AC is on or off
void updateACStatus() {
  digitalWrite(LED_GREEN, acRunning ? HIGH : LOW); 
  digitalWrite(LED_RED,   acRunning ? LOW  : HIGH); 
  Serial.println(acRunning ? "AC ON":"AC OFF");
}
