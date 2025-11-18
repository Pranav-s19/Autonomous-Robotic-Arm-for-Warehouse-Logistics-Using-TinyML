#define LDR1_PIN 1      // LDR Sensor 1 (Analog)
#define LDR2_PIN 2      // LDR Sensor 2 (Analog)
#define DIG_IN1 3       // Digital Input 1
#define DIG_IN2 4       // Digital Input 2
#define PUSH_BUTTON 7   // Push Button (Toggle control pin)
#define CONTROL_PIN 44  // Toggle this pin based on button press
#define MANGO 6         // Output GPIO
#define APPLE 43        // Output GPIO
#define LED1 8          // LED for LDR1 condition
#define LED2 9          // LED for LDR2 condition
#define BUILTIN_LED 13  // ESP32 XIAO S3 built-in LED

bool buttonState = false;  // Track button state
bool lastButtonState = HIGH; // Store last state to detect button press

void setup() {
    Serial.begin(115200);
    
    pinMode(DIG_IN1, INPUT);
    pinMode(DIG_IN2, INPUT);
    pinMode(PUSH_BUTTON, INPUT_PULLUP); // Internal pull-up enabled
    pinMode(CONTROL_PIN, OUTPUT);
    pinMode(MANGO, OUTPUT);
    pinMode(APPLE, OUTPUT);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(BUILTIN_LED, OUTPUT);
}

void loop() {
    // Read LDR sensor values
    int ldr1 = analogRead(LDR1_PIN);
    int ldr2 = analogRead(LDR2_PIN);
    
    // Read digital inputs
    bool dig1 = digitalRead(DIG_IN1);
    bool dig2 = digitalRead(DIG_IN2);

    // Read push button state
    bool currentButtonState = digitalRead(PUSH_BUTTON);
    
    // Toggle CONTROL_PIN (GPIO 44) on button press
    if (currentButtonState == LOW && lastButtonState == HIGH) { 
        buttonState = !buttonState; // Toggle state
        digitalWrite(CONTROL_PIN, buttonState);
        delay(300); // Debounce delay
    }
    lastButtonState = currentButtonState;

    // Control APPLE (GPIO 43)
    if (dig1 == HIGH && ldr1 > 200) {
        digitalWrite(APPLE, LOW);
    } else {
        digitalWrite(APPLE, HIGH);
    }

    // Control MANGO (GPIO 6)
    if (dig2 == HIGH && ldr2 > 200) {
        digitalWrite(MANGO, LOW);
    } else {
        digitalWrite(MANGO, HIGH);
    }

    // LED on GPIO 8 (LDR1 condition)
    if (ldr1 < 150) {
        digitalWrite(LED1, HIGH);
    } else {
        digitalWrite(LED1, LOW);
    }

    // LED on GPIO 9 (LDR2 condition)
    if (ldr2 < 150) {
        digitalWrite(LED2, HIGH);
    } else {
        digitalWrite(LED2, LOW);
    }

    // Blink BUILTIN LED if DIG1 is HIGH and LDR1 > 150
    if (dig1 == HIGH && ldr1 < 150) {
        digitalWrite(BUILTIN_LED, HIGH);
        delay(200);
        digitalWrite(BUILTIN_LED, LOW);
        delay(200);
    }

    // Blink BUILTIN LED if DIG2 is HIGH and LDR2 > 150
    if (dig2 == HIGH && ldr2 < 150) {
        digitalWrite(BUILTIN_LED, HIGH);
        delay(200);
        digitalWrite(BUILTIN_LED, LOW);
        delay(200);
    }

    // Print values for debugging
    Serial.print("LDR1: "); Serial.print(ldr1);
    Serial.print(" | LDR2: "); Serial.print(ldr2);
    Serial.print(" | DIG1: "); Serial.print(dig1);
    Serial.print(" | DIG2: "); Serial.print(dig2);
    Serial.print(" | Button: "); Serial.print(buttonState);
    Serial.print(" | Mango: "); Serial.print(digitalRead(MANGO));
    Serial.print(" | Apple: "); Serial.print(digitalRead(APPLE));
    Serial.print(" | LED1: "); Serial.print(digitalRead(LED1));
    Serial.print(" | LED2: "); Serial.print(digitalRead(LED2));
    Serial.println();

    delay(150);  // Small delay to stabilize readings
}
