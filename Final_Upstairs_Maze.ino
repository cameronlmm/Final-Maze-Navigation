#include <MPU6050_tockn.h>
#include <Keypad.h>
#include <LiquidCrystal.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

const int rs = 14, en = 27, d4 = 26, d5 = 25, d6 = 33, d7 = 32; // LCD pins
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//3x4 Matrix key pad
const byte ROWS = 4; // Four rows
const byte COLS = 3; // three columns

// Define the Keymap
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

byte rowPins[ROWS] = {12, 19, 18, 5};
byte colPins[COLS] = {3, 16, 4};

Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

const int slaveAddress = 8; // I2C slave address

int yaw; // Declare yaw globally

void setup()  
{
    Wire.begin();
    Serial.begin(9600);
    lcd.begin(16, 2);  
    scrollingText("Welcome to EEEBot Maze Navigation !", 0, 500);
    displayOnLCD("    Group 14");
    //lcd.clear();
    //lcd.setCursor(0,1);
    //lcd.print("    Group 14"); // Try and display this for 1 second then display EEEBot calibrating
    delay(2000); // Display for 2 seconds
    scrollingText("Beginning Sensor Calibration", 0, 400);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Do Not Move");
    lcd.setCursor(0,1);
    lcd.print("EEEBot");
    mpu6050.begin(); // Initialize the MPU6050
    mpu6050.calcGyroOffsets(true); // Autocalibration: Make sure the sensor is stable and not moving before running this
    countdown(5);

}

void loop() {
  mpu6050.update(); // Update sensor data
  yaw = mpu6050.getAngleX(); // Get the yaw angle (Angle on X-axis of gyroscope) and store it in 'yaw'
  //Serial.println(yaw);

  char key = kpd.getKey();
  if (!key) {
    key = 0xFF; // if no key is pressed, use/send a placeholder
  }

  Wire.beginTransmission(slaveAddress); // Transmit to downstairs esp32
  Wire.write((yaw >> 8) & 0xFF); // Isolates and sends High byte of yaw
  Wire.write(yaw & 0xFF);       // Isolates and send Low byte of yaw
  Wire.write(key);                // Sends either key pressed or placeholder
  Wire.endTransmission();

  int requestedByte = Wire.requestFrom(8, 1); // request 1 byte of data from downstairs esp32 (slave address is 8) and store in variable
  //Serial.printf("Turn Status: %u\n", requestedByte);

  if (requestedByte == 1) // If one byte of data has been received from the downstairs esp32
  {
    int currentActivity = Wire.read(); // read requested data
    //Serial.print("Turn Status: "); Serial.println(currentActivity);

    switch (currentActivity) {
    case 0: displayOnLCD("Ending Maze."); lcd.clear(); break;
    case 1: displayOnLCD("Going 10cm"); break;
    case 2: displayOnLCD("Going 20cm"); break;
    case 3: displayOnLCD("Going 30cm"); break;
    case 4: displayOnLCD("Going 40cm"); break;
    case 5: displayOnLCD("Going 50cm"); break;
    case 6: displayOnLCD("Going 60cm"); break;
    case 7: displayOnLCD("Going 70cm"); break;
    case 8: displayOnLCD("Going 80cm"); break;
    case 9: displayOnLCD("Going 90cm"); break;
    case 10: displayOnLCD("Turning Left"); break;
    case 11: displayOnLCD("Turning Right"); break;
    case 13: reinitializeSensor(); break;
    //case 14: displayOnLCD("Key Pressed: "); lcd.print(key); break;
    //default: displayOnLCD("Error."); break;
    }
  }

  if (key != 0xFF) 
  {
     Serial.println(key);
     displayOnLCD("Key Pressed: ");
     //lcd.clear();
     //lcd.setCursor(0, 1);
     //lcd.print("Key Pressed: ");
     lcd.print(key);
  }
  delay(100);
}


void reinitializeSensor() {
    //Serial.println("Reinitializing MPU6050...");
    mpu6050.begin(); // Reinitialise the MPU6050
    mpu6050.calcGyroOffsets(true); // Recalibrate MPU6050 - Ideally this function would be called with the reinitialisation of the sensor, but this would sacrifice time and would require a longer delay
    //mpu6050.update(); // Update sensor data
    //yaw = mpu6050.getAngleX(); // Get the yaw angle
    //Serial.println("Reinitialization complete.");
}

void scrollingText(const String &message, int row, unsigned int scrollSpeed) 
{
  int messageLength = message.length();
  
  // Scroll the message across the LCD from right to left
  for (int position = 0; position < messageLength + 16; position++) 
  {
    lcd.clear();  // Clear the LCD to refresh display
    
    // Display next part of message
    for (int i = 0; i < 16; i++) 
    {
      // Calculate position of characters to display
      int charIndex = position + i - 16;
      
      // Display characters if they are within bounds of the message
      if (charIndex >= 0 && charIndex < messageLength) 
      {
        lcd.setCursor(i, row);
        lcd.print(message[charIndex]);
      }
    }
    // Delay to control the speed of the scroll
    delay(scrollSpeed);
  }
}

void countdown(int startFrom) 
{
  //lcd.clear();  // Clear LCD before starting countdown
  lcd.setCursor(0, 1); // Set the cursor
  for (int i = startFrom; i >= 0; i--) 
  {
    lcd.setCursor(0, 1); // Reset cursor position for each number
    lcd.print("Starting in: "); // Display countdown text
    lcd.print(i); // Display current number
    delay(1000); // Wait one second before updating display
  }
  lcd.clear();
  lcd.setCursor(0, 0);
}

void displayOnLCD(const String &message) 
{
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(message);
}