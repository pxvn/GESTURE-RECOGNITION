
#include <SoftI2C.h>
#include <LiquidCrystal_I2C.h>

// Define custom I2C pins
#define SDA_PIN 8  // Custom SDA pin
#define SCL_PIN 9   // Custom SCL pin

// Create SoftI2C instance
SoftI2C SoftWire = SoftI2C(SDA_PIN, SCL_PIN);

// LCD I2C address and dimensions
#define LCD_I2C_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

// Commands for LCD
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// Flags for display/cursor settings
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// Flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

// Control bits
#define En 0x04  // Enable bit
#define Rw 0x02  // Read/Write bit
#define Rs 0x01  // Register select bit

// LCD variables
uint8_t _displayfunction;
uint8_t _displaycontrol;
uint8_t _displaymode;
uint8_t _backlightval = LCD_BACKLIGHT;

// Send command to LCD via I2C
void lcd_command(uint8_t cmd) {
  uint8_t highnib = cmd & 0xF0;
  uint8_t lownib = (cmd << 4) & 0xF0;
  
  // Send high nibble
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(highnib | _backlightval);
  SoftWire.write(highnib | En | _backlightval);  // En high
  SoftWire.endTransmission();
  delayMicroseconds(1);
  
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(highnib | _backlightval);       // En low
  SoftWire.endTransmission();
  delayMicroseconds(50);
  
  // Send low nibble
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(lownib | _backlightval);
  SoftWire.write(lownib | En | _backlightval);   // En high
  SoftWire.endTransmission();
  delayMicroseconds(1);
  
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(lownib | _backlightval);        // En low
  SoftWire.endTransmission();
  delayMicroseconds(50);
}

// Send data to LCD via I2C
void lcd_write(uint8_t data) {
  uint8_t highnib = data & 0xF0;
  uint8_t lownib = (data << 4) & 0xF0;
  
  // Send high nibble
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(highnib | Rs | _backlightval);
  SoftWire.write(highnib | Rs | En | _backlightval);  // En high
  SoftWire.endTransmission();
  delayMicroseconds(1);
  
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(highnib | Rs | _backlightval);       // En low
  SoftWire.endTransmission();
  delayMicroseconds(50);
  
  // Send low nibble
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(lownib | Rs | _backlightval);
  SoftWire.write(lownib | Rs | En | _backlightval);   // En high
  SoftWire.endTransmission();
  delayMicroseconds(1);
  
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(lownib | Rs | _backlightval);        // En low
  SoftWire.endTransmission();
  delayMicroseconds(50);
}

// Initialize LCD
void lcd_init() {
  // Initialize SoftI2C
  SoftWire.begin();
  delay(50);  // Wait for LCD to power up
  
  // Reset the display
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(_backlightval);
  SoftWire.endTransmission();
  delay(1000);
  
  // Put the LCD into 4 bit mode
  // This is according to the Hitachi HD44780 datasheet
  // Figure 24, pg 46
  
  // We start in 8bit mode, try to set 4 bit mode
  // High nibble only
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(0x30 | _backlightval);
  SoftWire.write(0x30 | En | _backlightval);  // En high
  SoftWire.endTransmission();
  delayMicroseconds(1);
  
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(0x30 | _backlightval);       // En low
  SoftWire.endTransmission();
  delayMicroseconds(4500);
  
  // Second try
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(0x30 | _backlightval);
  SoftWire.write(0x30 | En | _backlightval);  // En high
  SoftWire.endTransmission();
  delayMicroseconds(1);
  
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(0x30 | _backlightval);       // En low
  SoftWire.endTransmission();
  delayMicroseconds(4500);
  
  // Third try
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(0x30 | _backlightval);
  SoftWire.write(0x30 | En | _backlightval);  // En high
  SoftWire.endTransmission();
  delayMicroseconds(1);
  
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(0x30 | _backlightval);       // En low
  SoftWire.endTransmission();
  delayMicroseconds(150);
  
  // Finally, set to 4-bit interface
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(0x20 | _backlightval);
  SoftWire.write(0x20 | En | _backlightval);  // En high
  SoftWire.endTransmission();
  delayMicroseconds(1);
  
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(0x20 | _backlightval);       // En low
  SoftWire.endTransmission();
  delayMicroseconds(50);
  
  // Set # lines, font size, etc.
  _displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
  lcd_command(LCD_FUNCTIONSET | _displayfunction);
  
  // Turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
  
  // Clear it off
  lcd_command(LCD_CLEARDISPLAY);
  delayMicroseconds(2000);  // This command takes a long time!
  
  // Initialize to default text direction
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  lcd_command(LCD_ENTRYMODESET | _displaymode);
}

// Clear the LCD display
void lcd_clear() {
  lcd_command(LCD_CLEARDISPLAY);
  delayMicroseconds(2000);  // This command takes a long time!
}

// Set cursor position
void lcd_setCursor(uint8_t col, uint8_t row) {
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if (row >= LCD_ROWS) {
    row = LCD_ROWS - 1;
  }
  lcd_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Print a string to the LCD
void lcd_print(const char* str) {
  while (*str) {
    lcd_write(*str++);
  }
}

// Turn on backlight
void lcd_backlight() {
  _backlightval = LCD_BACKLIGHT;
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(_backlightval);
  SoftWire.endTransmission();
}

// Turn off backlight
void lcd_noBacklight() {
  _backlightval = LCD_NOBACKLIGHT;
  SoftWire.beginTransmission(LCD_I2C_ADDR);
  SoftWire.write(_backlightval);
  SoftWire.endTransmission();
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  while (!Serial);
  Serial.println("LCD Test with Custom I2C Pins");
  
  // Initialize the LCD
  lcd_init();
  
  // Turn on the backlight
  lcd_backlight();
  
  // Print welcome message
  lcd_setCursor(3, 0);
  lcd_print("Hello, world!");
  lcd_setCursor(2, 1);
  lcd_print("Custom I2C Pins!");
  
  Serial.println("LCD initialized");
}

void loop() {
  // Your loop code here
}
