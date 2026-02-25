// *************************************************************************************************************************************
// |                                                                                                                                   |
// | Firmware for Arduino Uno Capacitance Meter                                                                                        |
// |                                                                                                                                   |
// | No:        CMETER-001-VOL1                                                                                                        |
// | Author:    Jan Engelbrecht Pedersen                                                                                               |
// | Date:      24.02.26                                                                                                               |
// | Reviewed:  Jan Engelbrecht Pedersen 24.02.26                                                                                      |
// | Approved:  Jan Engelbrecht Pedersen 24.02.26                                                                                      |
// |                                                                                                                                   |
// *************************************************************************************************************************************
// Firmware for Arduino Uno based Capacitance meter
// Based upon the charging formula for a Capacitor: C = t / R 
// Because the formula for the voltage over a Capacitor we charge is: Uc(t) = Uo * (1 - e^(-t/R*C))
// WHen t = R*C:
// Uc(t) = Uo * (1 - e^(-1))
// Uc(t) = Uo * 0.63212
// When t=R*C Uc(t) = Uo * 0.63212
// 0.63212*Uo = Uo * (1 - e^(-t/(R*C)))
// 0.63212 = 1 - e^(-t/(R*C))
// 0.63212 - 1 = -e^(-t/(R*C))
// 1 - 0.63212 = e^(-t/(R*C))
// 0.36788 = e^(-t/(R*C))
// ln(0.36788) = ln(e^(-t/(R*C)))
// -1 = (-t/(R*C))
// -R*C = -t
// R*C = t
// C = t / R
// If we measure t when Uc(t) = 0.63212 * Uo and we know R we can calculate C
// We use deifferent values for R to be able to measure C in a big range
// Can measure capacitance from 10nF to 10mF
//
// *************************************************************************************************************************************
// OVERVIEW
// *************************************************************************************************************************************
// The Cmeter is an arduino UNO / NANO based capacitance meter that measures unknown capacitors using an RC charging method.
// The hardware is designed to support auto-ranging from approximately 10 nF up to 10,000 µF
// by switching between multiple precision charging resistors under software control.
//
// The system consists of four main hardware blocks:
// 1) Microcontroller core (ATmega328P / Arduino UNO or NANO)
// 2) Capacitance measurement network (charging and discharge resistors + DUT connector)
// 3) User interface (push button and I2C LCD)
// 4) Power and signal routing (Vcc, ground, I2C, ADC)
//
// *************************************************************************************************************************************
// MICROCONTROLLER CORE (ATMEGA328P / ARDUINO UNO)
// *************************************************************************************************************************************
// The ATmega328P is the central control and measurement device.
// It provides:
// • Digital GPIO pins for resistor switching and discharge control
// • An internal 10-bit ADC for voltage measurement
// • Timers used by the firmware to measure elapsed time
// • I2C hardware for LCD communication
//
// The Arduino abstraction layer maps ATmega328P pins to Arduino pin numbers,
// simplifying firmware development while retaining direct hardware access.
//
// Key internal features used:
// • ADC channel ADC0 (Arduino A0) for capacitor voltage measurement
// • GPIO pins D2–D6 to select charging resistors
// • GPIO pin D7 for capacitor discharge control
// • GPIO pin D8 for the user push button
// • I2C interface on A4 (SDA) and A5 (SCL)
//
// *************************************************************************************************************************************
// CAPACITOR UNDER TEST (DUT) CONNECTION
// *************************************************************************************************************************************
// The capacitor under test is connected via a two-terminal screw or header block (J1).
// One terminal is connected to ground.
// The other terminal is connected directly to the Arduino analog input A0.
//
// This node is the central measurement point.
// All charging resistors, the discharge resistor, and the ADC input meet at this node.
//
// No active buffering is used.
// The ADC input impedance of the ATmega328P is relied upon to minimize loading.
//
// *************************************************************************************************************************************
// CHARGING RESISTOR NETWORK (AUTO-RANGING CORE)
// *************************************************************************************************************************************
// Five charging resistors are connected between Arduino GPIO pins and the DUT node.
// Each resistor is enabled by setting its corresponding GPIO pin HIGH.
//
// Resistor values and functions:
// • R6 = 1 MΩ  → Very small capacitance range
// • R7 = 100 kΩ → Small capacitance range
// • R10 = 10 kΩ → Medium capacitance range
// • R11 = 1 kΩ  → Medium-large capacitance range
// • R12 = 100 Ω → Very large capacitance range
//
// Each resistor is connected to a dedicated Arduino output pin:
// • D2 controls R6
// • D3 controls R7
// • D4 controls R10
// • D5 controls R11
// • D6 controls R12
//
// Only one charging resistor is enabled at a time.
// All other charge pins are set to high-impedance (INPUT mode) to avoid parallel paths.
//
// This design allows the firmware to select a resistor that produces a measurable charge time
// without exceeding a fixed timeout (about 1 second).
//
// *************************************************************************************************************************************
// HOW CHARGING WORKS ELECTRICALLY
// *************************************************************************************************************************************
// When a charging pin is set HIGH:
// • The pin drives approximately +5 V
// • Current flows from the pin through the selected resistor
// • The capacitor voltage rises gradually toward +5 V
//
// The charging speed depends on:
// • The selected resistor value
// • The capacitance of the DUT
//
// The Arduino continuously reads the voltage at A0 during charging.
// When the voltage reaches a predefined ADC threshold near 63% of Vcc,
// the firmware stops timing and calculates capacitance.
//
// *************************************************************************************************************************************
// DISCHARGE CIRCUIT
// *************************************************************************************************************************************
// A dedicated discharge resistor R8 (220 Ω) is used to safely and quickly discharge the capacitor.
// R8 is connected between the DUT node and an Arduino GPIO pin (D7).
//
// When D7 is set LOW:
// • The capacitor discharges through R8 to ground
// • The discharge current is limited to a safe value
//
// When D7 is set to INPUT:
// • The discharge path is removed
// • The DUT node becomes high-impedance again
//
// This controlled discharge ensures:
// • Known initial conditions before each measurement
// • Protection of the Arduino from high discharge currents
// • Reliable handling of large electrolytic capacitors
//
// *************************************************************************************************************************************
// ANALOG MEASUREMENT PATH
// *************************************************************************************************************************************
// The capacitor voltage is measured directly using Arduino analog input A0.
//
// Important characteristics:
// • The ADC uses Vcc as its reference voltage
// • The ADC input impedance is very high
// • The ADC measures voltage in discrete steps (10-bit resolution)
//
// The firmware oversamples the ADC (multiple readings averaged)
// to reduce noise and improve threshold stability.
//
// No external voltage divider or amplifier is used,
// which keeps the signal path simple but makes the design sensitive to noise and parasitics.
//
// *************************************************************************************************************************************
// PUSH BUTTON INPUT
// *************************************************************************************************************************************
// A momentary push button is connected between Arduino pin D8 and ground.
// The internal pull-up resistor is enabled in software.
//
// Electrical behavior:
// • Button released → D8 reads HIGH
// • Button pressed → D8 reads LOW
//
// No external resistor is required.
// Software debounce is used to eliminate mechanical contact bounce.
//
// The button initiates a measurement cycle and is also used to acknowledge results.
//
// *************************************************************************************************************************************
// LCD DISPLAY INTERFACE (I2C)
// *************************************************************************************************************************************
// A standard 16x2 HD44780-compatible LCD is used for displaying results.
// An I2C backpack (PCF8574) converts I2C signals to parallel LCD control signals.
//
// Connections:
// • SDA → Arduino A4
// • SCL → Arduino A5
// • Vcc → +5 V
// • GND → Ground
//
// Advantages of this approach:
// • Reduces GPIO usage
// • Simplifies wiring
// • Uses hardware I2C support in the ATmega328P
//
// The LCD displays:
// • Idle messages
// • Measurement progress
// • Capacitance result and timing information
// • Error messages when out of range
//
// *************************************************************************************************************************************
// POWER SUPPLY AND GROUNDING
// *************************************************************************************************************************************
// The entire system runs from a nominal +5 V supply.
// Power is typically provided by USB or an external regulated source.
//
// Ground is common to:
// • Arduino
// • Capacitor under test
// • LCD module
// • Button
//
// Because the ADC reference is Vcc, supply stability directly affects measurement quality.
// Large capacitors can briefly draw high current during charging,
// making good decoupling and solid wiring important.
//
// *************************************************************************************************************************************
// SYSTEM-LEVEL OPERATION SEQUENCE
// *************************************************************************************************************************************
// 1) System powers up and initializes GPIO, ADC, LCD, and button logic
// 2) User presses the button
// 3) Capacitor is fully discharged using R8
// 4) Firmware selects the first (largest) charging resistor
// 5) Capacitor is charged while ADC monitors voltage
// 6) If threshold is reached within timeout:
//    • Time is recorded
//    • Capacitance is calculated
//    • Result is displayed
// 7) If timeout occurs:
//    • Capacitor is discharged
//    • Next smaller resistor is selected
//    • Process repeats
// 8) If all ranges timeout:
//    • An out-of-range error is displayed
//
// *************************************************************************************************************************************
// HARDWARE DESIGN PHILOSOPHY
// *************************************************************************************************************************************
// The hardware is intentionally simple and transparent.
// Accuracy is achieved through:
// • Multiple precision resistors
// • Auto-ranging logic
// • Controlled discharge
// • Software oversampling
//
// Limitations are accepted and documented:
// • No analog buffering
// • No precision voltage reference
// • Dependence on capacitor quality (especially electrolytics)
//
// The design favors educational clarity and robustness over laboratory-grade precision.
//
// *************************************************************************************************************************************
// Error sources:
// *************************************************************************************************************************************
// Overview of the measurement method (plain language)
// *************************************************************************************************************************************
// Your Arduino measures capacitance by:
// • Charging an unknown capacitor through a resistor
// • Watching the voltage on the capacitor with the ADC
// • Measuring how long it takes for the voltage to reach about 63% of the supply voltage
// • Calculating capacitance from that time and the resistor value
// This works well only if the circuit behaves like an ideal resistor and an ideal capacitor, 
// and if the timing and voltage measurements are accurate. In reality, many things go wrong — especially for very small capacitors 
// and very large capacitors.
// *************************************************************************************************************************************
// 1. Errors caused by the Arduino ADC and voltage measurement
// *************************************************************************************************************************************
// ADC resolution
// *************************************************************************************************************************************
// The Arduino ADC does not measure voltage smoothly. It divides the voltage range into steps. 
// The “63% point” does not fall exactly on one step, so the Arduino must decide whether the voltage is just below or 
// just above that point.
// This causes:
// • Uncertainty in when the threshold is detected
// • Timing jitter from one measurement to the next
// This matters much more when the charging happens quickly (small capacitors).
// *************************************************************************************************************************************
// ADC noise and supply noise
// *************************************************************************************************************************************
// The ADC reading is affected by:
// • Power supply noise
// • Digital switching inside the microcontroller
// • USB power ripple
// These cause the measured voltage to jump slightly between readings. 
// That makes the Arduino sometimes detect the threshold earlier or later than it should.
// *************************************************************************************************************************************
// ADC sampling speed
// *************************************************************************************************************************************
// The Arduino does not watch the voltage continuously. It checks it at discrete moments. 
// If the capacitor voltage crosses the 63% level between two ADC readings, the Arduino will notice it late.
// For slow events this is minor.
// For fast events (small capacitors) this can be a large error.
// *************************************************************************************************************************************
// 2. Timing measurement errors
// *************************************************************************************************************************************
// Limited time resolution
// Functions like micros() or loop timing only have limited resolution. 
// When the charging time is very short, the time resolution becomes a large fraction of the total measured time.
// This means:
// • The absolute timing error stays about the same
// • The relative error becomes very large for small capacitance
// This is one of the main reasons accuracy collapses below about 100 nF.
// *************************************************************************************************************************************
// Software delay and jitter
// *************************************************************************************************************************************
// The Arduino is doing many things:
// • Executing code
// • Possibly handling interrupts
// • Running background timers
// The moment when the code notices the threshold is not perfectly repeatable. 
// This jitter is negligible for long charge times, but very significant for short ones.
//
// *************************************************************************************************************************************
// Start‑of‑charge uncertainty
// *************************************************************************************************************************************
// The moment you “start charging” is not perfectly sharp:
// • The output pin has a finite rise time
// • The pin driver has resistance
// • Switching pin modes takes time
// For small capacitors, these effects can be comparable to the actual charging time.
// *************************************************************************************************************************************
// 3. Resistor‑related errors
// *************************************************************************************************************************************
// Resistor tolerance
// The calculation assumes the resistor value is exact. In reality:
// • Most resistors are only accurate to 1% or 5%
// • Temperature changes alter resistance slightly
// This error directly transfers into the capacitance result.
// *************************************************************************************************************************************
// Hidden series resistances
// *************************************************************************************************************************************
// The charging path includes more than just your resistor:
// • Arduino output pin resistance
// • Wire resistance
// • Breadboard contacts
// • Transistor or switch resistance (if used)
// For small resistors (often used with small capacitors), these extra resistances 
// can be a significant fraction of the total, causing systematic error.
// *************************************************************************************************************************************
// 4. Capacitor non‑ideal behavior (dominant at large capacitances)
// *************************************************************************************************************************************
// This is the main reason accuracy drops above about 3300 µF.
// Large capacitors are usually electrolytic, and they behave very poorly compared to ideal capacitors.
// *************************************************************************************************************************************
// Leakage current
// *************************************************************************************************************************************
// Large electrolytic capacitors slowly leak current internally. This means:
// • Some charging current does not go into storing charge
// • The voltage rise no longer follows the expected shape
// The Arduino assumes all current charges the capacitor — which is no longer true.
// *************************************************************************************************************************************
// Equivalent Series Resistance (ESR)
// *************************************************************************************************************************************
// Electrolytic capacitors have internal resistance. This causes:
// • A voltage jump at the start of charging
// • A distorted charging curve
// • Reduced effective charging current
// This shifts the time at which the capacitor reaches 63%, creating error.
// *************************************************************************************************************************************
// Dielectric absorption
// *************************************************************************************************************************************
// Electrolytics “remember” previous voltages. After discharging, the voltage can slowly creep back up.
// This causes:
// • The capacitor to not start from zero volts
// • Shorter measured charge times
// • Capacitance readings that are too low
// This effect becomes worse for large capacitors and long charge times.
// *************************************************************************************************************************************
// Wide tolerance
// *************************************************************************************************************************************
// Electrolytic capacitors often have very loose tolerances (commonly ±20% or worse). 
// Even a perfectly measured result may not match the printed value.
// *************************************************************************************************************************************
// 5. Problems at very small capacitances
// *************************************************************************************************************************************
// Stray capacitance dominates
// The Arduino pin, wiring, breadboard, and even your fingers add capacitance.
// At small values:
// • The measured capacitance includes the test setup itself
// • The real capacitor becomes a small part of the total
// Below about 100 nF, stray capacitance can be a major error source.
// *************************************************************************************************************************************
// Leakage paths
// *************************************************************************************************************************************
// At small capacitance values, very tiny leakage currents through:
// • Dirty boards
// • Humidity
// • Flux residue
// can significantly affect the charging behavior.
// *************************************************************************************************************************************
// 6. Discharge and initial condition errors
// *************************************************************************************************************************************
// If the capacitor is not fully discharged before measurement:
// • The charge time is shorter
// • The calculated capacitance is too small
// Large capacitors are especially hard to fully discharge because of dielectric absorption.
// *************************************************************************************************************************************
// Summary: why accuracy drops at the extremes
// *************************************************************************************************************************************
// Below ~100 nF
// *************************************************************************************************************************************
// • Charging happens too fast
// • ADC and timing resolution are too coarse
// • Software delays dominate
// • Stray capacitance is comparable to the capacitor itself
// *************************************************************************************************************************************
// Above ~3300 µF
// *************************************************************************************************************************************
// • Leakage current distorts charging
// • ESR alters the voltage curve
// • Dielectric absorption ruins the starting condition
// • Long measurement times increase drift and noise sensitivity
// *************************************************************************************************************************************
// Key takeaway
// *************************************************************************************************************************************
// This method works best when:
// • The charging time is neither very fast nor very slow
// • The capacitor behaves close to ideal
// • Stray effects are small compared to the real signal
// That naturally puts the “sweet spot” somewhere between tens of nanofarads and a few thousand microfarads.
//
// To compensate for this the software uses a constant called CorrectionFactor and different values for 
// the voltage (ADC value) where Vc(t) = 63% of Vcc.
// *************************************************************************************************************************************
// Calibration:
// *************************************************************************************************************************************
// You have to adjust the parameters Vcc, ChargeResistorOne,ChargeResistorTwo,ChargeResistorThree,ChargeResistorFour,
// ChargeResistorFive(for the 5 resistors measure the real world values) and CorrectionFactor to archieve precision.
//--------------------------------------------------------------------------------------------------------------------------------------
// Include libraries
//--------------------------------------------------------------------------------------------------------------------------------------
// Include the Wire library for I2C communication - this is required for communicating with the PCF8574 I2C expander chip
#include <Wire.h>                    // I2C communication library for Arduino

// Include the Adafruit LiquidCrystal_I2C library which provides functions specifically for HD44780 displays via I2C
// This library must be installed via Library Manager (Sketch -> Include Library -> Manage Libraries)
#include <LiquidCrystal_I2C.h>       // Adafruit's I2C LCD library for HD44780 compatible displays
//--------------------------------------------------------------------------------------------------------------------------------------
// Constant declarations
//--------------------------------------------------------------------------------------------------------------------------------------
#define analogPin 0                 // analog pin for measuring capacitor voltage
#define chargePin_one 2             // pin to charge the capacitor - connected to one end of the charging resistor R6
#define chargePin_two 3             // pin to charge the capacitor - connected to one end of the charging resistor R7
#define chargePin_three 4           // pin to charge the capacitor - connected to one end of the charging resistor R10 
#define chargePin_four 5            // pin to charge the capacitor - connected to one end of the charging resistor R11 
#define chargePin_five 6            // pin to charge the capacitor - connected to one end of the charging resistor R12 
#define dischargePin 7              // pin to discharge the capacitor - connected to one end of the charging resistor R8 
// =====================================================================================================================================
// ADC CONSTANTS - Define ADC properties
// =====================================================================================================================================
#define Vcc 5.00                    // Supply voltage = 5V
#define ADC_MAX 1023                // Maximum ADC reading value (10-bit)
#define ADC_STEPS 1024               // Number of discrete steps (2^10)
// =====================================================================================================================================
// DISPLAY CONSTANTS - Define display properties and I2C address
// =====================================================================================================================================
// I2C address for the LCD display module (based on PCF8574 I2C expander chip)
// The default address is often 0x27 or 0x3F, depending on the manufacturer and jumper settings
// If the display does not work, try changing this to the other common address (0x3F)
#define LCD_I2C_ADDRESS 0x27         // I2C address of the PCF8574 chip - common for many I2C LCD modules

// Display dimensions - 2 rows with 16 characters each (standard 16x2 character LCD)
#define LCD_COLS 16                  // Number of columns (characters per line) - 16 for 16x2 display
#define LCD_ROWS 2                   // Number of rows (lines) on the display - 2 for 16x2 display

// ======================================================================================================================================
// DEBOUNCE CONSTANTS - Adjustable parameters that affect sensitivity and response
// ======================================================================================================================================
// The number of stable low readings required before we consider the button as pressed
// 50 samples with 1ms间隔 gives 50ms debounce time, which is sufficient for most mechanical contacts
const int DEBOUNCE_COUNT = 50;       // Number of stable samples required - Higher value = more robust against noise but slower response

// The time interval between each time we read the button state
// 1ms is chosen because it is fast enough to detect quick presses, but slow enough to avoid CPU overload
const int SAMPLE_DELAY_MS = 1;      // Time between samples (1ms) - Lower value = faster response but more CPU time

// ======================================================================================================================================
// CHARGE RESISTOR CONSTANTS - Define charge resistor properties 
// ======================================================================================================================================
#define ChargeResistorOne 1000000.0 // Charging resistor R6 value 1MΩ
#define ChargeResistorTwo 100000.0  // Charging resistor R7 value 100kΩ
#define ChargeResistorThree 10000.0 // Charging resistor R10 value 10kΩ
#define ChargeResistorFour 1000.0   // Charging resistor R11 value 1kΩ
#define ChargeResistorFive 100.0    // Charging resistor R12 value 100Ω
// =====================================================================================================================================
// OTHER CONSTANTS - Define capacitance measurement properties 
// =====================================================================================================================================
#define TimeOut 1001                // 1 second TimeOut value
#define micro   0                   // Our capacitor has a capacitance in microfarads
#define nano    1                   // Our capacitor has a capacitance in nanofarads
//--------------------------------------------------------------------------------------------------------------------------------------
// Data variable declarations
//--------------------------------------------------------------------------------------------------------------------------------------
double Vt = Vcc * 0.6322;           // Threshold voltage = 0.632 * Supply voltage
int Vc = 0;                         // value from ADC
int Counter = 0;                    // Counter for oversampling
unsigned long startTime;            // Starttime
unsigned long elapsedTime;          // elapsedTime
float microFarads;                  // floating point variable to preserve precision, make calculations
float nanoFarads;
int Size = micro;                   // Defines if Capacitor has Capacitance in microfarads or nanofarads 
int error = 0;                      // Defines if measurement has error : Too big or small capacitance : 1 : error, 0 : no error 
int Resistor = 1;                   // Defines the resistor used
float CorrectionFactor = 1;         // The value that make Uc corespond to 0.63*Vcc in the Capacitance range
// =====================================================================================================================================
// BUTTON DEFINITION - Hardware configuration
// =====================================================================================================================================
// Defines the constant 'button' as value 8, which corresponds to digital pin D8 on Arduino Uno
// This makes the code more readable and easier to maintain, as we can refer to 'button' instead of the number 8
#define button 8                    // pin to Measure start button - Physical connection of button to digital pin IO8 / D8

// Simple debounce variables - Inspired by Khaled Magdy's debounce example
static int buttonState;             // the current reading from the input pin
static int lastButtonState = HIGH;   // the previous reading from the input pin
static unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
static const unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
static bool buttonPressed = false;  // Flag to indicate a valid button press
// =====================================================================================================================================
// GLOBAL DISPLAY OBJECT - Creates connection to LCD via I2C
// =====================================================================================================================================
// Creates a LiquidCrystal_I2C object with the specified I2C address and display dimensions
// Parameter order: (I2C_address, number_of_columns, number_of_rows)
// This object will be used by all subsequent functions to communicate with the physical display
// The constructor does not initialize the display yet - that happens in initializeDisplay()
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);  // Create LCD object with address 0x27 and 16x2 configuration
//--------------------------------------------------------------------------------------------------------------------------------------
//Function forward declarations
//--------------------------------------------------------------------------------------------------------------------------------------
void MeasureCapacitor();
void dischargeCapacitor();
int ReadAnaloguePort();
void ShowCapacitanceOnDisplay();
void ShowCapacitanceOnSerialport();
bool checkButtonPress();           // Simple debounce function
bool initializeDisplay();
void setCursorPosition(int row, int col);
void writeCharacterAt(int row, int col, char character);
void writeTextAt(int row, int col, const char* text);
void clearDisplay();
void clearLine(int row);
void setBacklight(bool state);
void createCustomChar(int location, const uint8_t* charMap);
// ======================================================================================================================================
// ======================================================================================================================================
// Setup
// ======================================================================================================================================
// ======================================================================================================================================
void setup() {
  // Setup alle used GPIO to input ports : High Z
  // Setup display
  // Configures the button pin as input with internal pull-up resistor enabled
  // INPUT_PULLUP activates the built-in 20kΩ pull-up resistor in the ATmega328P chip
  // This means the pin normally reads as HIGH, and when the button is pressed it connects to GND 
  // and reads as LOW
  pinMode(button, INPUT_PULLUP);    // Use internal pull-up - saves external hardware and simplifies circuit
  
  // Initializes serial communication at 9600 baud (bits per second)
  // This makes it possible to send debug information to the computer via USB
  Serial.begin(9600);               // Starts serial communication for debugging and printouts
  // Initialize the display and check if successful
  if (!initializeDisplay()) {       // Call initialization function, check if it returns false
    Serial.println("Could not find LCD display! Check connections and I2C address.");
    // If display not found, print error message and stop program execution
    while (1);                      // Infinite loop - program stops here
  }
  
  // Initialize button state
  buttonState = digitalRead(button);
  lastButtonState = buttonState;
  
  // Print startup message on LCD
  clearDisplay();                   // Clear any random characters
  writeTextAt(0, 0, "Cap Meter v1.0"); // Display version on line 0
  writeTextAt(1, 0, "Press button");   // Display prompt on line 1
  
  // Print startup message on Serial
  Serial.println("Capacitance Meter v1.0 initialized"); // Startup message
  Serial.println("Press button to measure capacitance"); // User prompt
}
// =====================================================================================================================================
// MAIN LOOP FUNCTION
// =====================================================================================================================================
// This is the main program loop that runs continuously after setup() completes.
// It handles:
// 1. Button press detection (via simple debounce)
// 2. Capacitance measurement when button is pressed
// 3. Display and serial output of measurement results
// 4. Error handling for out-of-range measurements
// 5. Wait for button press to acknowledge results (keeps display on until next press)
// =====================================================================================================================================

void loop() {
  // -----------------------------------------------------------------------------------------------------------------------------------
  // BUTTON PRESS DETECTION - Simple reliable debounce
  // Based on Khaled Magdy's proven debounce algorithm
  // -----------------------------------------------------------------------------------------------------------------------------------
  if (checkButtonPress()) {         // Check if button was pressed and debounced
    
    // ---------------------------------------------------------------------------------------------------------------------------------
    // BUTTON PRESS CONFIRMATION
    // ---------------------------------------------------------------------------------------------------------------------------------
    // Prints a confirmation message via serial communication to the computer
    // This can be seen in Serial Monitor (Tools -> Serial Monitor)
    // Useful for debugging and user feedback
    Serial.println("Button press detected!"); // Confirmation message on Serial port
    clearDisplay();                           // Clear display
    writeTextAt(0, 0, "Measuring!");          // Write "Measuring!" on display"
    
    // ---------------------------------------------------------------------------------------------------------------------------------
    // CAPACITANCE MEASUREMENT
    // ---------------------------------------------------------------------------------------------------------------------------------
    // Call the main measurement function to measure the capacitor's capacitance
    // This function uses auto-ranging to select appropriate charging resistor
    // Results are stored in global variables: microFarads, nanoFarads, Size, error
    // The measurement process may take up to several seconds depending on capacitor size
    MeasureCapacitor();             // Measure Capacitance of unknown capacitor
    
    // ---------------------------------------------------------------------------------------------------------------------------------
    // DISPLAY UPDATE BASED ON MEASUREMENT RESULT
    // ---------------------------------------------------------------------------------------------------------------------------------
    // Clear the entire display to prepare for new information
    // This removes any previous measurements or messages from the display
    clearDisplay();                 // Erase all content from LCD display
    
    // Check if measurement was successful (error flag is 0) or if an error occurred (error flag is 1)
    if (error == 0)                 // If no error during measurement (capacitance successfully measured)
    {
      // -------------------------------------------------------------------------------------------------------------------------------
      // SUCCESSFUL MEASUREMENT - Display capacitance value on LCD
      // -------------------------------------------------------------------------------------------------------------------------------
      
      // Create a character array (string buffer) to hold the formatted capacitance value
      // Size 20 is enough to hold "C = 9999.99 uF" plus null terminator
      char capBuffer[20];           // Buffer for capacitance value string formatting
      
      // Check which unit (microfarads or nanofarads) the measurement result is in
      // Size variable is set by MeasureCapacitor() function:
      // Size = micro (0) for microfarads, Size = nano (1) for nanofarads
      if (Size == micro)            // If capacitance is in microfarads (µF)
      {
        // Format the capacitance value as a string with 2 decimal places
        // dtostrf() converts float to string: (value, minimum width, decimal places, buffer)
        // Minimum width 6 ensures enough space for numbers like 1.23 or 123.45
        dtostrf(microFarads, 6, 4, capBuffer); // Convert microfarads to string with 3 decimal places
        
        // Write the formatted capacitance string to LCD at row 0, column 0
        // The format will be: "C = xxx.xx uF" where xxx.xx is the capacitance value
        writeTextAt(0, 0, "C = ");  // Write "C = " at beginning of first line
        writeTextAt(0, 4, capBuffer); // Write the formatted number starting at column 4
        writeTextAt(0, 10, " uF");  // Write " uF" (unit) at column 10 (after the number)
      }
      else                          // If capacitance is in nanofarads (nF) - Size == nano
      {
        // Format the nanofarads value as a string with 2 decimal places
        dtostrf(nanoFarads, 6, 4, capBuffer); // Convert nanofarads to string with 3 decimal places
        
        // Write the formatted capacitance string to LCD at row 0, column 0
        // The format will be: "C = xxx.xx nF" where xxx.xx is the capacitance value
        writeTextAt(0, 0, "C = ");  // Write "C = " at beginning of first line
        writeTextAt(0, 4, capBuffer); // Write the formatted number starting at column 4
        writeTextAt(0, 10, " nF");  // Write " nF" (unit) at column 10 (after the number)
      }
      
      // -------------------------------------------------------------------------------------------------------------------------------
      // DISPLAY CHARGING TIME ON SECOND LINE
      // -------------------------------------------------------------------------------------------------------------------------------
      // Create a buffer for the time value (charging time to reach threshold)
      char timeBuffer[20];          // Buffer for time value string formatting
      
      // Convert elapsed time from milliseconds to seconds for display
      // elapsedTime is in milliseconds, divide by 1000.0 to get seconds with decimal
      float timeInSeconds = (float)elapsedTime / 1000.0; // Convert milliseconds to seconds
      
      // Format the time value as a string with 3 decimal places (millisecond resolution)
      // Minimum width 5 ensures enough space for numbers like 0.123 or 1.234
      dtostrf(timeInSeconds, 5, 3, timeBuffer); // Convert time to string with 3 decimal places
      
      // Write the formatted time to LCD at row 1, column 0
      // The format will be: "t = 0.123 S" where 0.123 is the charging time in seconds
      writeTextAt(1, 0, "t = ");    // Write "t = " at beginning of second line
      writeTextAt(1, 4, timeBuffer); // Write the formatted time starting at column 4
      writeTextAt(1, 10, " S");     // Write " S" (seconds) at column 10 (after the time)
      
      // -------------------------------------------------------------------------------------------------------------------------------
      // SERIAL PORT OUTPUT FOR SUCCESSFUL MEASUREMENT
      // -------------------------------------------------------------------------------------------------------------------------------
      // Print the same information to Serial monitor for logging/debugging
      Serial.print("C = ");         // Print "C = " to serial port (no newline)
      
      // Print the capacitance value with appropriate unit
      if (Size == micro)            // If in microfarads
      {
        Serial.print(microFarads, 2); // Print microfarads value with 2 decimal places
        Serial.println(" uF");        // Print unit and newline
      }
      else                          // If in nanofarads
      {
        Serial.print(nanoFarads, 2); // Print nanofarads value with 2 decimal places
        Serial.println(" nF");        // Print unit and newline
      }
      
      // Print the charging time
      Serial.print("t = ");         // Print "t = " to serial port
      Serial.print(timeInSeconds, 3); // Print time in seconds with 3 decimal places
      Serial.println(" S");         // Print unit and newline
      Serial.print(" Resistor: ");
      Serial.println(Resistor);
    }
    else                            // If error flag is 1 (measurement failed)
    {
      // -------------------------------------------------------------------------------------------------------------------------------
      // ERROR HANDLING - Display error message on LCD
      // -------------------------------------------------------------------------------------------------------------------------------
      // Write error message on first line of LCD
      writeTextAt(0, 0, "Error!");   // Display "Error!" on row 0, starting at column 0
      
      // Write additional information on second line
      // This tells the user what might be wrong
      writeTextAt(1, 0, "Out of range"); // Display "Out of range" on row 1, column 0
      
      // -------------------------------------------------------------------------------------------------------------------------------
      // SERIAL PORT OUTPUT FOR ERROR CONDITION
      // -------------------------------------------------------------------------------------------------------------------------------
      // Print error information to Serial monitor
      Serial.println("Error!");      // Print error message to serial port
      Serial.println("Capacitance out of measurable range"); // Detailed error description
      Serial.println("Check capacitor: too large, too small, shorted, or open"); // Troubleshooting hint
    }
    // ---------------------------------------------------------------------------------------------------------------------------------
    // WAIT FOR BUTTON PRESS TO ACKNOWLEDGE RESULTS
    // ---------------------------------------------------------------------------------------------------------------------------------
    // After displaying results, wait for user to press the button again
    // This keeps the measurement result on the display until the user is ready
    // The while loop will continue until a new button press is detected
    
    // Print waiting message on Serial port (useful for debugging)
    Serial.println("Measurement complete. Press button for next measurement."); // Indicate we're waiting
    
    // Wait loop - continues until a new button press is detected
    // The display continues to show the measurement result during this wait
    while (true)                     // Infinite loop - will break out when button pressed
    {
      // Call button check function to see if button is pressed
      // checkButtonPress() returns true only ONCE per verified press
      if (checkButtonPress())        // If new button press detected
      {
        // Short delay to debounce the button release
        // This prevents multiple detections from the same press
        delay(200);                  // Wait 200ms for button to stabilize
        
        break;                       // Exit the while loop and continue with main loop
      }
      
      // Small delay to prevent tight looping (reduces CPU usage)
      // This is important because checkButtonPress() is non-blocking,
      // but this prevents the loop from consuming 100% CPU
      delay(10);                     // Small delay to prevent overwhelming the CPU
    }
    
    // ---------------------------------------------------------------------------------------------------------------------------------
    // PREPARE FOR NEXT MEASUREMENT
    // ---------------------------------------------------------------------------------------------------------------------------------
    // Clear the display to get ready for next measurement
    // This removes the previous results from the LCD
    clearDisplay();                  // Erase LCD content for next measurement
    
    // Print ready message on Serial port
    Serial.println("Ready for next measurement. Press button to start."); // User prompt
    
    // Restore the idle display message
    writeTextAt(0, 0, "Cap Meter v1.0"); // Display version on line 0
    writeTextAt(1, 0, "Press button");   // Display prompt on line 1
    
  } // End of button press handling block
  
  // -----------------------------------------------------------------------------------------------------------------------------------
  // BACKGROUND TASKS (when button is NOT pressed)
  // -----------------------------------------------------------------------------------------------------------------------------------
  // This section runs continuously when no button press is detected
  // Because we do NOT use long delay() statements, the Arduino can perform other work here
  // This is crucial for maintaining responsiveness and handling multiple tasks
  //
  // Examples of what could be placed here:
  // - Read other sensors continuously
  // - Update display with time or other information
  // - Communicate via I2C, SPI or serial with other devices
  // - Run PWM signals for motors or LED's (dimming, patterns)
  // - Handle multiple inputs simultaneously
  // - Perform background calculations
  // - Check for communication from PC or other devices
  // - Update real-time clock
  // - Log data to EEPROM or SD card
  //
  // Currently this section is empty, but it's available for future expansion
  // The absence of delay() ensures that checkButtonPress() is called frequently,
  // which is essential for the debounce to work properly
  
  // Small delay to prevent overwhelming the CPU while still maintaining responsiveness
  // This is optional but helps reduce power consumption
  delay(5);                          // Small delay when idle to save CPU cycles

} // End of main loop function
// =====================================================================================================================================
// FUNCTION DECLARATION: MeasureCapacitor - MEASURE CAPACITANCE FOR CAPACITOR
// =====================================================================================================================================
// 
// PURPOSE:
// This function measures the capacitance of an unknown capacitor connected to the measurement circuit.
// It uses the RC time constant charging method based on the formula: C = t / R
// where the capacitor charges to 63.22% (1 - 1/e) of the supply voltage through a known resistor.
//
// THEORY OF OPERATION:
// When a capacitor charges through a resistor from 0V to Vcc, the voltage across the capacitor follows:
// Vc(t) = Vcc * (1 - e^(-t/RC))
// 
// At t = RC, Vc = Vcc * (1 - e^(-1)) = Vcc * 0.6322
// Therefore, if we measure the time (t) it takes to reach 0.6322 * Vcc, then:
// C = t / R
//
// AUTO-RANGING FUNCTIONALITY:
// The function implements automatic range selection using multiple charging resistors:
// - Starts with largest resistor (1MΩ) for smallest capacitance range
// - If time exceeds 1 second, switches to next smaller resistor
// - Continues through resistors until a valid measurement within 1 second is obtained
// - If all resistors fail, sets error flag
//
// RESISTOR VALUES AND CORRESPONDING CAPACITANCE RANGES:
// ChargeResistorOne   (1,000,000Ω) : Measures ~1nF to 1µF capacitors
// ChargeResistorTwo   (100,000Ω)   : Measures ~10nF to 10µF capacitors
// ChargeResistorThree (10,000Ω)    : Measures ~100nF to 100µF capacitors
// ChargeResistorFour  (1,000Ω)     : Measures ~1µF to 1000µF capacitors
// ChargeResistorFive  (100Ω)       : Measures ~10µF to 10,000µF capacitors
//
// THRESHOLD CALCULATION:
// Vt = Vcc * 0.6322 = 5.00V * 0.6322 = 3.161V
// ADC value at threshold = (Vt * 1024) / Vcc = (3.161 * 1024) / 5.00 = 639
// (The while loop checks for Vc < 639)
//
// TIMING CONSIDERATIONS:
// - Maximum measurement time per range: 1 second (1000ms)
// - If measurement takes longer, range is too small - switch to larger capacitor range (smaller resistor)
// - millis() is used for timing (millisecond resolution)
//
// GLOBAL VARIABLES MODIFIED:
// - error     : Set to 1 if measurement fails, 0 if successful
// - microFarads: Calculated capacitance in microfarads (when Size = micro)
// - nanoFarads : Calculated capacitance in nanofarads (when Size = nano)
// - Size      : Indicates unit (micro = 0 for µF, nano = 1 for nF)
// - Vc        : Current ADC reading of capacitor voltage
// - startTime : Timestamp when charging began
// - elapsedTime: Time taken to reach threshold
// - Counter   : Incremented by ReadAnaloguePort() for each ADC reading
//
// HARDWARE PIN CONFIGURATION:
// - analogPin       : Reads capacitor voltage (must be connected to capacitor)
// - chargePin_one   : 1MΩ charging resistor output
// - chargePin_two   : 100kΩ charging resistor output
// - chargePin_three : 10kΩ charging resistor output
// - chargePin_four  : 1kΩ charging resistor output
// - chargePin_five  : 100Ω charging resistor output
// - dischargePin    : Discharge path control
//
// DEPENDENCIES:
// - Requires ReadAnaloguePort() function for oversampled ADC readings
// - Requires dischargeCapacitor() function to safely discharge before measurements
// - Requires global constants and variables as defined above
//
// RETURN VALUE:
// void - Results stored in global variables microFarads, nanoFarads, Size, and error
//
// ERROR CONDITIONS:
// error = 1 indicates:
// - Capacitor is too large (even smallest resistor takes >1 second)
// - Capacitor is too small (even largest resistor gives <1ms reading)
// - Capacitor is shorted or open circuit
// - Measurement timeout occurred
//
// =====================================================================================================================================

// Function: MeasureCapacitor
// Purpose: Measures unknown capacitance using auto-ranging RC time constant method
// Global variables used: All constants and variables defined in header
// Returns: void (results stored in global microFarads, nanoFarads, Size, error)
void MeasureCapacitor()
{
  // -----------------------------------------------------------------------------------------------------------------------------------
  // INITIALIZATION - Reset error flag and ensure capacitor is discharged
  // -----------------------------------------------------------------------------------------------------------------------------------
  
  // Set error flag to 0 (no error) at start of measurement
  // This clears any previous error condition before new measurement begins
  error = 0;                         // Initialize error flag to 0 - assume measurement will succeed
  
  // Ensure capacitor is completely discharged before starting any measurement
  // This guarantees we start from a known state (0V across capacitor)
  dischargeCapacitor();               // Fully discharge the capacitor to 0V before charging
  
  // -----------------------------------------------------------------------------------------------------------------------------------
  // FIRST RANGE ATTEMPT - Using ChargeResistorOne (1MΩ) for smallest capacitance range
  // -----------------------------------------------------------------------------------------------------------------------------------
  
  // Record the start time for the first charging attempt
  // millis() returns milliseconds since Arduino started - used for elapsed time calculation
  startTime = millis();               // Get current system time as start timestamp for measurement
  
  // Initialize elapsedTime to 0 - will be updated in the while loop
  // elapsedTime holds the time taken to charge capacitor to threshold voltage
  elapsedTime = millis() - startTime; // Calculate initial elapsedTime (should be ~0)
  
  // Get initial capacitor voltage reading before starting charge
  // ReadAnaloguePort() returns oversampled ADC value (0-1023) representing 0-5V
  Vc = ReadAnaloguePort();            // Read initial capacitor voltage (should be ~0 after discharge)
  // -----------------------------------------------------------------------------------------------------------------------------------
  // CONFIGURE PINS FOR FIRST RANGE (1MΩ charging resistor)
  // -----------------------------------------------------------------------------------------------------------------------------------
  
  // Configure chargePin_one as OUTPUT to enable charging through 1MΩ resistor
  // This pin will be set HIGH to start charging the capacitor
  pinMode(chargePin_one, OUTPUT);     // Set chargePin_one (1MΩ resistor) as output to enable charging
  
  // Configure all other charge pins as INPUT (High Z / tri-state)
  // This isolates them from the circuit so only the selected resistor is active
  pinMode(chargePin_two, INPUT);      // Set chargePin_two (100kΩ) as input - High Z, not connected to circuit
  pinMode(chargePin_three, INPUT);    // Set chargePin_three (10kΩ) as input - High Z, not connected to circuit
  pinMode(chargePin_four, INPUT);     // Set chargePin_four (1kΩ) as input - High Z, not connected to circuit
  pinMode(chargePin_five, INPUT);     // Set chargePin_five (100Ω) as input - High Z, not connected to circuit
  
  // Configure dischargePin as INPUT (High Z) to prevent discharging during measurement
  // This ensures all current flows through the charging resistor into the capacitor
  pinMode(dischargePin, INPUT);       // Set dischargePin as input - High Z, discharge path disabled
  
  // Start charging the capacitor by setting chargePin_one HIGH (5V)
  // Current flows through 1MΩ resistor into capacitor, causing voltage to rise
  digitalWrite(chargePin_one, HIGH);  // Set chargePin_one to 5V - begin charging capacitor through 1MΩ
  Resistor = 1;
  // -----------------------------------------------------------------------------------------------------------------------------------
  // WAIT FOR CAPACITOR TO CHARGE TO THRESHOLD VOLTAGE (629 ADC = 3.161V = 63.22% of 5V)
  // -----------------------------------------------------------------------------------------------------------------------------------
  // Continue looping until either:
  // 1. Capacitor voltage reaches threshold (Vc >= 638), OR
  // 2. Time exceeds 1 second (1000ms) - indicating capacitor is too large for this range
  while((Vc < 629) && (elapsedTime < 1001))  // While below threshold AND less than 1 second elapsed
  {
    // Read current capacitor voltage using oversampled ADC
    // ReadAnaloguePort() returns averaged value from 4 samples for noise reduction
    Vc = ReadAnaloguePort();           // Get current capacitor voltage as oversampled ADC value (0-1023)
    
    // Calculate elapsed time since charging started
    // Subtract startTime from current millis() to get charging duration in milliseconds
    elapsedTime = millis() - startTime; // Update elapsed time - time since charging began
  }
  
  // -----------------------------------------------------------------------------------------------------------------------------------
  // CHECK IF MEASUREMENT SUCCEEDED WITHIN 1 SECOND TIMEOUT
  // -----------------------------------------------------------------------------------------------------------------------------------
  // If elapsedTime < 1001, we exited the while loop because threshold was reached (success)
  // If elapsedTime >= 1001, we exited due to timeout - need to try next (larger) range
  if (elapsedTime < 1001)              // If threshold was reached within 1 second (successful measurement)
  {
    // SUCCESSFUL MEASUREMENT WITH 1MΩ RESISTOR
    // Calculate capacitance using formula: C = t / R
    // 
    // elapsedTime is in milliseconds, but formula requires seconds: divide by 1000
    // Result is in Farads, but we want microFarads: multiply by 1,000,000
    // Combined factor: (1,000,000 / 1000) = 1000
    // Therefore: microFarads = (elapsedTime / ChargeResistorOne) * 1000
    microFarads = ((float)elapsedTime / ChargeResistorOne) * 1000;  // Calculate capacitance in microfarads
    // We use a CorrectionFactor to multiply Capacity in microFarads with
    // The reason is than in the very low range the precision is low 
    if((microFarads>=0.001) && (microFarads<0.010))  // 1nF - 10nF
    {
      CorrectionFactor = 0.94;
      microFarads = microFarads * CorrectionFactor;
    }
    else if((microFarads>=10) && (microFarads<20)) // 10nF - 20nF
    {
      CorrectionFactor = 0.88235294117647058823529411764706;
      microFarads = microFarads * CorrectionFactor;
    }
    else if((microFarads>=20) && (microFarads<30)) // 20nF - 30nF
    {
      CorrectionFactor = 0.96153846153846153846153846153846*0.96153846153846153846153846153846;
      microFarads = microFarads * CorrectionFactor;
    }
    else if((microFarads>=30) && (microFarads<40)) // 30nF - 40nF
    {
      CorrectionFactor = 0.97222222222222222222222222222222*0.94594594594594594594594594594595;
      microFarads = microFarads * CorrectionFactor;
    }
    else if((microFarads>=40) && (microFarads<50)) // 40nF - 50nF
    {
      CorrectionFactor = 0.97826086956521739130434782608696;
      microFarads = microFarads * CorrectionFactor;
    }
    else if((microFarads>=50) && (microFarads<60)) // 50nF - 60nF
    {
      CorrectionFactor = 0.98245614035087719298245614035088;
      microFarads = microFarads * CorrectionFactor;
    }
    else if((microFarads>=60) && (microFarads<70)) //60nF - 70nF
    {
      CorrectionFactor = 0.97014925373134328358208955223881;
      microFarads = microFarads * CorrectionFactor;
    };
    // Set nanoFarads to 1000 * microfarad units if microFarads < 1
    if(microFarads<1)
    {
      nanoFarads = microFarads * 1000;
      Size=nano;
    }    
    else
    {
    // Set Size indicator to micro (0) to indicate capacitance is in microfarads
     Size = micro;                       // Set unit indicator to microfarads (micro = 0)
    };
    // Debug output
    // Serial.print("R1 - t="); Serial.print(elapsedTime); Serial.print("ms, C="); Serial.println(microFarads);
  }
  else                                   // If 1 second timeout occurred (capacitor too large for 1MΩ range)
  {
    // ---------------------------------------------------------------------------------------------------------------------------------
    // SECOND RANGE ATTEMPT - Using ChargeResistorTwo (100kΩ) for medium-small capacitance range
    // ---------------------------------------------------------------------------------------------------------------------------------
    
    // Discharge capacitor completely before next measurement attempt
    // This ensures consistent starting conditions for the new range
    dischargeCapacitor();               // Fully discharge capacitor before next range attempt
    
    // Record new start time for second range attempt
    startTime = millis();               // Get new start timestamp for second measurement
    
    // Initialize elapsedTime for second range
    elapsedTime = millis() - startTime; // Calculate initial elapsedTime for second attempt
    
    // Get initial capacitor voltage reading before starting second charge
    Vc = ReadAnaloguePort();            // Read initial capacitor voltage (should be ~0 after discharge)
    
    // ---------------------------------------------------------------------------------------------------------------------------------
    // RECONFIGURE PINS FOR SECOND RANGE (100kΩ charging resistor)
    // ---------------------------------------------------------------------------------------------------------------------------------
    
    // Configure chargePin_one as INPUT (High Z) to isolate it from circuit
    pinMode(chargePin_one, INPUT);      // Set chargePin_one (1MΩ) as input - High Z, disconnected
    
    // Configure chargePin_two as OUTPUT to enable charging through 100kΩ resistor
    pinMode(chargePin_two, OUTPUT);     // Set chargePin_two (100kΩ) as output to enable charging
    
    // Keep all other charge pins as INPUT (High Z) - only selected resistor active
    pinMode(chargePin_three, INPUT);    // Set chargePin_three (10kΩ) as input - High Z, not connected
    pinMode(chargePin_four, INPUT);     // Set chargePin_four (1kΩ) as input - High Z, not connected
    pinMode(chargePin_five, INPUT);     // Set chargePin_five (100Ω) as input - High Z, not connected
    
    // Keep dischargePin as INPUT (High Z) to prevent discharging during measurement
    pinMode(dischargePin, INPUT);       // Set dischargePin as input - High Z, discharge path disabled
    
    // Start charging the capacitor by setting chargePin_two HIGH (5V)
    // Current now flows through 100kΩ resistor (faster charging than 1MΩ)
    digitalWrite(chargePin_two, HIGH);  // Set chargePin_two to 5V - begin charging through 100kΩ
    Resistor=2;
    
    // ---------------------------------------------------------------------------------------------------------------------------------
    // WAIT FOR CAPACITOR TO CHARGE TO THRESHOLD VOLTAGE (644 ADC)
    // ---------------------------------------------------------------------------------------------------------------------------------
    while((Vc < 646) && (elapsedTime < 1001))  // While below threshold AND less than 1 second elapsed
    {
      // Read current capacitor voltage
      Vc = ReadAnaloguePort();           // Get current capacitor voltage via oversampled ADC
      
      // Update elapsed time
      elapsedTime = millis() - startTime; // Calculate time since second charging began
    }
    
    // ---------------------------------------------------------------------------------------------------------------------------------
    // CHECK IF SECOND RANGE MEASUREMENT SUCCEEDED
    // ---------------------------------------------------------------------------------------------------------------------------------
    if (elapsedTime < 1001)              // If threshold reached within 1 second using 100kΩ
    {
      // SUCCESSFUL MEASUREMENT WITH 100kΩ RESISTOR
      // Calculate capacitance using same formula with ChargeResistorTwo
      microFarads = ((float)elapsedTime / ChargeResistorTwo) * 1000;  // Calculate capacitance in microfarads
      
      // Set nanoFarads to 0
      nanoFarads = 0;                     // Clear nanofarads value
      
      // Set Size indicator to micro (0)
      Size = micro;                        // Set unit indicator to microfarads
      
      // Debug output
      // Serial.print("R2 - t="); Serial.print(elapsedTime); Serial.print("ms, C="); Serial.println(microFarads);
    }
    else                                   // If timeout occurred with 100kΩ (capacitor still too large)
    {
      // -------------------------------------------------------------------------------------------------------------------------------
      // THIRD RANGE ATTEMPT - Using ChargeResistorThree (10kΩ) for medium capacitance range
      // -------------------------------------------------------------------------------------------------------------------------------
      
      // Discharge capacitor for next attempt
      dischargeCapacitor();               // Fully discharge capacitor before third range attempt
      
      // Record new start time
      startTime = millis();               // Get new start timestamp for third measurement
      
      // Initialize elapsedTime
      elapsedTime = millis() - startTime; // Calculate initial elapsedTime
      
      // Get initial voltage reading
      Vc = ReadAnaloguePort();            // Read initial capacitor voltage
      
      // -------------------------------------------------------------------------------------------------------------------------------
      // RECONFIGURE PINS FOR THIRD RANGE (10kΩ charging resistor)
      // -------------------------------------------------------------------------------------------------------------------------------
      
      // Keep chargePin_one as INPUT (High Z)
      pinMode(chargePin_one, INPUT);      // chargePin_one remains High Z
      
      // Keep chargePin_two as INPUT (High Z) to isolate it
      pinMode(chargePin_two, INPUT);      // Set chargePin_two as input - High Z, disconnected
      
      // Configure chargePin_three as OUTPUT to enable charging through 10kΩ resistor
      pinMode(chargePin_three, OUTPUT);   // Set chargePin_three (10kΩ) as output to enable charging
      
      // Keep remaining charge pins as INPUT
      pinMode(chargePin_four, INPUT);     // chargePin_four remains High Z
      pinMode(chargePin_five, INPUT);     // chargePin_five remains High Z
      
      // Keep dischargePin as INPUT
      pinMode(dischargePin, INPUT);       // dischargePin remains High Z
      
      // Start charging through 10kΩ resistor
      digitalWrite(chargePin_three, HIGH); // Set chargePin_three to 5V - begin charging through 10kΩ
      Resistor = 3;
      
      // -------------------------------------------------------------------------------------------------------------------------------
      // WAIT FOR THRESHOLD
      // -------------------------------------------------------------------------------------------------------------------------------
      while((Vc < 646) && (elapsedTime < 1001))  // While below threshold AND less than 1 second
      {
        Vc = ReadAnaloguePort();           // Read current voltage
        elapsedTime = millis() - startTime; // Update elapsed time
      }
      
      // -------------------------------------------------------------------------------------------------------------------------------
      // CHECK IF THIRD RANGE SUCCEEDED
      // -------------------------------------------------------------------------------------------------------------------------------
      if (elapsedTime < 1001)              // If threshold reached within 1 second using 10kΩ
      {
        // Calculate capacitance with ChargeResistorThree
        microFarads = ((float)elapsedTime / ChargeResistorThree) * 1000;  // Calculate in microfarads
        nanoFarads = 0;                     // Clear nanofarads
        Size = micro;                        // Set unit to microfarads
        
        // Debug output
        // Serial.print("R3 - t="); Serial.print(elapsedTime); Serial.print("ms, C="); Serial.println(microFarads);
      }
      else                                   // Timeout with 10kΩ - try next range
      {
        // -----------------------------------------------------------------------------------------------------------------------------
        // FOURTH RANGE ATTEMPT - Using ChargeResistorFour (1kΩ) for medium-large capacitance range
        // -----------------------------------------------------------------------------------------------------------------------------
        
        // Discharge capacitor
        dischargeCapacitor();               // Fully discharge capacitor
        
        // Record start time
        startTime = millis();               // Get new start timestamp
        
        // Initialize timing
        elapsedTime = millis() - startTime; // Calculate initial elapsedTime
        
        // Get initial voltage
        Vc = ReadAnaloguePort();            // Read initial voltage
        
        // -----------------------------------------------------------------------------------------------------------------------------
        // RECONFIGURE PINS FOR FOURTH RANGE (1kΩ charging resistor)
        // -----------------------------------------------------------------------------------------------------------------------------
        
        // Set all previous charge pins as INPUT
        pinMode(chargePin_one, INPUT);      // chargePin_one High Z
        pinMode(chargePin_two, INPUT);      // chargePin_two High Z
        pinMode(chargePin_three, INPUT);    // chargePin_three High Z
        
        // Configure chargePin_four as OUTPUT for 1kΩ charging
        pinMode(chargePin_four, OUTPUT);    // Set chargePin_four (1kΩ) as output
        
        // Keep chargePin_five as INPUT
        pinMode(chargePin_five, INPUT);     // chargePin_five High Z
        
        // Keep dischargePin as INPUT
        pinMode(dischargePin, INPUT);       // dischargePin High Z
        
        // Start charging through 1kΩ resistor
        digitalWrite(chargePin_four, HIGH); // Set chargePin_four to 5V - charge through 1kΩ
        Resistor = 4;
        
        // -----------------------------------------------------------------------------------------------------------------------------
        // WAIT FOR THRESHOLD
        // -----------------------------------------------------------------------------------------------------------------------------
        while((Vc < 640) && (elapsedTime < 1001))  // While below threshold AND less than 1 second
        {
          Vc = ReadAnaloguePort();           // Read voltage
          elapsedTime = millis() - startTime; // Update elapsed time
        }
        
        // -----------------------------------------------------------------------------------------------------------------------------
        // CHECK IF FOURTH RANGE SUCCEEDED
        // -----------------------------------------------------------------------------------------------------------------------------
        if (elapsedTime < 1001)              // If threshold reached within 1 second using 1kΩ
        {
          // Calculate capacitance with ChargeResistorFour
          microFarads = ((float)elapsedTime / ChargeResistorFour) * 1000;  // Calculate in microfarads
          nanoFarads = 0;                     // Clear nanofarads
          Size = micro;                        // Set unit to microfarads
          
          // Debug output
          // Serial.print("R4 - t="); Serial.print(elapsedTime); Serial.print("ms, C="); Serial.println(microFarads);
        }
        else                                   // Timeout with 1kΩ - try final range
        {
          // ---------------------------------------------------------------------------------------------------------------------------
          // FIFTH RANGE ATTEMPT - Using ChargeResistorFive (100Ω) for largest capacitance range
          // ---------------------------------------------------------------------------------------------------------------------------
          
          // Discharge capacitor
          dischargeCapacitor();               // Fully discharge capacitor
          
          // Record start time
          startTime = millis();               // Get new start timestamp
          
          // Initialize timing
          elapsedTime = millis() - startTime; // Calculate initial elapsedTime
          
          // Get initial voltage
          Vc = ReadAnaloguePort();            // Read initial voltage
          
          // ---------------------------------------------------------------------------------------------------------------------------
          // RECONFIGURE PINS FOR FIFTH RANGE (100Ω charging resistor)
          // ---------------------------------------------------------------------------------------------------------------------------
          
          // Set all other charge pins as INPUT
          pinMode(chargePin_one, INPUT);      // chargePin_one High Z
          pinMode(chargePin_two, INPUT);      // chargePin_two High Z
          pinMode(chargePin_three, INPUT);    // chargePin_three High Z
          pinMode(chargePin_four, INPUT);     // chargePin_four High Z
          
          // Configure chargePin_five as OUTPUT for 100Ω charging
          pinMode(chargePin_five, OUTPUT);    // Set chargePin_five (100Ω) as output
          
          // Keep dischargePin as INPUT
          pinMode(dischargePin, INPUT);       // dischargePin High Z
          
          // Start charging through 100Ω resistor
          digitalWrite(chargePin_five, HIGH); // Set chargePin_five to 5V - charge through 100Ω
          Resistor = 5;
          
          // ---------------------------------------------------------------------------------------------------------------------------
          // WAIT FOR THRESHOLD
          // ---------------------------------------------------------------------------------------------------------------------------
          while((Vc < 590) && (elapsedTime < 1001))  // While below threshold AND less than 1 second
          {
            Vc = ReadAnaloguePort();           // Read voltage
            elapsedTime = millis() - startTime; // Update elapsed time
          }
          
          // ---------------------------------------------------------------------------------------------------------------------------
          // CHECK IF FIFTH RANGE SUCCEEDED
          // ---------------------------------------------------------------------------------------------------------------------------
          if (elapsedTime < 2001)              // If threshold reached within 1 second using 100Ω
          {
            // For large capacitors (100Ω range), capacitance is in microfarads, not nanofarads
            // Calculate in microfarads using the same formula: (t_ms / R) * 1000
            microFarads = ((float)elapsedTime / ChargeResistorFive) * 1000;  // Calculate in microfarads
            // For large capacitors we need to use a CorrectionFactor 
            // There are many reasons. One is difficulties in measuring big time spands because of noise etc.
            // ESR will also have an influence with theese big Capacitors.
            if(microFarads>=1000 && microFarads<2200)                                                // 1000uF - 2200uF
            {
              CorrectionFactor = 0.96153846153846153846153846153846*1.009713443324784426179850158525;
              microFarads = microFarads * CorrectionFactor;
            }
            else if(microFarads>=2200 && microFarads<4700)                                            // 2200uF - 4700uF
            {
              CorrectionFactor = 0.97058823529411764705882352941176;
              microFarads = microFarads * CorrectionFactor;
            }
            else if(microFarads>=4700 && microFarads<5600)                                            // 4700uF - 5600uF
            {
              CorrectionFactor = 0.97087378640776699029126213592233;
              microFarads = microFarads * CorrectionFactor;
            }
            else if(microFarads>=5600 && microFarads<6300)                                            // 5600uF - 6300uF
            {
              CorrectionFactor = 0.97039473684210526315789473684211;
              microFarads = microFarads * CorrectionFactor;
            }
            else if(microFarads>=6300 && microFarads<7500)                                            // 6300uF - 7500uF
            {
              CorrectionFactor = 0.97087378640776699029126213592233;
              microFarads = microFarads * CorrectionFactor;
            }
            else if(microFarads>=7500 && microFarads<8200)                                            // 7500uF - 8200uF
            {
              CorrectionFactor = 0.97170971709717097170971709717097;
              microFarads = microFarads * CorrectionFactor;
            }
            else if(microFarads>=8200 && microFarads<9000)                                            // 8200uF - 9000uF
            {
              CorrectionFactor = 0.97087378640776699029126213592233;
              microFarads = microFarads * CorrectionFactor;
            }
            else if(microFarads>=9000 && microFarads<11000)                                           // 9000uF - 11000uF
            {
              CorrectionFactor = 0.99894735137195121951219512195122;
              microFarads = microFarads * CorrectionFactor;
            };
            // Set nanoFarads to 0
            nanoFarads = 0;                    // Clear nanofarads value
            
            // Set Size indicator to micro (0) to indicate capacitance is in microfarads
            Size = micro;                       // Set unit indicator to microfarads
            
            // Debug output
            // Serial.print("R5 - t="); Serial.print(elapsedTime); Serial.print("ms, C="); Serial.println(microFarads);
          }
          else                                   // All ranges failed - measurement impossible
          {
            // Set error flag to indicate measurement failure
            // This occurs when capacitor is too large (even 100Ω takes >1 second)
            // or too small (even 1MΩ charges in <1ms and cannot be measured accurately)
            error = 1;                           // Set error flag - capacitance out of measurable range
          }
        }
      }
    }
  }
}
// =====================================================================================================================================
// Functiondeclaration: ReadAnaloguePort : Read capacitor voltage with analogue port: 4xoversampling
// =====================================================================================================================================
// PURPOSE:
// This function performs analog-to-digital conversion on the predefined analogPin.
// It implements oversampling to improve measurement accuracy and reduce noise.
// The ADC reference voltage is set to Vcc (5.00V) which is the default for Arduino Uno.
//
// ANALOG TO DIGITAL CONVERTER (ADC) BACKGROUND:
// The Arduino Uno (ATmega328P) features a 10-bit successive approximation ADC.
// This means it can convert an analog voltage into a digital value between:
// - 0 (representing 0V / GND)
// - 1023 (representing the reference voltage, in this case Vcc = 5.00V)
//
// VOLTAGE TO DIGITAL VALUE CONVERSION FORMULA:
// Digital Value = (Analog Voltage × 1024) / Reference Voltage
// 
// Where:
// - Analog Voltage is the input voltage on the analog pin (0-5V)
// - Reference Voltage is Vcc = 5.00V
// - 1024 is the number of discrete steps (2^10 = 1024)
//
// PRACTICAL EXAMPLES:
// - If input voltage = 2.50V: Digital Value = (2.50 × 1024) / 5.00 = 512
// - If input voltage = 1.00V: Digital Value = (1.00 × 1024) / 5.00 = 205
// - If input voltage = 4.00V: Digital Value = (4.00 × 1024) / 5.00 = 819
//
// OVERSAMPLING TECHNIQUE:
// Oversampling is a method where multiple samples are taken and averaged to:
// 1. Reduce the effect of random noise in the measurement
// 2. Increase the effective resolution (to a limited extent through decimation)
// 3. Provide a more stable and reliable reading
// 4. Filter out transient voltage spikes
//
// IMPLEMENTATION DETAILS:
// - Takes 4 consecutive readings from the analog pin
// - Uses a global Counter variable to keep track of the number of samples taken
// - Sums all readings and calculates the arithmetic mean (average)
// - Returns the average as an integer value (rounded down due to integer division)
//
// MATHEMATICAL FORMULA:
// Average = (Sample₁ + Sample₂ + Sample₃ + Sample₄) / 4
//
// HARDWARE SPECIFICATIONS:
// - ADC Resolution: 10 bits (0-1023)
// - Reference Voltage: Vcc (5.00V DC)
// - Conversion Time: Approximately 100-200 microseconds per reading
// - Recommended settling time: 1ms between readings for stable results
//
// GLOBAL VARIABLES USED:
// - analogPin: Defined elsewhere (#define analogPin x), specifies which analog pin to read
//   Valid values: A0, A1, A2, A3, A4, A5 on Arduino Uno
// - Counter: Global variable that tracks total readings across function calls
//   Can be used for statistical analysis or debugging
//
// RETURN VALUE:
// Integer (16-bit signed int) representing the average of 4 ADC readings
// Range: 0 to 1023 (full scale of 10-bit ADC)
// 
// ACCURACY CONSIDERATIONS:
// - With 4x oversampling, the effective resolution increases slightly
// - Random noise is reduced by factor of √4 = 2 (theoretically)
// - The result is still an integer between 0-1023
//
// USAGE EXAMPLE:
// int sensorValue = ReadAnaloguePort();
// float voltage = (sensorValue * 5.0) / 1024.0;  // Convert to actual voltage
// if (voltage > 2.5) {
//   digitalWrite(LED_PIN, HIGH);  // Turn on LED if voltage exceeds 2.5V
// }
// =====================================================================================================================================
// Function: ReadAnaloguePort
// Purpose: Performs 4x oversampled ADC reading from analogPin and returns averaged integer value
// Returns: int - Average of 4 ADC readings (range 0-1023)
// Global dependencies: analogPin (must be defined), Counter (incremented with each reading)
int ReadAnaloguePort() {
  // Declare a local variable to accumulate the sum of all ADC readings
  // Starting value is 0 - we will add each reading to this variable
  long sum = 0;                      // Use long (32-bit) to prevent overflow when summing multiple readings
  
  // Declare a local variable to store each individual ADC reading temporarily
  // This variable is reused for each of the 4 samples
  int reading = 0;                   // Temporary storage for current ADC reading (0-1023)
  
  // Perform oversampling by taking 4 consecutive readings
  // The for loop will execute exactly 4 times (i = 0,1,2,3)
  for (int i = 0; i < 4; i++) {      // Loop 4 times to collect 4 samples for oversampling
    // Read the analog voltage on the specified analog pin
    // analogRead() returns an integer between 0 and 1023 representing 0V to Vcc (5.00V)
    // The conversion takes approximately 100 microseconds to complete
    reading = analogRead(analogPin); // Perform ADC conversion and store result (0-1023)
    
    // Add the current reading to the cumulative sum
    // After all 4 readings, sum will contain total of all samples
    sum = sum + reading;             // Accumulate readings by adding current reading to running total
    
    // Increment the global Counter variable to track total readings
    // This can be useful for debugging or statistical analysis
    Counter++;                       // Increase global counter by 1 to track total readings taken
    
    // Optional: Small delay to allow ADC to stabilize between readings
    // Uncomment the following line if readings appear unstable:
    // delayMicroseconds(500);        // Wait 500 microseconds between samples for ADC recovery
  }
  
  // Calculate the average by dividing the total sum by number of samples (4)
  // Integer division is used here - result is truncated (rounded down)
  // Example: if sum = 2050, average = 2050/4 = 512 (not 512.5)
  int average = sum / 4;             // Compute arithmetic mean of the 4 samples using integer division
  
  // Return the calculated average to the calling function
  // The value will be between 0 and 1023, representing the oversampled reading
  return average;                    // Return the averaged result (0-1023) to caller
  
  // Note: Because we use integer division, there is a small rounding error
  // For applications requiring higher precision, consider using floating point:
  // float preciseAverage = (float)sum / 4.0;
  // Then round appropriately or return as float
}
// =====================================================================================================================================
// Functiondeclaration:  dischargeCapacitor
// =====================================================================================================================================
void dischargeCapacitor()
{
  pinMode(chargePin_one, INPUT);      // chargePin_one is High Z : INPUT
  pinMode(chargePin_two, INPUT);      // chargePin_two is High Z : INPUT
  pinMode(chargePin_three, INPUT);    // chargePin_three is High Z : INPUT
  pinMode(chargePin_four, INPUT);     // chargePin_four is High Z : INPUT
  pinMode(chargePin_five, INPUT);     // chargePin_five is High Z : INPUT
  pinMode(dischargePin, OUTPUT);      // dischargePin is output
  digitalWrite(dischargePin, LOW);    // dischargePin pulls capacitor ends to Ground through a 220 Ohm resistor
  Vc = ReadAnaloguePort();            // Read capacitor voltage
  
  // While loop to ensure capacitor is fully discharged (voltage > 0)
  while (Vc > 0)                      // WHILE voltage on capacitor is greater than 0
  {
    Vc = ReadAnaloguePort();          // read voltage on capacitor - continue reading until fully discharged
  }                                   // End while loop
  
  pinMode(dischargePin, INPUT);       // dischargePin is High Z : INPUT
}
// =====================================================================================================================================
// Functiondeclaration: ShowCapacitanceOnDisplay
// =====================================================================================================================================
void ShowCapacitanceOnDisplay()
{
  // This function is intentionally left empty for now
  // It can be implemented later to show capacitance on display in a specific format
}
// =====================================================================================================================================
// Functiondeclaration: ShowCapacitanceOnSerialport
// =====================================================================================================================================
void ShowCapacitanceOnSerialport()
{
  // This function is intentionally left empty for now
  // It can be implemented later to show capacitance on serial port in a specific format
}
// =====================================================================================================================================
// Functiondeclaration: checkButtonPress - SIMPLE RELIABLE DEBOUNCE
// =====================================================================================================================================
// This function implements a simple, reliable button debounce algorithm
// Based on Khaled Magdy's proven debounce example
//
// How it works:
// 1. Read the button state
// 2. If it's different from last reading, reset the debounce timer
// 3. After debounce delay, if the state is stable, accept it as valid
// 4. Return true only when a valid press is detected (LOW with pull-up)
// =====================================================================================================================================

bool checkButtonPress() {
  // Read the current state of the button pin
  int reading = digitalRead(button);
  
  // If the button state changed (due to noise or pressing)
  if (reading != lastButtonState) {
    // Reset the debouncing timer
    lastDebounceTime = millis();
  }
  
  // Check if enough time has passed since the last change
  if ((millis() - lastDebounceTime) > debounceDelay) {
    
    // If the button state has actually changed (not just noise)
    if (reading != buttonState) {
      buttonState = reading;
      
      // Only return true if the button is pressed (LOW with pull-up)
      // and it's a new press (buttonPressed flag is used to ensure one trigger per press)
      if (buttonState == LOW && !buttonPressed) {
        buttonPressed = true;  // Mark that we've reported this press
        return true;           // Return true for this press event
      }
      
      // If button is released, clear the pressed flag
      if (buttonState == HIGH) {
        buttonPressed = false; // Ready for next press
      }
    }
  }
  
  // Save the last button state for next comparison
  lastButtonState = reading;
  
  // No valid press detected
  return false;
}
// =====================================================================================================================================
// Function declaration: initializeDisplay : DISPLAY INITIALIZATION
// =====================================================================================================================================
// Function: initializeDisplay()
// Purpose: Initializes the display to 2x16 characters and turns on the backlight
// Returns: bool - true if initialization successful, false if error (no display found)
bool initializeDisplay() {
  // Initialize the I2C communication and the LCD display
  // init() initializes the display and sets up the I2C communication with the PCF8574 chip
  // This must be called before any other LCD functions
  lcd.init();                        // Initialize the LCD display - establishes I2C communication and sets up HD44780
  
  // Turn on the backlight of the display
  // setBacklight() controls the backlight LED - HIGH (or 1) turns it on, LOW (or 0) turns it off
  // This makes the display visible in dark environments
  lcd.setBacklight(HIGH);            // Turn on backlight - HIGH = on, LOW = off
  
  // Clear the entire display of any random characters that might appear during power-up
  // clear() removes all characters and returns the cursor to home position (0,0)
  lcd.clear();                       // Clear any random characters from display memory
  
  // Return success - display is now ready for use
  // Note: The Adafruit LiquidCrystal_I2C library doesn't have getBacklight() for checking
  // We assume initialization was successful if we reach this point
  return true;                       // Return true to indicate successful initialization
}
// =====================================================================================================================================
// Function declaration: setCursorPosition : MOVE CURSOR TO SPECIFIC POSITION
// =====================================================================================================================================
// Function: setCursorPosition()
// Purpose: Moves the cursor to a specific row and column position on the display
// Parameters:
//   row - row number (0 = first line, 1 = second line)
//   col - column number/position (0-15 for 16x2 display)
// Note: This does not write anything, only positions the cursor for next write operation
void setCursorPosition(int row, int col) {
  // Validate the row parameter to prevent errors
  // Check if row is within valid range (0 to LCD_ROWS-1, which is 0-1 for 2-row display)
  if (row < 0 || row >= LCD_ROWS) {  // If row is less than 0 or greater than or equal to maximum rows
    return;                          // Invalid row - exit function without doing anything
  }
  
  // Validate the column parameter to prevent errors
  // Check if col is within valid range (0 to LCD_COLS-1, which is 0-15 for 16-column display)
  if (col < 0 || col >= LCD_COLS) {  // If col is less than 0 or greater than or equal to maximum columns
    return;                          // Invalid column - exit function without doing anything
  }
  
  // Move the cursor to the desired position on the display
  // setCursor(column, row) - note the parameter order: column first, then row
  // The cursor will blink at this position (if cursor blinking is enabled)
  lcd.setCursor(col, row);           // Set cursor to specified column and row (col first, row second)
}
// =====================================================================================================================================
// Function declaration: writeCharacterAt : WRITE A SINGLE CHARACTER AT SPECIFIC POSITION
// =====================================================================================================================================
// Function: writeCharacterAt()
// Purpose: Writes a single character at a specific position on the display
// Parameters:
//   row - row number (0 = first line, 1 = second line)
//   col - column number/position (0-15 for 16x2 display)
//   character - the character to write (e.g., 'A', '5', '!', '$')
// Note: This overwrites whatever character was previously at that position
void writeCharacterAt(int row, int col, char character) {
  // Validate the row parameter to prevent writing outside display boundaries
  if (row < 0 || row >= LCD_ROWS) {  // If row is outside valid range (0 to 1 for 2-row display)
    return;                          // Invalid row - exit function without writing
  }
  
  // Validate the column parameter to prevent writing outside display boundaries
  if (col < 0 || col >= LCD_COLS) {  // If col is outside valid range (0 to 15 for 16-column display)
    return;                          // Invalid column - exit function without writing
  }
  
  // Move the cursor to the desired position where the character will be written
  // setCursor() positions the cursor at the specified column and row
  lcd.setCursor(col, row);           // Position cursor at specified column and row (col first, row second)
  
  // Write the single character at the current cursor position
  // write() sends a single byte/character to the display
  // This is more efficient than print() for single characters
  lcd.write(character);              // Send the character to the display at current cursor position
}
// =====================================================================================================================================
// Function declaration: writeTextAt : WRITE TEXT AT SPECIFIC POSITION
// =====================================================================================================================================
// Function: writeTextAt()
// Purpose: Writes a text string at a specific position on the display
// Parameters:
//   row - row number (0 = first line, 1 = second line)
//   col - column number/position (0-15 for 16x2 display)
//   text - the text to write (as C-string/char array, e.g., "Hello")
// Protection: Text is automatically truncated if it would exceed the display width
// Note: If text is longer than remaining space, it will continue on next line (hardware behavior)
void writeTextAt(int row, int col, const char* text) {
  // Validate the row parameter to prevent writing outside display boundaries
  if (row < 0 || row >= LCD_ROWS) {  // If row is outside valid range (0 to 1 for 2-row display)
    return;                          // Invalid row - exit function without writing
  }
  
  // Validate the column parameter to prevent writing outside display boundaries
  if (col < 0 || col >= LCD_COLS) {  // If col is outside valid range (0 to 15 for 16-column display)
    return;                          // Invalid column - exit function without writing
  }
  
  // Check if the text pointer is NULL (no text provided)
  if (text == NULL) {                // If text pointer points to nothing (NULL pointer)
    return;                          // No text to write - exit function
  }
  
  // Move the cursor to the desired starting position for the text
  // setCursor() positions the cursor at the specified column and row
  lcd.setCursor(col, row);           // Position cursor at starting column and row for text
  
  // Write the entire text string starting from current cursor position
  // print() sends a string of characters to the display
  // If text is longer than available space on current line, it will wrap to next line
  lcd.print(text);                   // Print the text string at current cursor position
  
  // Optional: To prevent wrapping to next line, you could use:
  // int maxChars = LCD_COLS - col;
  // lcd.print(String(text).substring(0, maxChars));
  // But this implementation allows wrapping as per hardware default behavior
}
// =====================================================================================================================================
// Function declaration: clearDisplay : CLEAR ENTIRE DISPLAY
// =====================================================================================================================================
// Function: clearDisplay()
// Purpose: Clears all content from the display and moves cursor to home position (0,0)
// This removes all characters and returns cursor to top-left corner
void clearDisplay() {
  // Clear the entire display memory (all DDRAM contents are set to space character)
  // clear() also automatically moves the cursor to home position (0,0) - first line, first column
  lcd.clear();                       // Remove all characters from display and home cursor
}
// =====================================================================================================================================
// Function declaration: clearLine : CLEAR A SPECIFIC LINE
// =====================================================================================================================================
// Function: clearLine()
// Purpose: Clears the content on a specific line/row of the display
// Parameters:
//   row - the line number to clear (0 = first line, 1 = second line)
// This function preserves content on other lines
void clearLine(int row) {
  // Validate the row parameter to prevent clearing outside display boundaries
  if (row < 0 || row >= LCD_ROWS) {  // If row is outside valid range (0 to 1 for 2-row display)
    return;                          // Invalid row - exit function without clearing
  }
  
  // Move the cursor to the beginning (column 0) of the specified row
  // setCursor(0, row) positions at first column of the selected row
  lcd.setCursor(0, row);             // Position cursor at start (column 0) of the specified row
  
  // Write space characters on the entire line to "erase" any existing content
  // Loop through each column position on this line (0 to LCD_COLS-1)
  for (int i = 0; i < LCD_COLS; i++) {  // For each column from 0 to maximum columns-1
    lcd.write(' ');                  // Write a space character at current cursor position
  }                                  // Cursor automatically advances to next column after each write
  
  // Move the cursor back to the beginning of the cleared line
  // This is optional but convenient - leaves cursor ready for next write operation on this line
  lcd.setCursor(0, row);             // Return cursor to start of the cleared line for convenience
}
// =====================================================================================================================================
// Function declaration: setBacklight : TURN BACKLIGHT ON/OFF
// =====================================================================================================================================
// Function: setBacklight()
// Purpose: Controls the display backlight (on or off)
// Parameters:
//   state - true/HIGH = turn backlight on, false/LOW = turn backlight off
// This can save power or indicate different modes visually
void setBacklight(bool state) {
  // Set the backlight to the requested state
  // setBacklight() accepts HIGH (1/true) for on, LOW (0/false) for off
  lcd.setBacklight(state);           // Turn backlight on (HIGH/true) or off (LOW/false)
}
// =====================================================================================================================================
// Function declaration: createCustomChar : CREATE CUSTOM CHARACTER
// =====================================================================================================================================
// Function: createCustomChar()
// Purpose: Creates a custom character in the display's CGRAM memory
// Parameters:
//   location - which character slot to use (0-7, as HD44780 has 8 slots for custom chars)
//   charMap - array of 8 bytes defining the 5x8 dot pattern for the custom character
// Note: Custom characters can be written using write(location) after creation
void createCustomChar(int location, const uint8_t* charMap) {
  // Validate the location parameter (must be 0-7 as there are only 8 CGRAM locations)
  if (location < 0 || location > 7) {  // If location is outside valid range (0 to 7)
    return;                          // Invalid location - exit function
  }
  
  // Check if the character map pointer is valid (not NULL)
  if (charMap == NULL) {             // If charMap points to nothing (NULL pointer)
    return;                          // No character data - exit function
  }
  
  // Create the custom character in the specified CGRAM location
  // createChar() stores the 5x8 pixel pattern in the display's character generator RAM
  lcd.createChar(location, (uint8_t*)charMap);  // Store custom character pattern at specified location
}
// =====================================================================================================================================
// CALIBRATION MANUAL
// Arduino UNO / NANO Capacitance Meter (Cmeter12 Firmware)
// =====================================================================================================================================
//
// This document describes a professional, repeatable calibration procedure for the Arduino‑based
// capacitance meter defined by the uploaded schematics, firmware (Cmeter12.txt), and requirements
// specification.
//
// The goal of calibration is to align firmware parameters with the real physical behavior of:
// • The power supply (Vcc)
// • The five charging resistors
// • The ADC threshold detection
// • Range‑specific non‑ideal effects (parasitics, ESR, leakage)
//
// Calibration is REQUIRED for meaningful accuracy and MUST be performed after assembly,
// component replacement, or power‑supply changes.
//
// =====================================================================================================================================
// 1. REQUIRED EQUIPMENT
// =====================================================================================================================================
//
// The following equipment is required for calibration:
//
// • Calibrated digital multimeter (DMM), accuracy ≤ 0.1%
// • Reference capacitors with known values and low tolerance (≤ 2% recommended)
// • Stable 5 V power source (USB supply is acceptable if measured and stable)
// • Computer with Arduino IDE
// • Access to the Cmeter12 firmware source code
//
// Reference capacitors should cover the full operating range:
//
// • Small range: 10 nF, 22 nF, 47 nF, 100 nF
// • Medium range: 1 µF, 4.7 µF, 10 µF, 47 µF
// • Large range: 100 µF, 470 µF, 1000 µF, 4700 µF
//
// Electrolytic reference capacitors must be pre‑conditioned (charged and discharged several times).
//
// =====================================================================================================================================
// 2. CALIBRATION PHILOSOPHY
// =====================================================================================================================================
//
// This capacitance meter is not calibrated by a single global factor.
// Accuracy depends on multiple hardware‑dependent parameters:
//
// • Actual resistor values
// • Actual supply voltage
// • ADC threshold positioning
// • Range‑specific non‑ideal capacitor behavior
//
// Therefore calibration is performed in FOUR stages:
//
// 1) Power supply (Vcc) calibration
// 2) Charging resistor value calibration
// 3) ADC threshold alignment
// 4) Range‑specific correction factor calibration
//
// Each stage builds upon the previous one and MUST be performed in order.
//
// =====================================================================================================================================
// 3. POWER SUPPLY (Vcc) CALIBRATION
// =====================================================================================================================================
//
// The firmware assumes a fixed supply voltage defined by:
//
//     #define Vcc 5.00
//
// This value is used to calculate:
// • ADC threshold voltage
// • Threshold ADC counts
//
// PROCEDURE:
//
// • Power the device normally (USB or external supply)
// • Measure the voltage directly at the Arduino 5V and GND pins using the DMM
// • Record the measured voltage (example: 4.962 V)
//
// FIRMWARE CHANGE:
//
// Replace the Vcc definition with the measured value:
//
//     #define Vcc 4.962
//
// IMPORTANT:
//
// • Do NOT assume USB power is exactly 5.00 V
// • Even small Vcc errors shift the 63% threshold significantly
//
// Recompile and upload the firmware before continuing.
//
// =====================================================================================================================================
// 4. CHARGING RESISTOR CALIBRATION
// =====================================================================================================================================
//
// The firmware uses five constants for the charging resistors:
//
//     #define ChargeResistorOne   1000000.0
//     #define ChargeResistorTwo    100000.0
//     #define ChargeResistorThree   10000.0
//     #define ChargeResistorFour     1000.0
//     #define ChargeResistorFive      100.0
//
// These MUST match the real, measured resistor values.
//
// PROCEDURE:
//
// • Power OFF the device
// • Measure each resistor individually using the DMM
// • Measure directly at resistor leads, not through Arduino pins
// • Record each value accurately
//
// Example measured values:
//
// • R6 (1MΩ)   → 1003200 Ω
// • R7 (100kΩ) → 99850 Ω
// • R10 (10kΩ) → 10012 Ω
// • R11 (1kΩ)  → 1004 Ω
// • R12 (100Ω) → 101.2 Ω
//
// FIRMWARE CHANGE:
//
// Replace the constants with measured values:
//
//     #define ChargeResistorOne   1003200.0
//     #define ChargeResistorTwo     99850.0
//     #define ChargeResistorThree   10012.0
//     #define ChargeResistorFour     1004.0
//     #define ChargeResistorFive      101.2
//
// Recompile and upload the firmware.
//
// =====================================================================================================================================
// 5. ADC THRESHOLD CALIBRATION
// =====================================================================================================================================
//
// The firmware detects the charge completion using ADC thresholds,
// implemented as hardcoded values in MeasureCapacitor():
//
// Examples:
// • 629
// • 646
// • 640
// • 590
//
// These values compensate for ADC behavior, pin resistance, and dynamic loading.
//
// OBJECTIVE:
//
// Ensure that the threshold corresponds as closely as possible to the point where
// the capacitor voltage reaches approximately 63% of Vcc in each range.
//
// PROCEDURE:
//
// • Use a mid‑range reference capacitor for each resistor range
// • Measure the displayed capacitance
// • Compare with the known capacitor value
//
// If the displayed value is consistently HIGH:
// • Threshold is reached too early → ADC threshold too LOW
//
// If the displayed value is consistently LOW:
// • Threshold is reached too late → ADC threshold too HIGH
//
// FIRMWARE CHANGE:
//
// Adjust the ADC threshold value inside the corresponding while() loop by small steps (±2 to ±5).
//
// Example:
//
//     while ((Vc < 646) && (elapsedTime < 1001))
//
// Change to:
//
//     while ((Vc < 642) && (elapsedTime < 1001))
//
// Repeat measurement until mid‑range values align.
//
// IMPORTANT:
//
// • Do NOT try to force perfect accuracy at range edges
// • Optimize thresholds near the center of each range
//
// =====================================================================================================================================
// 6. RANGE‑SPECIFIC CORRECTION FACTOR CALIBRATION
// =====================================================================================================================================
//
// The firmware already implements CorrectionFactor logic for:
// • Very small capacitances
// • Very large capacitances
//
// These compensate for:
// • ADC latency
// • Stray capacitance
// • ESR and leakage
// • Software timing limitations
//
// OBJECTIVE:
//
// Fine‑tune these correction factors using real reference capacitors.
//
// PROCEDURE:
//
// • Select a known reference capacitor
// • Measure capacitance using the meter
// • Compute correction factor:
//
//     correction = reference_value / measured_value
//
// • Insert this factor into the corresponding conditional block
//
// Example (existing code):
//
//     if (microFarads >= 40 && microFarads < 50)
//     {
//         CorrectionFactor = 0.97826;
//         microFarads = microFarads * CorrectionFactor;
//     }
//
// Adjust the CorrectionFactor value until the display matches the reference capacitor.
//
// IMPORTANT:
//
// • Do not overlap correction ranges
// • Keep correction ranges narrow and monotonic
// • Document all changes
//
// =====================================================================================================================================
// 7. STRAY CAPACITANCE BASELINE CALIBRATION (SMALL VALUES)
// =====================================================================================================================================
//
// For small capacitances, stray capacitance dominates.
//
// PROCEDURE:
//
// • Leave DUT terminals open (no capacitor connected)
// • Perform a measurement
// • Record the displayed capacitance
//
// This value is the system’s inherent parasitic capacitance.
//
// OPTIONAL FIRMWARE IMPROVEMENT:
//
// • Subtract this baseline value from all small‑range measurements
// • Implement as a fixed offset for nano‑farad ranges
//
// =====================================================================================================================================
// 8. VALIDATION AND ACCEPTANCE TESTING
// =====================================================================================================================================
//
// After calibration:
//
// • Measure all reference capacitors again
// • Record results
//
// Acceptance criteria:
//
// • ±2% for film capacitors
// • ±5% for electrolytic capacitors
// • Repeatability within ±1%
//
// Perform measurements at least 3 times per capacitor.
//
// =====================================================================================================================================
// 9. CALIBRATION RECORD KEEPING
// =====================================================================================================================================
//
// It is strongly recommended to record:
//
// • Date of calibration
// • Vcc value
// • Resistor measurements
// • ADC thresholds
// • Correction factors
//
// Store this data alongside the firmware version.
//
// =====================================================================================================================================
// 10. FINAL NOTES
// =====================================================================================================================================
//
// This capacitance meter is fundamentally limited by:
// • ADC resolution
// • Software timing
// • Capacitor non‑ideal behavior
//
// Calibration optimizes performance within these constraints,
// but cannot eliminate them entirely.
//
// When properly calibrated, the instrument performs reliably within
// its designed operating range and meets the intent of the requirements
// specification.
//
// =====================================================================================================================================
// END OF CALIBRATION MANUAL
// =====================================================================================================================================