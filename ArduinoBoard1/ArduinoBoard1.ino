//Configuration Defines

#define FIRMWARE_MAJOR_VERSION 0
#define FIRMWARE_MINOR_VERSION 0
#define FIRMWARE_BUILD_NUMBER 0

//Configuration Defines
#define BOARD_ID 18
#define BOARD_TYPE BOARDTYPE_ARDUINOMEGA
#define PRINT_DEBUG_LINES 1
#define SHIELD1_TYPE SHIELDTYPE_NONE
#define SHIELD2_TYPE SHIELDTYPE_NONE
#define SHIELD3_TYPE SHIELDTYPE_NONE
#define SHIELD4_TYPE SHIELDTYPE_NONE

#define LEDS_PER_STRIP 8
#define LED_STRIPS 4

//Pin Definitions
#define LEDSTRIP_PIN 47

#include "Arduino.h"
#include "spimessage.h"
#include "Definitions.h"

#include <Wire.h>
#include <avr/wdt.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif


//Defines for SPI Comm between Raspberry Pi and Arduino Board;
unsigned char outputBuffer_AB14[13];
unsigned char outputBuffer_AB19[13];
unsigned char outputBuffer_AB20[13];
bool run_spi_handler = false;
byte current_command = 0;
int outputBuffer_index = 0;
int received_command = 0;
int message_ready_to_send = 0;
int receive_index = 0;
int message_index = 0;
byte marker = 0;
unsigned char dat;
int compute_checksum(unsigned char * outputbuffer);



//Configuration defines for individual Shield Types


//Debugging Defines
#define PRINT_RECV_BUFFER 1


void run_veryfastrate_code(); //1000 Hz
void run_fastrate_code(); //100 Hz
void run_mediumrate_code(); //10 Hz
void run_slowrate_code(); //1 Hz
void run_veryslowrate_code(); //0.1 Hz



unsigned int armed_command = ROVERCOMMAND_DISARM;
unsigned int armed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
unsigned char recv_buffer[32];
int passed_checksum_counter = 0;
int failed_checksum_counter = 0;

//Board Definitions

#if(BOARD_TYPE == BOARDTYPE_ARDUINOMEGA)
  #define LED 13
#endif

//Hardware Definitions
#define LED_OFF 0
#define LED_RED 1
#define LED_GREEN 2
#define LED_BLUE 3
#define LED_YELLOW 4

//Hardware Function Definitions
Adafruit_NeoPixel led_strips = Adafruit_NeoPixel(LEDS_PER_STRIP*LED_STRIPS, LEDSTRIP_PIN, NEO_RGBW  + NEO_KHZ800 ); //FOR PN: 600004
bool init_ledstrips();
void set_stripcolor(int color);
void run_cylonmode(int position);

int led_position = 0;
int led_direction = 1;

int veryfastrate_counter = 0;
int fastrate_counter = 0;
int mediumrate_counter = 0;
int slowrate_counter = 0;
int veryslowrate_counter = 0;

unsigned long loop_counter = 0;
unsigned long send_mode_counter = 0;
unsigned long recv_mode_counter = 0;
unsigned long send_configure_shield_counter = 0;
unsigned long recv_configure_shield_counter = 0;
unsigned long send_configure_dio_counter = 0;
unsigned long recv_configure_dio_counter = 0;
unsigned long send_set_dio_counter = 0;
unsigned long recv_set_dio_counter = 0;
unsigned long send_set_dio_defaultvalue_counter = 0;
unsigned long recv_set_dio_defaultvalue_counter = 0;
unsigned long send_armcommand_counter = 0;
unsigned long recv_armcommand_counter = 0;
unsigned long send_pps_counter = 0;
unsigned long recv_pps_counter = 0;
unsigned long time_since_last_rx = 0;
unsigned long level_debug_counter = 0;
unsigned long level_info_counter = 0;
unsigned long level_notice_counter = 0;
unsigned long level_warn_counter = 0;
unsigned long level_error_counter = 0;
unsigned long level_fatal_counter = 0;
byte transmit_testcounter = 0;
int comm_established_once = 0;



int temp_counter = 1000;
bool reverse = false;
int shield_count = -1;
int new_pps = 0;
int pps_received = 0;

//Function Prototypes for individual shields



void init_shields();
String map_level_tostring(int v);
String map_component_tostring(int v);
String map_diagnostictype_tostring(int v);
String map_message_tostring(int v);

int process_AB14_Query();
int process_AB19_Query();
int process_AB20_Query();


void(*resetFunc)(void) = 0;

//Message processing functions.  This should be as fast as possible
int process_AB14_Query()
{
  int msg_length;
  encode_TestMessageCounterSPI(outputBuffer_AB14,&msg_length,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter--,
    transmit_testcounter--,
    transmit_testcounter--);
}
int process_AB19_Query()
{
}
int process_AB20_Query()
{
}
void init_shields()
{
 
}
void setup() 
{
  wdt_disable();
  wdt_enable(WDTO_1S);
  init_ledstrips();
  //Setup pins

  pinMode(SCK, INPUT);
  pinMode(MOSI,INPUT);
  pinMode(SS,INPUT);
  pinMode(MISO, OUTPUT);
  SPCR = (1 << SPE);
  SPCR |= _BV(SPE);
  Serial.begin(115200);
  //Serial.setTimeout(1000);
  while(Serial.read() >= 0);
  Serial.flush();
  #if(BOARD_TYPE == BOARDTYPE_ARDUINOMEGA)
  {
    Serial.println("ArduinoBoard Booting");
    Serial.print("FW Major Version: ");
    Serial.println(FIRMWARE_MAJOR_VERSION,DEC);
    Serial.print("FW Minor Version: ");
    Serial.println(FIRMWARE_MINOR_VERSION,DEC);
    Serial.print("FW Build Number: ");
    Serial.println(FIRMWARE_BUILD_NUMBER,DEC);
  }
  #endif
  delay(500);
  wdt_reset();
  
  wdt_reset();
  wdt_reset();
  
  #if(BOARD_TYPE == BOARDTYPE_ARDUINOMEGA)
  {
    Serial1.println("Board Executing");
  }
  #endif
}

boolean SSlast = LOW;         // SS last flag.
boolean ledState = HIGH;  
byte SPItransfer(byte value) {
  SPDR = value;
  while(!(SPSR & (1<<SPIF)));
  delay(1);
  return SPDR;
}
void loop() 
{ 
  wdt_reset();
  spiHandler();
  
  
}

void run_veryfastrate_code() //1000 Hz
{
  
}
void run_fastrate_code() //100 Hz
{
  if(led_direction == 1) { led_position++; }
  if(led_direction == 0) { led_position--; }
  if(led_position == 0) { led_direction = 1; }
  if(led_position == ((LEDS_PER_STRIP*LED_STRIPS)-1)) { led_direction = 0; }
  
  //Serial.println(position);
  run_cylonmode(LED_RED,led_position);
  //delay(75);
      
}
void run_mediumrate_code() //10 Hz
{

}
void run_slowrate_code() //1 Hz
{  
  Serial.println("Running");
}
void run_veryslowrate_code() //0.1 Hz
{
  
  #if BOARD_TYPE == BOARDTYPE_ARDUINOMEGA
  {
    #if PRINT_DEBUG_LINES == 1
    
    #endif
  }
  #endif
}
boolean in_message = false;
void spiHandler()
{
   // Slave Enabled?
  if (!digitalRead(SS)) {
    // Yes, first time?
    if (SSlast != LOW) {
      // Yes, take MISO pin.
      pinMode(MISO, OUTPUT);
      Serial.println("***Slave Enabled.");
      // Write -1 slave response code and receive master command code
      byte rx = SPItransfer('a');
      Serial.println("Initial -1 slave response code sent");
      Serial.println("rx:" + String(rx) + ".");
      // cmdBtn?
      if ((marker == 0) && (rx == 0xAB))
      {
        marker++;
        // Acknowledge cmdBtn.
        byte rx = SPItransfer('a');
        Serial.println("cmdBtn Acknowledged.");
        Serial.println("rx:" + String(rx) + ".");
        // Toggle LED State
        ledState = !ledState;
        //digitalWrite(led, ledState);
      }
      else if((marker == 1) && (rx == SPI_TestMessageCounter_ID))
      {
        marker++;
        byte rx = SPItransfer(SPI_TestMessageCounter_ID);
        process_AB14_Query();
        Serial.println("cmdLEDState Acknowledged.");
        Serial.println("rx:" + String(rx) + ".");
        rx = SPItransfer(rx);
        Serial.println("ledState:" + String(ledState) + " Sent.");
        Serial.println("rx:" + String(rx) + "."); 
        in_message = true;       
      }
      else if(in_message == true)
      {
        marker++;
        // Unrecognized command.
        byte rx = SPItransfer(outputBuffer_AB14[outputBuffer_index]);
        outputBuffer_index++;
        
        
      }
      if(marker == 13)
      { 
        in_message = false;
        marker = 0; 
        outputBuffer_index = 0;
        }
      // Update SSlast.
      SSlast = LOW;
    }
  }
  else {
    // No, first time?
    if (SSlast != HIGH) {
      // Yes, release MISO pin.
      pinMode(MISO, INPUT);
      Serial.println("Slave Disabled.");
      // Update SSlast.
      SSlast = HIGH;
    }
  }
  /*
  if(marker == 0)
  {
    dat = SPDR;
    if(dat == 0xAB)
    {
      SPDR = 'a';
      marker++;
    }
  }
  else if(marker == 1)
  {
    dat = SPDR;
    current_command = dat;
    if(current_command == SPI_TestMessageCounter_ID)
    {
      process_AB14_Query();
      
    }
    if(current_command == SPI_Get_DIO_Port1_ID)
    {
      process_AB19_Query();
    }
    else if(current_command == SPI_Get_ANA_Port1_ID)
    {
      process_AB20_Query();
    }
    marker++;
  }
  else
  {
    if(current_command == SPI_TestMessageCounter_ID)
    {
      SPDR = outputBuffer_AB14[outputBuffer_index];
    }
    else if(current_command == SPI_Get_DIO_Port1_ID)
    {
      SPDR = outputBuffer_AB19[outputBuffer_index];
    }
    else if(current_command == SPI_Get_ANA_Port1_ID)
    {
      SPDR = outputBuffer_AB20[outputBuffer_index];
    }
    outputBuffer_index++;
    marker++;
    if(outputBuffer_index == 13)
    {
      outputBuffer_index = 0;
      marker = 0;
      run_spi_handler = false;
    }
  }
  */
}

String map_component_tostring(int v)
{
  switch(v)
  {
    case CONTROLLER_NODE:
      return "Control";
      break;
    case DIAGNOSTIC_NODE:
      return "Diag";
      break;
    case NAVIGATION_NODE:
      return "Nav";
      break;
    case EVOLUTION_NODE:
      return "Evol";
      break;
    case TARGETING_NODE:
      return "Target";
      break;
    case TIMING_NODE:
      return "Timing";
      break;
    case VISION_NODE:
      return "Vision";
      break;
    case COMMUNICATION_NODE:
      return "Comm";
      break;
    case DYNAMICS_NODE:
      return "Dyn";
      break;
    case POWER_NODE:
      return "Power";
      break;
    case POSE_NODE:
      return "Pose";
      break;
    default:
      return "UNK";
      break;
  }
}
String map_level_tostring(int v)
{
  switch(v)
  {
    case DEBUG:
      return "DEBUG";
      break;
    case INFO:
      return "INFO";
      break;
    case NOTICE:
      return "NOTICE";
      break;
    case WARN:
      return "WARN";
      break;
    case ERROR:
      return "ERROR";
      break;
    case FATAL:
      return "FATAL";
      break;
    default:
      return "UNKNOWN";
  }
}
String map_diagnostictype_tostring(int v)
{
  switch(v)
  {
    case NOERROR:
      return "NOERROR";
      break;
    case ELECTRICAL:
      return "ELEC";
      break;
    case SOFTWARE:
      return "SFTWR";
      break;
    case COMMUNICATIONS:
      return "COMM";
      break;
    case SENSORS:
      return "SNSRS";
      break;
    case ACTUATORS:
      return "ACTRS";
      break;
    case DATA_STORAGE:
      return "STRGE";
      break;
    case REMOTE_CONTROL:
      return "RC";
      break;
    case TARGET_ACQUISITION:
      return "TGTACQ";
      break;
    case POWER:
      return "PWR";
      break;
    case POSE:
      return "POSE";
      break;
    case GENERAL_ERROR:
      return "GEN";
      break;
    default:
      return "UNK";
      break;
  }
}
String map_message_tostring(int v)
{
  switch(v)
  {
    case NOERROR:
      return "NOERROR";
      break;
    case INITIALIZING:
      return "Init";
      break;
    case INITIALIZING_ERROR:
      return "Init Error";
      break;
    case DROPPING_PACKETS:
      return "Drop Pkt";
      break;
    case MISSING_HEARTBEATS:
      return "Msng Hrtbt";
      break;
    case DEVICE_NOT_AVAILABLE:
      return "Device N/A";
      break;
    case ROVER_ARMED:
      return "Armed";
      break;
    case ROVER_DISARMED:
      return "Disarmed";
      break;
    case TEMPERATURE_HIGH:
      return "Temp High";
      break;
    case TEMPERATURE_LOW:
      return "Temp Low";
      break;
    case DIAGNOSTIC_PASSED:
      return "Diag Ok";
      break;
    case DIAGNOSTIC_FAILED:
      return "Diag Bad";
      break;
    case RESOURCE_LEAK:
      return "Res Leak";
      break;
    case HIGH_RESOURCE_USAGE:
      return "High Res Usg";
      break;
    default:
      return "Unknown";
      break;
  }
}
int compute_checksum(unsigned char * outputbuffer)
{
  int checksum = 0;
  for(int i = 0; i < 12; i++)
  {
    checksum ^= outputbuffer[i];
  }
  return checksum;
}
bool init_ledstrips()
{
  led_strips.begin();
  led_strips.show(); // Initialize all pixels to 'off'
  for(uint16_t i=0; i<led_strips.numPixels(); i++) 
  {
    led_strips.setPixelColor(i, led_strips.Color(0, 2, 0));
    led_strips.show();
    delay(20);
  }
  for(uint16_t i=0; i<led_strips.numPixels(); i++) 
  {
    led_strips.setPixelColor(i, led_strips.Color(0, 0, 0));
    led_strips.show();
  }
}
void set_stripcolor(int m)
{
  int R,G,B = 0;
  switch(m)
  {
    case LED_OFF:
      R = 0;
      G = 0;
      B = 0;
      break;
    case LED_RED:
      R = 100;
      G = 0;
      B = 0;
      break;
    case LED_GREEN:
      R = 0;
      G = 100;
      B = 0;
      break;
    case LED_BLUE:
      R = 0;
      G = 0;
      B = 100;
      break;
    case LED_YELLOW:
      R = 50;
      G = 50;
      B = 0;
      break;
    default:
      R = 0;
      G = 0;
      B = 0;
      break;
  }
  for(uint16_t i=0; i<led_strips.numPixels(); i++) 
  {
    led_strips.setPixelColor(i, led_strips.Color(G, R, B));
    led_strips.show();
  }
}
void run_cylonmode(int m,int position)
{
  int R,G,B = 0;
  switch(m)
  {
    case LED_OFF:
      R = 0;
      G = 0;
      B = 0;
      break;
    case LED_RED:
      R = 255;
      G = 0;
      B = 0;
      break;
    case LED_GREEN:
      R = 0;
      G = 255;
      B = 0;
      break;
    case LED_BLUE:
      R = 0;
      G = 0;
      B = 255;
      break;
    case LED_YELLOW:
      R = 255;
      G = 255;
      B = 0;
      break;
    default:
      R = 0;
      G = 0;
      B = 0;
      break;
  }
  if(position == 0)
  {
    led_strips.setPixelColor(position, led_strips.Color(G, R, B));
    led_strips.setPixelColor(position+1, led_strips.Color(G/10, R/10, B/10));
    led_strips.setPixelColor(position+2, led_strips.Color(G/100, R/100, B/100));
  }
  if(position == 1)
  {
    led_strips.setPixelColor(position-1, led_strips.Color(G/10, R/10, B/10));
    led_strips.setPixelColor(position, led_strips.Color(G, R, B));
    led_strips.setPixelColor(position+1, led_strips.Color(G/10, R/10, B/10));
    led_strips.setPixelColor(position+2, led_strips.Color(G/100, R/100, B/100));
  }
  else if(position == ((LEDS_PER_STRIP*LED_STRIPS)-2))
  {
    led_strips.setPixelColor(position-2, led_strips.Color(G/100, R/100, B/100));
    led_strips.setPixelColor(position-1, led_strips.Color(G/10, R/10, B/10));
    led_strips.setPixelColor(position, led_strips.Color(G, R, B));
    led_strips.setPixelColor(position+1, led_strips.Color(G/10, R/10, B/10));
  }
  else if(position == ((LEDS_PER_STRIP*LED_STRIPS)-1))
  {
    led_strips.setPixelColor(position-2, led_strips.Color(G/100, R/100, B/100));
    led_strips.setPixelColor(position-1, led_strips.Color(G/10, R/10, B/10));
    led_strips.setPixelColor(position, led_strips.Color(G, R, B));
  }
  else
  {
    led_strips.setPixelColor(position-3, led_strips.Color(0, 0, 0));
    led_strips.setPixelColor(position-2, led_strips.Color(G/100, R/100, B/100));
    led_strips.setPixelColor(position-1, led_strips.Color(G/10, R/10, B/10));
    led_strips.setPixelColor(position, led_strips.Color(G, R, B));
    led_strips.setPixelColor(position+1, led_strips.Color(G/10, R/10, B/10));
    led_strips.setPixelColor(position+2, led_strips.Color(G/100, R/100, B/100));
    led_strips.setPixelColor(position+3, led_strips.Color(0, 0, 0));
  }
  led_strips.show();
}

