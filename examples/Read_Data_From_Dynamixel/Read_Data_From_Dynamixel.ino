/*
J.Teda 21/04/2013

This is an example of how to read data from a Dynamixel and write its value via a second serial port on an Arduino Mega.

You will need to make or purchase a half duplex to full duplex circuit to run this sketch. ( http://support.robotis.com/en/product/dynamixel/dxl_mx_main.htm )
*/

#include <Dynamixel_Serial.h>       // Library needed to control Dynamixal servo

#define SERVO_ID 0x01               // ID of which we will set Dynamixel too
#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set too (57600)

// The define error codes below are as per Robotis e-Manual ( http://support.robotis.com/en/product/dynamixel/communication/dxl_packet.htm )
#define VOLTAGE_ERROR          B00000001
#define ANGLE_LIMIT_ERROR      B00000010
#define OVER_HEAT_ERROR        B00000100
#define COMMAND_OUT_OFF_RANGE  B00001000
#define TX_CHECKSUM_ERROR      B00010000    // the packet recived by dynamixel had a checksum error
#define OVERLOAD_ERROR         B00100000
#define INSTRUCTION_ERROR      B01000000
#define RX_ERROR               B10000000    // packet sent by dynamixel was not recived by Arduino


void setup(){
  Serial.begin(57600);                                       // Set up Software Serial

  delay(1000);                                               // Give time for Dynamixel to start on power-up

  Dynamixel.begin(SERVO_SET_Baudrate);     // Set up Arduino to communicate to Dynamixel
  Dynamixel.setDirectionPin(SERVO_ControlPin);               // Optional. Set direction control pin
  Dynamixel.setStatusPaketReturnDelay(SERVO_ID, 6);          // Set Return packet delay to 6 uSec
  Dynamixel.setMode(SERVO_ID, WHEEL, 0, 0);                  // Set servo to WHEEL mode
  Dynamixel.wheel(SERVO_ID,LEFT,0x00D);                      // Move in wheel mode left
  Dynamixel.ledState(SERVO_ID, ON);                          // Set LED on Dynamixel ON
}


void loop(){

// ################### Read TEMPERATURE ###################
unsigned int   Temp = Dynamixel.readTemperature(SERVO_ID);

    if (Temp >> 12 == 0){               // There was no error found - Print value
        Serial.print("Servo ID: ");Serial.print(SERVO_ID);Serial.print(" Temperature is ");Serial.println(Temp);
    }else{                              // There was an error found - Print the error
        readStatusPacketError(Temp);
    }



// ################### Read Voltage ###################
// NOTE : Voltage value returned from Dynamixel is TEN times the real value e.g. 109 is 10.9 Volats

unsigned int Volt = Dynamixel.readVoltage(SERVO_ID);
    if (Volt >> 12 == 0){            // There was no error found - Print value
        Serial.print("Servo ID: ");Serial.print(SERVO_ID);Serial.print(" Voltage is ");Serial.println(Volt * 0.1);
    }else{                            // There was an error found - Print the error
        readStatusPacketError(Volt);
    }



// ################### Read Speed ###################
// Note one uni is about 0.11 rpm
unsigned int   Speed = Dynamixel.readSpeed(SERVO_ID);

    if (Speed >> 12 == 0){              // There was no error found -  Print value
        Serial.print("Servo ID: ");Serial.print(SERVO_ID);Serial.print(" speed is rpm ");Serial.println(Speed * 0.11);
    }else{                              // There was an error found - Print the error
        readStatusPacketError(Speed);
    }



// ################### Read Position ###################
// NOTE : one unit is about 0.088 of a Deg  ( 360 deg / 4095  = 0.088 deg)
unsigned int   Pos = Dynamixel.readPosition(SERVO_ID);

    if (Pos >> 12 == 0){              // There was no error found - Print value
        Serial.print("Servo ID: ");Serial.print(SERVO_ID);Serial.print(" Position is Deg ");Serial.println(Pos * 0.088);
    }else{                            // There was an error found - Print the error
        readStatusPacketError(Speed);
    }



// ################### Read Position ###################
// Note: bit 10 is load direction 0 = Counter clockwise, 1 = Clockwise
// Note: one unit is about 0.1% of maximum torque
unsigned int Load = Dynamixel.readLoad(SERVO_ID);

    if (Load >> 12 == 0){              // There was no error found - Print value
        if (Load >> 10 == 0){        // Load direction is CCW
           Serial.print("Servo ID: ");Serial.print(SERVO_ID);Serial.print(" Load is Counter Clockwise at % ");Serial.println((Load & 0x3FF) *0.1 );
        }else if (Load >> 10 == 1){  // Load direction is CW
          Serial.print("Servo ID: ");Serial.print(SERVO_ID);Serial.print(" Load is Clockwise at % ");Serial.println((Load & 0x3FF) * 0.1);
        }else{
          Serial.println("Process error");
        }

    }else{                            // There was an error found - Print the error
        readStatusPacketError(Load);
    }


  Serial.println("#######################################################");
  delay(1000);

}


// ################### This is used to decode error from Dynamixel ###################
void readStatusPacketError( byte error){    // We are using byte becasue we are only using the lower 8 bits, if you use a "unsigned int" you will need to mask it e.g (error & 0xFF)

  switch (error){
      case VOLTAGE_ERROR:
      Serial.print("Servo ID:");Serial.print(SERVO_ID);Serial.println(" Voltage is out of set operating range ");
      break;

      case ANGLE_LIMIT_ERROR:
      Serial.print("Servo ID:");Serial.print(SERVO_ID);Serial.println(" Goal postion is out of set operating range ");
      break;

      case OVER_HEAT_ERROR:
      Serial.print("Servo ID:");Serial.print(SERVO_ID);Serial.println(" Interal temperature is out of set operating range ");
      break;

      case COMMAND_OUT_OFF_RANGE:
      Serial.print("Servo ID:");Serial.print(SERVO_ID);Serial.println(" Command sent is incorrect ");
      break;

      case TX_CHECKSUM_ERROR:
      Serial.print("Servo ID:");Serial.print(SERVO_ID);Serial.println(" Packet recived by Dynamixel had checksum error ");
      break;

      case OVERLOAD_ERROR:
      Serial.print("Servo ID:");Serial.print(SERVO_ID);Serial.println(" Load on Dynamixel is greater then torque set ");
      break;

      case INSTRUCTION_ERROR:
      Serial.print("Servo ID:");Serial.print(SERVO_ID);Serial.println(" Instruction sent is undefined ");
      break;

      case RX_ERROR:
      Serial.print("Servo ID:");Serial.print(SERVO_ID);Serial.println(" NO! packet recived from Dynamixel ");
      break;

      default:
      Serial.println(" Switch ERROR!");

  }

}


