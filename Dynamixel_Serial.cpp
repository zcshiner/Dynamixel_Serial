/*

Version 2.2

25/04/2013
J.Teda "setMode" re-writen due to bug

21/04/2013
J.Teda "readStatusPacket" re-writen

20/04/2013
J.Teda TX pin High now controlled by UCSR0A ( Bit 6 ñ TXCn: USART Transmit Complete, Bit 5 ñ UDREn: USART Data Register Empty )

14/04/2013
J.Teda Error fixed in Angle limit settings (thanks = Chlen.Nigera )

02/06/2012
J.Teda New code writen using arrays, smaller and faster

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,  
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 

How Dynamixel work can be found
--------------------------------
Robotis e-Manual  
http://support.robotis.com

Overview of Communication 
http://support.robotis.com/en/product/dynamixel/dxl_communication.htm

Kind of Instruction 
http://support.robotis.com/en/product/dynamixel/communication/dxl_instruction.htm 

Instruction Packet & Status Packet (Return Packet)
http://support.robotis.com/en/product/dynamixel/communication/dxl_packet.htm
 
Control Table
http://support.robotis.com/en/product/dynamixel/mx_series/mx-28.htm
 
 */

#include "Dynamixel_Serial.h"

	unsigned char	Instruction_Packet_Array[15];	// Array to hold instruction packet data 
	unsigned char	Status_Packet_Array[8];			// Array to hold returned status packet data
	unsigned long 	Time_Counter;					// Timer for time out watchers
	char 	Direction_Pin = -1;		    	// Pin to control TX/RX buffer chip
	unsigned char 	Status_Return_Value = READ;		// Status packet return states ( NON , READ , ALL )


//##############################################################################
//############################ Public Methods ##################################
//##############################################################################

void DynamixelClass::begin(long baud){

#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__)
	Serial1.begin(baud);  // Set up Serial for Leonardo and Mega
	_serial = &Serial1;
#else
	Serial.begin(baud);   // Set up Serial for all others (Uno, etc)
    _serial = &Serial;
#endif

}

void DynamixelClass::begin(HardwareSerial &HWserial, long baud){
    HWserial.begin(baud); // Set up Serial for a specified Serial object
    _serial = &HWserial;
}

void DynamixelClass::begin(Stream &serial){
	_serial = &serial;  // Set a reference to a specified Stream object (Hard or Soft Serial)
}
 
void DynamixelClass::end(){
#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__)
	Serial1.end();
#else
	Serial.end();
#endif
}
 
 
void DynamixelClass::setDirectionPin(unsigned char D_Pin){
    Direction_Pin = D_Pin;
	pinMode(Direction_Pin,OUTPUT);
}

unsigned int DynamixelClass::reset(unsigned char ID){

	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = RESET_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_RESET;
	Instruction_Packet_Array[3] = ~(ID + RESET_LENGTH + COMMAND_RESET);	//Checksum;

    clearRXbuffer();

	transmitInstructionPacket();	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}	
}

unsigned int DynamixelClass::ping(unsigned char ID){
	
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = PING_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_PING;
	Instruction_Packet_Array[3] = ~(ID + PING_LENGTH + COMMAND_PING);
	
    clearRXbuffer();
	
	transmitInstructionPacket();    
	readStatusPacket();
	
	if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
		return (Status_Packet_Array[0]);			// Return SERVO ID
	}else{
		return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
	}            
}

unsigned int DynamixelClass::setStatusPaketReturnDelay(unsigned char ID,unsigned char ReturnDelay){
 	
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SET_RETURN_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = EEPROM_RETURN_DELAY_TIME;
	Instruction_Packet_Array[4] = byte(ReturnDelay/2);
	Instruction_Packet_Array[5] = ~(ID + SET_RETURN_LENGTH + COMMAND_WRITE_DATA + EEPROM_RETURN_DELAY_TIME + byte(ReturnDelay/2)); 	
	
    clearRXbuffer();
	
	transmitInstructionPacket();	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}		

}

unsigned int DynamixelClass::setID(unsigned char ID, unsigned char New_ID){    

	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SET_ID_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = EEPROM_ID;
	Instruction_Packet_Array[4] = New_ID;
	Instruction_Packet_Array[5] = ~(ID + SET_ID_LENGTH + COMMAND_WRITE_DATA+ EEPROM_ID + New_ID);  
	
    clearRXbuffer();
	
	transmitInstructionPacket();	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}		
}

unsigned int DynamixelClass::setBaudRate(unsigned char ID, long Baud){    					
	
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SET_BD_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = EEPROM_BAUD_RATE;
//	if (Baud > 2250000){
if (Baud >= 1000000){
	switch (Baud){
		case 2250000:
		Instruction_Packet_Array[4] = 0xFA;
		break;
		case 2500000:
		Instruction_Packet_Array[4] = 0xFB;
		break;
		case 3000000:
		Instruction_Packet_Array[4] = 0xFC;
		break;
		case 1000000:
		Instruction_Packet_Array[4] = 0x01;
		}
	}else{
	Instruction_Packet_Array[4] = byte((2000000/Baud) - 1);
	}
	Instruction_Packet_Array[5] = ~(ID + SET_BD_LENGTH + COMMAND_WRITE_DATA + EEPROM_BAUD_RATE + byte((2000000/Baud) - 1) ); 
	
    clearRXbuffer();

	transmitInstructionPacket();	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}	
}

unsigned int DynamixelClass::setMaxTorque( unsigned char ID, int Torque){
	
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SET_MAX_TORQUE_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = EEPROM_MAX_TORQUE_L ;
	Instruction_Packet_Array[4] = byte(Torque);
	Instruction_Packet_Array[5] = byte(Torque >> 8);
	Instruction_Packet_Array[6] = ~(ID + SET_MAX_TORQUE_LENGTH + COMMAND_WRITE_DATA + EEPROM_MAX_TORQUE_L + byte(Torque) + byte(Torque >> 8));
	
    clearRXbuffer();
	
	transmitInstructionPacket();	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}		
}

unsigned int DynamixelClass::setHoldingTorque(unsigned char ID, bool Set){
  
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SET_HOLDING_TORQUE_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = RAM_TORQUE_ENABLE;
	Instruction_Packet_Array[4] = Set;
	Instruction_Packet_Array[5] = ~(ID + SET_HOLDING_TORQUE_LENGTH + COMMAND_WRITE_DATA + RAM_TORQUE_ENABLE + Set);	
	
    clearRXbuffer();
	
	transmitInstructionPacket();	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}		
}

unsigned int DynamixelClass::setAlarmShutdown(unsigned char  ID,unsigned char Set){

  	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SET_ALARM_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = EEPROM_ALARM_SHUTDOWN;
	Instruction_Packet_Array[4] = Set;
	Instruction_Packet_Array[5] = ~(ID + SET_ALARM_LENGTH + COMMAND_WRITE_DATA + EEPROM_ALARM_SHUTDOWN + Set);	
    
    clearRXbuffer();

	transmitInstructionPacket();	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}			

}

unsigned int DynamixelClass::setStatusPaket(unsigned char  ID,unsigned char Set){

    Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SET_RETURN_LEVEL_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = EEPROM_RETURN_LEVEL;
	Instruction_Packet_Array[4] = Set;
	Instruction_Packet_Array[5] = ~(ID + SET_RETURN_LEVEL_LENGTH + COMMAND_WRITE_DATA + EEPROM_RETURN_LEVEL + Set);
    
	Status_Return_Value = Set;
	
    clearRXbuffer();

	transmitInstructionPacket();	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}		

}


unsigned int DynamixelClass::setMode(unsigned char ID, bool Dynamixel_Mode, unsigned int Dynamixel_CW_Limit,unsigned int Dynamixel_CCW_Limit){

	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SET_MODE_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = EEPROM_CW_ANGLE_LIMIT_L;
	 if ( Dynamixel_Mode == WHEEL) {									// Set WHEEL mode, this is done by setting both the clockwise and anti-clockwise angle limits to ZERO
		Instruction_Packet_Array[4] = 0x00;
		Instruction_Packet_Array[5] = 0x00;
		Instruction_Packet_Array[6] = 0x00;
		Instruction_Packet_Array[7] = 0x00;
		Instruction_Packet_Array[8] = ~(ID + SET_MODE_LENGTH + COMMAND_WRITE_DATA + EEPROM_CW_ANGLE_LIMIT_L);
	}else {																// Else set SERVO mode
		Instruction_Packet_Array[4] = byte(Dynamixel_CW_Limit);
		Instruction_Packet_Array[5] = byte((Dynamixel_CW_Limit & 0x0F00) >> 8);
		Instruction_Packet_Array[6] = byte(Dynamixel_CCW_Limit);
		Instruction_Packet_Array[7] = byte((Dynamixel_CCW_Limit & 0x0F00) >> 8);
		Instruction_Packet_Array[8] = ~(ID + SET_MODE_LENGTH + COMMAND_WRITE_DATA + EEPROM_CW_ANGLE_LIMIT_L + byte(Dynamixel_CW_Limit) + byte((Dynamixel_CW_Limit & 0x0F00) >> 8) + byte(Dynamixel_CCW_Limit) + byte((Dynamixel_CCW_Limit & 0x0F00) >> 8));
	}	
		
	
    clearRXbuffer();
	
	transmitInstructionPacket();
	
		if (Status_Return_Value == ALL){
		readStatusPacket();
			if (Status_Packet_Array[2] != 0){
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
			}
			
		}

 } 
 
 unsigned int DynamixelClass::setPunch(unsigned char ID,unsigned int Punch){
 
    Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SET_PUNCH_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = RAM_PUNCH_L;
	Instruction_Packet_Array[4] = byte(Punch);
	Instruction_Packet_Array[5] = byte(Punch >> 8);
	Instruction_Packet_Array[6] = ~(ID + SET_PUNCH_LENGTH + COMMAND_WRITE_DATA + RAM_PUNCH_L + byte(Punch) + byte(Punch >> 8) );
    
    clearRXbuffer();

	transmitInstructionPacket();	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}	
 
 }
 
 unsigned int DynamixelClass::setPID(unsigned char ID ,unsigned char P,unsigned char I,unsigned char D){
 
    Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SET_PID_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = RAM_PROPORTIONAL_GAIN;
	Instruction_Packet_Array[4] = P;
	Instruction_Packet_Array[5] = I;
	Instruction_Packet_Array[6] = D;
	Instruction_Packet_Array[7] = ~(ID + SET_PID_LENGTH + COMMAND_WRITE_DATA + RAM_PROPORTIONAL_GAIN + P + I + D );
    
    clearRXbuffer();

	transmitInstructionPacket();	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}
 }

unsigned int DynamixelClass::setTemp(unsigned char ID,unsigned char temp){
 	
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SET_TEMP_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = EEPROM_LIMIT_TEMPERATURE;
	Instruction_Packet_Array[4] = temp;
	Instruction_Packet_Array[5] = ~(ID + SET_TEMP_LENGTH + COMMAND_WRITE_DATA + EEPROM_LIMIT_TEMPERATURE + temp); 	
	
    clearRXbuffer();

	transmitInstructionPacket();	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}	
}

unsigned int DynamixelClass::setVoltage(unsigned char ID,unsigned char Volt_L, unsigned char Volt_H){
 	
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SET_VOLT_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = EEPROM_LOW_LIMIT_VOLTAGE;
	Instruction_Packet_Array[4] = Volt_L;
	Instruction_Packet_Array[5] = Volt_H;
	Instruction_Packet_Array[6] = ~(ID + SET_VOLT_LENGTH + COMMAND_WRITE_DATA + EEPROM_LOW_LIMIT_VOLTAGE + Volt_L + Volt_H); 	
	
    clearRXbuffer();

	transmitInstructionPacket();	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}
}
 
unsigned int DynamixelClass::servo(unsigned char ID,unsigned int Position,unsigned int Speed){
    
   	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SERVO_GOAL_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = RAM_GOAL_POSITION_L;
	Instruction_Packet_Array[4] = byte(Position);
	Instruction_Packet_Array[5] = byte((Position & 0x0F00) >> 8);
	Instruction_Packet_Array[6] = byte(Speed);
	Instruction_Packet_Array[7] = byte((Speed & 0x0F00) >> 8);
	Instruction_Packet_Array[8] = ~(ID + SERVO_GOAL_LENGTH + COMMAND_WRITE_DATA + RAM_GOAL_POSITION_L + Position + byte((Position & 0x0F00) >> 8) + Speed + byte((Speed & 0x0F00) >> 8));	
	
    clearRXbuffer();

	transmitInstructionPacket();	
	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}
}

unsigned int DynamixelClass::servoPreload(unsigned char ID,unsigned int Position,unsigned int Speed){
	
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SERVO_GOAL_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_REG_WRITE_DATA;
	Instruction_Packet_Array[3] = RAM_GOAL_POSITION_L;
	Instruction_Packet_Array[4] = byte(Position);
	Instruction_Packet_Array[5] = byte(Position >> 8);
	Instruction_Packet_Array[6] = byte(Speed);
	Instruction_Packet_Array[7] = byte(Speed >> 8);
	Instruction_Packet_Array[8] = ~(ID + SERVO_GOAL_LENGTH + COMMAND_REG_WRITE_DATA + RAM_GOAL_POSITION_L + byte(Position) + byte(Position >> 8) + byte(Speed) + byte(Speed >> 8));	
	
    clearRXbuffer();

	transmitInstructionPacket();	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}
}
 
unsigned int DynamixelClass::wheel(unsigned char ID, bool Rotation,unsigned int Speed){	

	byte Speed_H,Speed_L;
	Speed_L = Speed;	
		if (Rotation == 0){                         // Move Left                     
			Speed_H = Speed >> 8;
			}
		else if (Rotation == 1){					// Move Right
			Speed_H = (Speed >> 8)+4;	
			}	
			
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = WHEEL_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = RAM_GOAL_SPEED_L;
	Instruction_Packet_Array[4] = Speed_L;
	Instruction_Packet_Array[5] = Speed_H;
	Instruction_Packet_Array[6] = ~(ID + WHEEL_LENGTH + COMMAND_WRITE_DATA + RAM_GOAL_SPEED_L  + Speed_L + Speed_H);			
			

    clearRXbuffer();

	transmitInstructionPacket();	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}			

}

void DynamixelClass::wheelSync(unsigned char ID1, bool Dir1, unsigned int Speed1, unsigned char ID2, bool Dir2, unsigned int Speed2, unsigned char ID3, bool Dir3, unsigned int Speed3){
	
	byte Speed1_H,Speed1_L;
	Speed1_L = Speed1; 
		if (Dir1 == 0){                          // Move Left
			Speed1_H = Speed1 >> 8;
		}
		else if (Dir1 == 1)						// Move Right
		{   
			Speed1_H = (Speed1 >> 8)+4;
		}	

	byte Speed2_H,Speed2_L;
	Speed2_L = Speed2; 
		if (Dir2 == 0){                          // Move Left
			Speed2_H = Speed2 >> 8;
		}
		else if (Dir2 == 1)						// Move Right
		{   
			Speed2_H = (Speed2 >> 8)+4;
		}
  
	byte Speed3_H,Speed3_L;
	Speed3_L = Speed3; 
		if (Dir3 == 0){                          // Move Left
			Speed3_H = Speed3 >> 8;
		}
		else if (Dir3 == 1)						// Move Right
		{   
			Speed3_H = (Speed3 >> 8)+4;
		}		
		
	Instruction_Packet_Array[0] = 0xFE;			// When Writing a Sync comman you must address all(0xFE) servos
	Instruction_Packet_Array[1] = SYNC_LOAD_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_SYNC_WRITE;
	Instruction_Packet_Array[3] = RAM_GOAL_SPEED_L;
	Instruction_Packet_Array[4] = SYNC_DATA_LENGTH;
	Instruction_Packet_Array[5] = ID1;
	Instruction_Packet_Array[6] = Speed1_L;
	Instruction_Packet_Array[7] = Speed1_H;
	Instruction_Packet_Array[8] = ID2;
	Instruction_Packet_Array[9] = Speed2_L;
	Instruction_Packet_Array[10] = Speed2_H;
	Instruction_Packet_Array[11] = ID3;
	Instruction_Packet_Array[12] = Speed3_L;
	Instruction_Packet_Array[13] = Speed3_H;	
	Instruction_Packet_Array[14] = byte(~(0xFE + SYNC_LOAD_LENGTH + COMMAND_SYNC_WRITE + RAM_GOAL_SPEED_L + SYNC_DATA_LENGTH + ID1 + Speed1_L + Speed1_H + ID2 + Speed2_L + Speed2_H + ID3 + Speed3_L + Speed3_H));				
	
	transmitInstructionPacket();
 
}

unsigned int DynamixelClass::wheelPreload(unsigned char ID, bool Dir,unsigned int Speed){
	
	byte Speed_H,Speed_L;
	Speed_L = Speed; 
		if (Dir == 0){                          // Move Left
			Speed_H = Speed >> 8;
		}
		else if (Dir == 1)						// Move Right
		{   
			Speed_H = (Speed >> 8)+4;
		}	
		
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = WHEEL_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_REG_WRITE_DATA;
	Instruction_Packet_Array[3] = RAM_GOAL_SPEED_L;
	Instruction_Packet_Array[4] = Speed_L;
	Instruction_Packet_Array[5] = Speed_H;
	Instruction_Packet_Array[6] = ~(ID + WHEEL_LENGTH + COMMAND_REG_WRITE_DATA + RAM_GOAL_SPEED_L + Speed_L + Speed_H);				
			
    clearRXbuffer();

	transmitInstructionPacket();	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}		

}

unsigned int DynamixelClass::action(unsigned char ID){
	
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = RESET_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_ACTION;
	Instruction_Packet_Array[3] = ~(ID + ACTION_LENGTH + COMMAND_ACTION);
	
    clearRXbuffer();

	transmitInstructionPacket();	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}		
}

unsigned int DynamixelClass::ledState(unsigned char ID, bool Status){  
  
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = LED_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = RAM_LED;
	Instruction_Packet_Array[4] = Status;
	Instruction_Packet_Array[5] = ~(ID + LED_LENGTH + COMMAND_WRITE_DATA + RAM_LED + Status);	
	
    clearRXbuffer();

	transmitInstructionPacket();	
	
	if (ID == 0XFE || Status_Return_Value != ALL ){		// If ID of FE is used no status packets are returned so we do not need to check it
		return (0x00);
	}else{
		readStatusPacket();
		if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
			return (Status_Packet_Array[0]);			// Return SERVO ID
		}else{
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}	
}

unsigned int DynamixelClass::readTemperature(unsigned char ID){	
		
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = READ_TEMP_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_READ_DATA;
	Instruction_Packet_Array[3] = RAM_PRESENT_TEMPERATURE;
	Instruction_Packet_Array[4] = READ_ONE_BYTE_LENGTH;
	Instruction_Packet_Array[5] = ~(ID + READ_TEMP_LENGTH  + COMMAND_READ_DATA + RAM_PRESENT_TEMPERATURE + READ_ONE_BYTE_LENGTH);
	
    clearRXbuffer();

	transmitInstructionPacket();
	readStatusPacket();	

	if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value
		return Status_Packet_Array[3];
	}else{
		return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
	}
}

unsigned int DynamixelClass::readPosition(unsigned char ID){	
		
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = READ_POS_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_READ_DATA;
	Instruction_Packet_Array[3] = RAM_PRESENT_POSITION_L;
	Instruction_Packet_Array[4] = READ_TWO_BYTE_LENGTH;
	Instruction_Packet_Array[5] = ~(ID + READ_POS_LENGTH + COMMAND_READ_DATA + RAM_PRESENT_POSITION_L + READ_TWO_BYTE_LENGTH);	
	
    clearRXbuffer();

	transmitInstructionPacket();	
	readStatusPacket();
	
	if (Status_Packet_Array[2] == 0){ 				// If there is no status packet error return value											// If there is no status packet error return value
		return Status_Packet_Array[4] << 8 | Status_Packet_Array[3];	// Return present position value
	}else{
		return (Status_Packet_Array[2] | 0xF000);            				// If there is a error Returns error value
	}
}

unsigned int DynamixelClass::readLoad(unsigned char ID){	
	
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = READ_LOAD_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_READ_DATA;
	Instruction_Packet_Array[3] = RAM_PRESENT_LOAD_L;
	Instruction_Packet_Array[4] = READ_TWO_BYTE_LENGTH;
	Instruction_Packet_Array[5] = ~(ID + READ_LOAD_LENGTH + COMMAND_READ_DATA + RAM_PRESENT_LOAD_L  + READ_TWO_BYTE_LENGTH);
	
    clearRXbuffer();

	transmitInstructionPacket();
	readStatusPacket();
	
	if (Status_Packet_Array[2] == 0){ 											// If there is no status packet error return value
		return ((Status_Packet_Array[4] << 8) | Status_Packet_Array[3]);	// Return present load value
	}else{
		return (Status_Packet_Array[2] | 0xF000);            						// If there is a error Returns error value
	}
}

unsigned int DynamixelClass::readSpeed(unsigned char ID){	
	
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = READ_SPEED_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_READ_DATA;
	Instruction_Packet_Array[3] = RAM_PRESENT_SPEED_L;
	Instruction_Packet_Array[4] = READ_TWO_BYTE_LENGTH;
	Instruction_Packet_Array[5] = ~(ID + READ_SPEED_LENGTH + COMMAND_READ_DATA + RAM_PRESENT_SPEED_L + READ_TWO_BYTE_LENGTH);
	
    clearRXbuffer();

	transmitInstructionPacket();
	readStatusPacket();
	
	if (Status_Packet_Array[2] == 0){ 											// If there is no status packet error return value
		return (Status_Packet_Array[4] << 8) | Status_Packet_Array[3];	// Return present position value
	}else{
		return (Status_Packet_Array[2] | 0xF000);            				// If there is a error Returns error value
	}
}


unsigned int DynamixelClass::readVoltage(unsigned char ID){    
		
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = READ_VOLT_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_READ_DATA;
	Instruction_Packet_Array[3] = RAM_PRESENT_VOLTAGE;
	Instruction_Packet_Array[4] = READ_ONE_BYTE_LENGTH;
	Instruction_Packet_Array[5] = ~(ID + READ_VOLT_LENGTH + COMMAND_READ_DATA + RAM_PRESENT_VOLTAGE + READ_ONE_BYTE_LENGTH);	

    clearRXbuffer();

	transmitInstructionPacket();
	readStatusPacket();
	
	if (Status_Packet_Array[2] == 0){ 					// If there is no status packet error return value
		return Status_Packet_Array[3];					// Return voltage value (value retured by Dynamixel is 10 times actual voltage)
	}else{
		return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
	}
}

unsigned int DynamixelClass::checkRegister(unsigned char ID){    
		
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = READ_REGISTER_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_READ_DATA;
	Instruction_Packet_Array[3] = RAM_REGISTER;
	Instruction_Packet_Array[4] = READ_ONE_BYTE_LENGTH;
	Instruction_Packet_Array[5] = ~(ID + READ_REGISTER_LENGTH + COMMAND_READ_DATA + RAM_REGISTER + READ_ONE_BYTE_LENGTH);

    clearRXbuffer();

	transmitInstructionPacket();
	readStatusPacket();
	
	if (Status_Packet_Array[2] == 0){ 					// If there is no status packet error return value
		return (Status_Packet_Array[3]);			// Return register value
	}else{
		return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
	}
}

unsigned int DynamixelClass::checkMovement(unsigned char ID){    
		
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = READ_MOVING_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_READ_DATA;
	Instruction_Packet_Array[3] = RAM_MOVING;
	Instruction_Packet_Array[4] = READ_ONE_BYTE_LENGTH;
	Instruction_Packet_Array[5] = ~(ID + READ_MOVING_LENGTH + COMMAND_READ_DATA + RAM_MOVING + READ_ONE_BYTE_LENGTH);

    clearRXbuffer();

	transmitInstructionPacket();
	readStatusPacket();
	
	if (Status_Packet_Array[2] == 0){ 					// If there is no status packet error return value
		return (Status_Packet_Array[3]);			// Return movement value
	}else{
		return (Status_Packet_Array[2] | 0xF000);            // If there is a error Returns error value
	}
}

unsigned int DynamixelClass::checkLock(unsigned char ID){    
    
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = READ_LOCK_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_READ_DATA;
	Instruction_Packet_Array[3] = RAM_LOCK;
	Instruction_Packet_Array[4] = READ_ONE_BYTE_LENGTH;
	Instruction_Packet_Array[5] = ~(ID + READ_LOCK_LENGTH + COMMAND_READ_DATA + RAM_LOCK + READ_ONE_BYTE_LENGTH);	

    clearRXbuffer();

	transmitInstructionPacket();
	readStatusPacket();
	
	if (Status_Packet_Array[2] == 0){ 					// If there is no status packet error return value
		return (Status_Packet_Array[3]);			// Return Lock value
	}else{
		return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
	}
}


//##############################################################################
//########################## Private Methods ###################################
//##############################################################################

void DynamixelClass::transmitInstructionPacket(void){									// Transmit instruction packet to Dynamixel

	unsigned char Counter;
	Counter = 0;
	
	if (Direction_Pin > -1){
	    digitalWrite(Direction_Pin,HIGH);												// Set TX Buffer pin to HIGH	
    }

	_serial->write(HEADER);																// Write Header (0xFF) data 1 to serial                     
	_serial->write(HEADER);																// Write Header (0xFF) data 2 to serial
	_serial->write(Instruction_Packet_Array[0]);		    							// Write Dynamixal ID to serial	
	_serial->write(Instruction_Packet_Array[1]);										// Write packet length to serial	
	
	do{																					
		_serial->write(Instruction_Packet_Array[Counter + 2]);							// Write Instuction & Parameters (if there is any) to serial
		Counter++;
	}while((Instruction_Packet_Array[1] - 2) >= Counter);
	
	_serial->write(Instruction_Packet_Array[Counter + 2]);								// Write check sum to serial

#if defined(__AVR_ATmega32U4__)	 // Arduino Leonardo uses a different hardware address
	if ((UCSR1A & B01100000) != B01100000){												// Wait for TX data to be sent
		_serial->flush();
	}
	
#else	
	if ((UCSR0A & B01100000) != B01100000){												// Wait for TX data to be sent
		_serial->flush();
	}	

#endif	

    if (Direction_Pin > -1){
        digitalWrite(Direction_Pin,LOW);													//Set TX Buffer pin to LOW after data has been sent
    }    
}


unsigned int DynamixelClass::readStatusPacket(void){

	unsigned char Counter = 0x00;
	unsigned char First_Header = 0x00;
	
	Status_Packet_Array[0] = 0x00;
	Status_Packet_Array[1] = 0x00;
	Status_Packet_Array[2] = 0x00;														
	Status_Packet_Array[3] = 0x00;
	

	Time_Counter = STATUS_PACKET_TIMEOUT + millis(); 									// Setup time out error
	
while(STATUS_FRAME_BUFFER >= _serial->available()){										// Wait for " header + header + frame length + error " RX data

	    if (millis() >= Time_Counter){
		return Status_Packet_Array[2] = B10000000;										// Return with Error if Serial data not received with in time limit
		}
} 
	
	if (_serial->peek() == 0xFF && First_Header != 0xFF){
		First_Header = _serial->read();													// Clear 1st header from RX buffer
		}else if (_serial->peek() == -1){
		return Status_Packet_Array[2] = B10000000;										// Return with Error if two headers are not found
		}
		if(_serial->peek() == 0xFF && First_Header == 0xFF){
			_serial->read();																// Clear 2nd header from RX buffer
			Status_Packet_Array[0] = _serial->read();                 					// ID sent from Dynamixel
			Status_Packet_Array[1] = _serial->read();										// Frame Length of status packet
			Status_Packet_Array[2] = _serial->read();										// Error byte 
			
			Time_Counter = STATUS_PACKET_TIMEOUT + millis();
				while(Status_Packet_Array[1] - 2 >= _serial->available()){				// Wait for wait for "Para1 + ... Para X" received data

					if (millis() >= Time_Counter){
					return Status_Packet_Array[2] = B10000000;							// Return with Error if Serial data not received with in time limit
					}
				} 
		
					do{
						Status_Packet_Array[3 + Counter] = _serial->read();
						Counter++;				
					}while(Status_Packet_Array[1] > Counter);							// Read Parameter(s) into array
			
			Status_Packet_Array[Counter + 4] = _serial->read();							// Read Check sum	
				
		}else{
		return Status_Packet_Array[2] = B10000000;										// Return with Error if two headers are not found
		}
}

void DynamixelClass::clearRXbuffer(void){
	while (_serial->read() != -1);  // Clear RX buffer;
}

DynamixelClass Dynamixel;
