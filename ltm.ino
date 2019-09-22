/* LTM decoder
*
*  This is a state machince type architecture to decommutate serial data from
*  an INAV FC pushing LTM to a companion computer
*  states:
* 0: idle/wait for sync
*   1: First header ($)
* 2: Second header (T)
* 3: Decommutate R frame & take R frame action
* 4: Decommutate T frame & take T frame action
* 5: Decommutate G frame 
* 6: Decommutate A frame
* 7: Decoomutate S frame
* 8: Decommutate O frame 
* 9: Decommutate N frame
* 10: Decommutate X frame
*
*/

uint8_t state;
uint8_t c;
long bailout;

// R and T frame 16 bytes each
uint16_t rcChannels[16]; 

// G frame 14 bytes
long latLong[2];
uint8_t groundSpeed;
long altitude;
uint8_t satcount;
uint8_t fix;

// A frame 6 bytes

int16_t  attitude[3];

// S frame 7 bytes
uint16_t vbat;
uint16_t mah;
uint8_t rssi;
uint8_t airSpeed;
bool armStatus;
bool failsafeStatus;
uint8_t status;

// O frame 14 bytes 
long originlatLong[2];
uint32_t originAltitude;  //not used in iNav
uint8_t OSDon; //always 1
uint8_t fix;

// N frame 6 bytes
uint8_t GPSmode;
uint8_t navMode;
uint8_t navAction;
uint8_t waypointNumber;
uint8_t navError;
uint8_t flags;

//X frame 6 bytes

uint16_t hdop;
uint8_t hwStatus;
uint8_t xframeCount;
uint8_t disarmReason;

void setup(){
  Serial.begin(115200);
  Serial1.begin(9600);
  state = 0;
  bailout = millis();
}

void loop(){
  
  //bailout timer
  if(millis() - bailout > 3000 && state !=0){
    state = 0;
    }
    
  switch(state) {
    
    case 0: //idle and serial clearing 
      
      if(Serial1.available() > 0){
        c = Serial1.read();
          if(char(c) == '$'){
            state = 1;
            bailout = millis();
          }
      }
      
      break;
      
    case 1: //Making sure that $ wasn't a happy coindcidence
      
      if(Serial1.available() > 0){
        c = Serial1.read();
          if(char(c) == 'T'){ //valid frame
            state = 2;
            bailout = millis();
          }
          else{
            state = 0; //Nope, back to square 1
          }
      }
      
      break;
      
    case 2:
    
      if(Serial1.available() > 0){
        c = Serial1.read();
          if(char(c) == 'R'){ //RX frame
            state = 3;
            bailout = millis();
          }
          else if(char(c) == 'Y'){
            state = 4;
            bailout = millis();
          }
		  else if(char(c) == 'G'){
			state =5;
			bailout = millis();			 
		  }
		  else if(char(c) == 'A'){
			state =6;
			bailout = millis();			 
		  }
		  else if(char(c) == 'S'){
			state =7;
			bailout = millis();			 
		  }
		  else if(char(c) == 'O'){
			state =8;
			bailout = millis();			 
		  }
		  else if(char(c) == 'N'){
			state =9;
			bailout = millis();			 
		  }
		  else if(char(c) == 'X'){
			state =10;
			bailout = millis();			 
		  }
          else {
            state = 0;
          }
          
//        Serial.print("Received valid header, here is the frame: ");
//        Serial.print(char(c));
//        Serial.print(" -- which means this state: ");
//        Serial.println(state);
      }
      
      break;
      
    case 3:
    
      if(Serial1.available() >16){ //R frame is 17 bytes long with checksum so lets wait until it gets there
        bailout = millis();
        uint8_t serBuffer[16];
        uint8_t crc =0;
        
        for(int i = 0; i < 16; i++){
          serBuffer[i] = Serial1.read();
          crc ^= serBuffer[i];
        }
        
        //process checksum
        
        //getting the checksum
        c = Serial1.read();
        Serial.println(c);
        Serial.println(crc);
        if(crc == c){  //good checksum, if not reset the state machine.
          
          //I feel like a Gorilla on the keyboard for this next section
          rcChannels[0] = serBuffer[0];
          rcChannels[0] |= serBuffer[1] << 8;
          rcChannels[1] = serBuffer[2];
          rcChannels[1] |= serBuffer[3] << 8;
          rcChannels[2] = serBuffer[4];
          rcChannels[2] |= serBuffer[5] << 8;
          rcChannels[3] = serBuffer[6];
          rcChannels[3] |= serBuffer[7] << 8;
          rcChannels[4] = serBuffer[8];
          rcChannels[4] |= serBuffer[9] << 8;
          rcChannels[5] = serBuffer[10];
          rcChannels[5] |= serBuffer[11] << 8;
          rcChannels[6] = serBuffer[12];
          rcChannels[6] |= serBuffer[13] << 8;
          rcChannels[7] = serBuffer[14];
          rcChannels[7] |= serBuffer[15] << 8;
          
          
          Serial.print("Channel 1: ");
          Serial.print(rcChannels[0]);
          Serial.print(" Channel 2: ");
          Serial.print(rcChannels[1]);
          Serial.print(" Channel 3: ");
          Serial.print(rcChannels[2]);
          Serial.print(" Channel 4: ");
          Serial.print(rcChannels[3]);
          Serial.println();
          Serial.print("Channel 5: ");
          Serial.print(rcChannels[4]);
          Serial.print(" Channel 6: ");
          Serial.print(rcChannels[5]);
          Serial.print(" Channel 7: ");
          Serial.print(rcChannels[6]);
          Serial.print(" Channel 8: ");
          Serial.print(rcChannels[7]);
          Serial.println();
        }
        
        //reset the state machine
        
        state = 0;
      
      }
      
      break;
      
    case 4:
    
      if(Serial1.available() >16){ //T frame is 17 bytes long with checksum so lets wait until it gets there
        bailout = millis();
        uint8_t serBuffer[16];
        uint8_t crc =0;
        
        for(int i = 0; i < 16; i++){
          serBuffer[i] = Serial1.read();
          crc ^= serBuffer[i];
        }
        
        //process checksum
        
        //getting the checksum
        c = Serial1.read();
        Serial.println(c);
        Serial.println(crc);
        if(crc == c){  //good checksum, if not reset the state machine.
          
          //I feel like a Gorilla on the keyboard for this next section
          rcChannels[8] = serBuffer[0];
          rcChannels[8] |= serBuffer[1] << 8;
          rcChannels[9] = serBuffer[2];
          rcChannels[9] |= serBuffer[3] << 8;
          rcChannels[10] = serBuffer[4];
          rcChannels[10] |= serBuffer[5] << 8;
          rcChannels[11] = serBuffer[6];
          rcChannels[11] |= serBuffer[7] << 8;
          rcChannels[12] = serBuffer[8];
          rcChannels[12] |= serBuffer[9] << 8;
          rcChannels[13] = serBuffer[10];
          rcChannels[13] |= serBuffer[11] << 8;
          rcChannels[14] = serBuffer[12];
          rcChannels[14] |= serBuffer[13] << 8;
          rcChannels[15] = serBuffer[14];
          rcChannels[15] |= serBuffer[15] << 8;
        
        Serial.print("Channel 9: ");
        Serial.print(rcChannels[8]);
        Serial.print(" Channel 10: ");
        Serial.print(rcChannels[9]);
        Serial.print(" Channel 11: ");
        Serial.print(rcChannels[10]);
        Serial.print(" Channel 12: ");
        Serial.print(rcChannels[11]);
        Serial.println();
        Serial.print("Channel 13: ");
        Serial.print(rcChannels[12]);
        Serial.print(" Channel 14: ");
        Serial.print(rcChannels[13]);
        Serial.print(" Channel 15: ");
        Serial.print(rcChannels[14]);
        Serial.print(" Channel 16: ");
        Serial.print(rcChannels[15]);
        Serial.println();
        }
      state = 0;
      }
      
      break;
	  
    case: 5
		
		if(Serial1.available() >14){ //G frame is 15 bytes long with checksum so lets wait until it gets there
			bailout = millis();
			uint8_t serBuffer[14];
			uint8_t crc =0;
        
			for(int i = 0; i < 14; i++){
				serBuffer[i] = Serial1.read();
				crc ^= serBuffer[i];
			}
			
			c = Serial1.read();

			if(c == crc){
				// latitude decimal degrees * 1E7
				latLong[0] = serBuffer[0];
				latLong[0] = serBuffer[1] << 8;
				latLong[0] = serBuffer[2] << 16;
				latLong[0] = serBuffer[3] << 24;
				// longitude decimal degrees * 1E7
				latLong[1] = serBuffer[4];
				latLong[1] = serBuffer[5] << 8;
				latLong[1] = serBuffer[6] << 16;
				latLong[1] = serBuffer[7] << 24;
				
				groundSpeed = serBuffer[8];
				
				altitude = serBuffer[9];
				altitude = serBuffer[10] << 8;
				altitude = serBuffer[11] << 16 ;
				altitude = serBuffer[12] << 24;
				
				satcount = serBuffer[13] >> 2 & 0xFF;
				fix = serBuffer[13] & 0x3;
			}
			
			state = 0;
		}
	break;
	
	case 6: //A frame 6 bytes

		if(Serial1.available() >6){ //A frame is 6 bytes long with checksum so lets wait until it gets there
			bailout = millis();
			uint8_t serBuffer[6];
			uint8_t crc =0;
        
			for(int i = 0; i < 6; i++){
				serBuffer[i] = Serial1.read();
				crc ^= serBuffer[i];
			}
			
			c = Serial1.read();

			if(c == crc){
				attitude[0] = serBuffer[0];
				attitude[0] = serBuffer[1] << 8;
				attitude[1] = serBuffer[2];
				attitude[1] = serBuffer[3] << 8;
				attitude[2] = serBuffer[4];
				attitude[2] = serBuffer[5] << 8;
			}
			
			state = 0;	
		}
		
	break;
	
	case 7: //S frame 7 bytes

		if(Serial1.available() >7){ //S frame is 7 bytes long with checksum so lets wait until it gets there
			bailout = millis();
			uint8_t serBuffer[7];
			uint8_t crc =0;
        
			for(int i = 0; i < 7; i++){
				serBuffer[i] = Serial1.read();
				crc ^= serBuffer[i];
			}
			
			c = Serial1.read();

			if(c == crc){
				vbat = serBuffer[0];
				vbat = serBuffer[1] << 8;
				mah = serBuffer[2];
				mah = serBuffer[3] << 8;
				rssi = serBuffer[4];
				airSpeed = serBuffer[5];
				armStatus = bitRead(serBuffer[6], 0);
				failsafeStatus = bitRead(serBuffer[6], 1);
				status = serBuffer[6] >> 2 & 0xFF;
			}
			
			state = 0;	
		}
	break;

	case 8: //O frame 14 bytes

		if(Serial1.available() >14){ //O frame is 14 bytes long with checksum so lets wait until it gets there
			bailout = millis();
			uint8_t serBuffer[14];
			uint8_t crc =0;
        
			for(int i = 0; i < 14; i++){
				serBuffer[i] = Serial1.read();
				crc ^= serBuffer[i];
			}
			
			c = Serial1.read();

			if(c == crc){
				// Origin latitude decimal degrees * 1E7
				originlatLong[0] = serBuffer[0];
				originlatLong[0] = serBuffer[1] << 8;
				originlatLong[0] = serBuffer[2] << 16;
				originlatLong[0] = serBuffer[3] << 24;
				// Origin longitude decimal degrees * 1E7
				originlatLong[1] = serBuffer[4];
				originlatLong[1] = serBuffer[5] << 8;
				originlatLong[1] = serBuffer[6] << 16;
				originlatLong[1] = serBuffer[7] << 24;
				//originAltitude
				originAltitude = serBuffer[8];
				originAltitude = serBuffer[9] << 8;
				originAltitude = serBuffer[10] << 16;
				originAltitude = serBuffer[11] << 24;
				OSDon = serBuffer[12];
				fix = serBuffer[13];
			}
			
			state = 0;	
		}
	break;
	
	case 9: //N frame 6 bytes

		if(Serial1.available() >6){ //X frame is 6 bytes long with checksum so lets wait until it gets there
			bailout = millis();
			uint8_t serBuffer[6];
			uint8_t crc =0;
        
			for(int i = 0; i < 6; i++){
				serBuffer[i] = Serial1.read();
				crc ^= serBuffer[i];
			}
			
			c = Serial1.read();

			if(c == crc){
				GPSmode = serBuffer[0];
				navMode = serBuffer[1];
				navAction = serBuffer[2];
				waypointNumber = serBuffer[3];
				navError = serbuffer[4];
				flags = serbuffer[5];
			}
			
			state = 0;	
		}
	break;
	
	case 10: //X frame 6 bytes

		if(Serial1.available() >6){ //X frame is 6 bytes long with checksum so lets wait until it gets there
			bailout = millis();
			uint8_t serBuffer[6];
			uint8_t crc =0;
        
			for(int i = 0; i < 6; i++){
				serBuffer[i] = Serial1.read();
				crc ^= serBuffer[i];
			}
			
			c = Serial1.read();

			if(c == crc){
				hdop = serBuffer[0];
				hdop = serBuffer[1] << 8;
				hwStatus = serBuffer[2];
				xframeCount = serBuffer[3];
				disarmReason = serbuffer[4];
			}
			
			state = 0;	
		}
	break;
	
    default:
    
    break;
  
  }
  
}