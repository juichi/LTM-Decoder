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
* 
*
*/

uint8_t state;
uint8_t c;
uint16_t rcChannels[16]; 
long bailout;

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
      
    default:
    
      break;
  
  }
  
}
