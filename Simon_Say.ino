#include "pitches.h"
#include "Simon_Say.h"

#include <SPI.h>

#define  uchar unsigned char
#define uint  unsigned int

// Maximum length of an array
#define MAX_LEN 16

/////////////////////////////////////////////////////////////////////
//set the pin
/////////////////////////////////////////////////////////////////////
const int chipSelectPin = 10;
const int NRSTPD = 5;

// MF522 Command Words
#define PCD_IDLE              0x00               // No action; Cancel the current command
#define PCD_AUTHENT           0x0E               // Authentication key
#define PCD_RECEIVE           0x08               // Receive data
#define PCD_TRANSMIT          0x04               // Send data
#define PCD_TRANSCEIVE        0x0C               // Send and receive data
#define PCD_RESETPHASE        0x0F               // Reset
#define PCD_CALCCRC           0x03               // CRC calculation

// Mifare One Command Words
#define PICC_REQIDL           0x26               // Request hibernation
#define PICC_REQALL           0x52               // Look for all cards in range
#define PICC_ANTICOLL         0x93               // Anti-collision
#define PICC_SElECTTAG        0x93               // Election card
#define PICC_AUTHENT1A        0x60               // Verify authentication key A
#define PICC_AUTHENT1B        0x61               // Verify authentication key B
#define PICC_READ             0x30               // Read block
#define PICC_WRITE            0xA0               // Write block
#define PICC_DECREMENT        0xC0               // Chargeback
#define PICC_INCREMENT        0xC1               // Recharge
#define PICC_RESTORE          0xC2               // Adjust block data to buffer
#define PICC_TRANSFER         0xB0               // Save the buffer data
#define PICC_HALT             0x50               // Sleep


// MF522 communication error codes
#define MI_OK                 0
#define MI_NOTAGERR           1
#define MI_ERR                2


//------------------MFRC522 Register---------------
//Page 0:Command and Status
#define     Reserved00            0x00    
#define     CommandReg            0x01    
#define     CommIEnReg            0x02    
#define     DivlEnReg             0x03    
#define     CommIrqReg            0x04    
#define     DivIrqReg             0x05
#define     ErrorReg              0x06    
#define     Status1Reg            0x07    
#define     Status2Reg            0x08    
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     Reserved01            0x0F
//Page 1:Command     
#define     Reserved10            0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAutoReg             0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     Reserved11            0x1A
#define     Reserved12            0x1B
#define     MifareReg             0x1C
#define     Reserved13            0x1D
#define     Reserved14            0x1E
#define     SerialSpeedReg        0x1F
//Page 2:CFG    
#define     Reserved20            0x20  
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     Reserved21            0x23
#define     ModWidthReg           0x24
#define     Reserved22            0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsPReg            0x28
#define     ModGsPReg             0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F
//Page 3:TestRegister     
#define     Reserved30            0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39  
#define     TestDAC2Reg           0x3A   
#define     TestADCReg            0x3B   
#define     Reserved31            0x3C   
#define     Reserved32            0x3D   
#define     Reserved33            0x3E   
#define     Reserved34        0x3F
//-----------------------------------------------

// 4-byte card serial number, 5-byte checksum
uchar serNum[5];

uchar  writeData[16]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // Initialise
uchar  moneyConsume = 20 ;  // Spend $20
uchar  moneyAdd = 10 ;  // Recharge $10
// Sector A Password, 16 sectors, each sector password is 6-bytes
 uchar sectorKeyA[16][16] = {{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                             {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},//when you try it again, please change it into your new password
                             {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                             {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                            };
 uchar sectorNewKeyA[16][16] = {{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xff,0x07,0x80,0x69, 0x19,0x84,0x07,0x15,0x76,0x14},
                                {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xff,0x07,0x80,0x69, 0x19,0x33,0x07,0x15,0x34,0x14},
                                {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xff,0x07,0x80,0x69, 0x19,0x33,0x07,0x15,0x34,0x14},
                               };


int num=0;//儲值後總金額
int saveMoney=0;




#define TONE_PIN 0 //定義蜂鳴器腳位
#define NUMBER 4   //4顆LED燈與4個開關
const int switches[NUMBER] = {6, 7, 8, 9};
const int leds[NUMBER] = {2, 3, 4, 5};
const int notes[NUMBER] = {//定義LED對應的音符頻率
  NOTE_C4, NOTE_D4, NOTE_E4,NOTE_F4,
};

const byte sprite[4][8] = {
  {0x00,0x90,0xCA,0x3F,0x3F,0x4A,0xD0,0x00},
  {0x00,0x90,0xCA,0x3F,0xBF,0xCA,0x10,0x00},
  {0x00,0x10,0x8A,0xFF,0x7F,0xCA,0x10,0x00},
  {0x00,0x90,0xCA,0x3F,0x3F,0x4A,0xD0,0x00}
};
const byte lose[8] = {0x81,0x42,0x24,0x18,0x18,0x24,0x42,0x81};
const byte win[8] = {0x0E,0x1E,0xAB,0xE1,0xAF,0x1E,0x0E,0x00};

int q_num = 3;
int *questions = NULL;
int *answers = NULL;
int answer_num = 0;
unsigned long lastClickTime;

int score = 0;
int totscore;

State state = STATE_START;

int noteStart[] = {
  NOTE_C4, NOTE_F4, NOTE_C4, NOTE_F4, NOTE_C4, NOTE_F4, NOTE_C4, 
  NOTE_F4, NOTE_G4, NOTE_F4, NOTE_E4, NOTE_F4, NOTE_G4
};
int noteDurationsStart[] = {16, 8, 16, 8, 16, 4, 16, 16, 16, 16, 8, 16, 4};

int noteCorrect[] = {NOTE_C4, NOTE_C4, NOTE_G4, NOTE_C5, NOTE_G4, NOTE_C5};
int noteDurationsCorrect[] = {16, 16, 16, 8, 16, 8};

int noteWrong[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, NOTE_, NOTE_B3, NOTE_C4
};
int noteDurationsWrong[] = {16, 32, 32, 16, 16, 16, 16, 16};

Melody melodys[MELODY_MAX] = {
  {noteStart, noteDurationsStart, 13},
  {noteCorrect, noteDurationsCorrect, 6},
  {noteWrong, noteDurationsWrong, 8},
};

void playtone(int *note, int *noteDurations, int num){
  for(int thisNote = 0; thisNote < num; thisNote++){

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 3000 / noteDurations[thisNote];
    tone(TONE_PIN, note[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    //noTone(8);
  }
}

void playMelody(Melody_Enum me){
  playtone(melodys[me].note, melodys[me].duration, melodys[me].number);
}

#include<SPI.h>
const byte NOOP=0x0;
const byte DECODEMODE=0x9;
const byte INTENSITY=0xA;
const byte SCANLIMIT=0xB;
const byte SHUTDOWN=0xC;
const byte DISPLAYTEST=0xF;
void max7219(const byte reg,const byte data){
  digitalWrite(SS,LOW);
  SPI.transfer(reg);
  SPI.transfer(data);
  digitalWrite(SS,HIGH);
}

void setup(){
Serial.begin(9600);
SPI.begin();
pinMode(chipSelectPin,OUTPUT);             // Set digital pin 10 as OUTPUT to connect it to the RFID /ENABLE pin 
    digitalWrite(chipSelectPin, LOW);          // Activate the RFID reader
pinMode(NRSTPD,OUTPUT);               // Set digital pin 10 , Not Reset and Power-down
    digitalWrite(NRSTPD, HIGH);

    
  for(int i = 0; i < NUMBER; i++){
    pinMode(switches[i], INPUT);
    digitalWrite(switches[i], HIGH);
    
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);
  }
  randomSeed(analogRead(A1));//亂數製作問題

}

void reset(){
  free(questions);
  questions = NULL;
  
  answer_num = 0;
  free(answers);
  answers = NULL;
  
  for(int i = 0; i < NUMBER; i++){
    digitalWrite(leds[i], LOW);
  }
}

void playOneTone(int note, float delayScale){
  int noteDuration = 3000 / 8;
    tone(TONE_PIN, note, noteDuration);
  
    int pauseBetweenNotes = noteDuration * delayScale;
    delay(pauseBetweenNotes);

}
void playQuestionsTone(){
  for(int i = 0; i < q_num; i++){
    digitalWrite(leds[questions[i]], HIGH);
    playOneTone(notes[questions[i]], 1.3);
    digitalWrite(leds[questions[i]], LOW);
  }
}

boolean check(){
  score=0;
  for(int i = 0; i < q_num; i++){
    if(questions[i] != answers[i]){
      return false;
    }
    score=i+1;                              //得分累計
  }
  return true;
}

void loop(){
  uchar i,tmp;
  uchar status;
  uchar str[MAX_LEN];
  uchar RC_size;
  uchar blockAddr;  // Select the operating block address, 0-63


int a=0,b=0;
  int addmoney;
  int money[3];
  char tag;
 if(Serial.available())       //輸入最多3位數字
 {
  int i=0;
  while ((tag=Serial.read())!=-1)
  {
    if(i<3){
    money[i]=tag-'0';
    i++;
   }
  }
  addmoney=money[0]*100+money[1]*10+money[2]*1;
  Serial.print("Add money: NT$ ");
  Serial.println(addmoney);
  num=num+addmoney;
  Serial.print("Total: NT$ ");
  Serial.println(num);
  if(num>500)
  {
    Serial.println("Don't addicting!");
    }
  saveMoney=num;
  Serial.println(saveMoney);
  while(saveMoney>=255)
  { 
    saveMoney=num-255;
    a++;
    Serial.println(saveMoney);
   }
  Serial.println(a);
  while(a > 0  && saveMoney < 255)
  {
   writeData[15-b]=255; 
   b++;
   a--;
    }
  Serial.println(b);
  writeData[15-b]=saveMoney; 
  for(int k=0;k<16;k++)
  {
    Serial.print(writeData[k]);
    Serial.print(" ");
    }  
  Serial.println(" ");
 } 




  
  
status = MFRC522_Request(PICC_REQIDL, str); 
    if (status == MI_OK)
    {
      Serial.println("Find out a card ");
      Serial.print(str[0],BIN);
      Serial.print(" , ");
      Serial.print(str[1],BIN);
      Serial.println(" ");
    }

    // Anti-collision, return the card's 4-byte serial number
    status = MFRC522_Anticoll(str);
    memcpy(serNum, str, 5);
    if (status == MI_OK)
    {

      Serial.println("The card's number is  : ");
      Serial.print(serNum[0],HEX);      
      Serial.print(serNum[1],HEX);      
      Serial.print(serNum[2],HEX);     
      Serial.print(serNum[3],HEX);
      Serial.println(serNum[4],HEX);
    }

    // Election card, return capacity
    RC_size = MFRC522_SelectTag(serNum);
    if (RC_size != 0)

    {
      Serial.print("The size of the card is  :   ");
      Serial.print(RC_size,DEC);
      Serial.println(" K ");
    }
                
    // Registration card
    blockAddr = 11;   // Data block 11    
    status = MFRC522_Auth(PICC_AUTHENT1A, blockAddr, sectorKeyA[blockAddr/4], serNum);  // Authentication
    if (status == MI_OK)
    {
      // Write data
     /* status = MFRC522_Write(blockAddr, sectorNewKeyA[blockAddr/4]);
                        Serial.print("set the new card password, and can modify the data of the Sector ");
                        Serial.print(blockAddr/4,DEC);
                        Serial.println(" : ");
      for (i=0; i<6; i++)
            {
                      Serial.print(sectorNewKeyA[blockAddr/4][i],HEX);
                      Serial.print(" , ");
            }
                        Serial.println(" ");*/
                        blockAddr = blockAddr - 3 ; 
                        status = MFRC522_Write(blockAddr, writeData);
                        if(status == MI_OK)
                        {
                           Serial.print("You are B2CQSHOP VIP Member, The card has  $");
                           Serial.println(num);
                        }
    }
// Card reader
    blockAddr = 11;   // Data block 11    
    status = MFRC522_Auth(PICC_AUTHENT1A, blockAddr, sectorNewKeyA[blockAddr/4], serNum); // Authentication
    if (status == MI_OK)
    {
      // Read data
                        blockAddr = blockAddr - 3 ; 
                        status = MFRC522_Read(blockAddr, str);
      if (status == MI_OK)
      {
                          Serial.println("Read from the card ,the data is : ");
        for (i=0; i<16; i++)
        {
                          Serial.print(str[i],DEC);
                          Serial.print(" , ");
        }
                          Serial.println(" ");
      }
    }

                // Consumer 
    blockAddr = 11;   // Data block 11    
    status = MFRC522_Auth(PICC_AUTHENT1A, blockAddr, sectorNewKeyA[blockAddr/4], serNum); // Authentication
    if (status == MI_OK)
    {
      // Read data
      blockAddr = blockAddr - 3 ;
      status = MFRC522_Read(blockAddr, str);
      if (status == MI_OK)
      {
                          if( str[15] < moneyConsume )
                          {
                              Serial.println(" The money is not enough !");
                          }
                          else
                          {
                              num = num - moneyConsume; 
                              status = MFRC522_Write(blockAddr, str);
                              if(status == MI_OK)
                              {
                                 Serial.print("You pay $20 for items in B2CQSHOP.COM . Now, Your money balance is :   $");
                                 Serial.print(num);
                                 Serial.println(" ");
                              }
                          }
      }
    }
   Serial.println(" ");
   MFRC522_Halt();











 switch(state){
    case STATE_START:{
      reset();
      playMelody(MELODY_START);
      state = STATE_QUESTION;
      break;
    }
    
    case STATE_QUESTION:{
for(byte j=0; j<4; j++){
 for(byte i=0; i<8; i++)
{
  max7219(i+1,sprite[j][i]);
  }
  delay(200);
 }
      
      questions = (int *)(malloc(sizeof(int) * q_num));
      answers = (int *)(malloc(sizeof(int) * q_num));
      for(int i = 0; i < q_num; i++){
        questions[i] = random(0, NUMBER);
      }
      answer_num = 0;
      playQuestionsTone();
      lastClickTime = millis();
      state = STATE_ANSWER;
      break;
    }
    
    case STATE_ANSWER:{
      const unsigned long nowTime = millis();
      if(nowTime >= lastClickTime + 10000UL){
        state = STATE_WRONG;
        break;
      }
      
      for(int i = 0; i < NUMBER; i++){
        int ss = digitalRead(switches[i]);
        if(ss == LOW){
          digitalWrite(leds[i], HIGH);
          lastClickTime = nowTime;
          answers[answer_num] = i;
          answer_num++;
          playOneTone(notes[i], 1);
          digitalWrite(leds[i], LOW);
          delay(200);
          break;
        }
        
      }
      
      if(answer_num >= q_num){
        state = check() ? STATE_CORRECT : STATE_WRONG;
      }
      break;
    }
    
    case STATE_CORRECT:{
      q_num++;
      playMelody(MELODY_CORRECT);
      totscore = totscore+score;
for(byte i=0; i<8; i++)
{
  max7219(i+1,win[i]);
  }
      delay(2000);
      state = STATE_START;
      break;
    }
    
    case STATE_WRONG:{
      score = 0;
      playMelody(MELODY_WRONG); 
for(byte i=0; i<8; i++)
{
  max7219(i+1,lose[i]);
  }
      Serial.print("Score: ");
      Serial.println(totscore);
      delay(2000);
      state = STATE_START;
      break;
    }
    
    default:{
      state = STATE_START;
      break;
    }
  }
}












/*
 * Function：Write_MFRC5200
 * Description：Write a byte of data to MFRC522 register
 * Input parameters：addr--register address；val--value to be written
 * Return value: 
 */
void Write_MFRC522(uchar addr, uchar val)
{
  digitalWrite(chipSelectPin, LOW);

  // Address format：0XXXXXX0
  SPI.transfer((addr<<1)&0x7E); 
  SPI.transfer(val);
  
  digitalWrite(chipSelectPin, HIGH);
}


/*
 * Function：Read_MFRC522
 * Description：Read a byte of data from MFRC522 register
 * Input parameters：addr--register address
 * Return value: Returns a byte of data
 */
uchar Read_MFRC522(uchar addr)
{
  uchar val;

  digitalWrite(chipSelectPin, LOW);

  // Address format：1XXXXXX0
  SPI.transfer(((addr<<1)&0x7E) | 0x80);  
  val =SPI.transfer(0x00);
  
  digitalWrite(chipSelectPin, HIGH);
  
  return val; 
}

/*
 * Function：SetBitMask
 * Description：Set RC522 register bit
 * Input parameters：reg--register address;mask--set value
 * Return value:
 */
void SetBitMask(uchar reg, uchar mask)  
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp | mask);  // set bit mask
}


/*
 * Function：ClearBitMask
 * Description：Clear RC522 register bit
 * Input parameters：reg--register address;mask--clear bit value
 * Return value:
 */
void ClearBitMask(uchar reg, uchar mask)  
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp & (~mask));  // clear bit mask
} 


/*
 * Function：AntennaOn
 * Description：Power up ready for use, minimum 1ms interval between on/off cycle
 * Input values:
 * Return value:
 */
void AntennaOn(void)
{
  uchar temp;

  temp = Read_MFRC522(TxControlReg);
  if (!(temp & 0x03))
  {
    SetBitMask(TxControlReg, 0x03);
  }
}


/*
 * Function：AntennaOff
 * Description：Power down after use, minimum 1ms interval between on/off cycle
 * Input values: 
 * Output value:
 */
void AntennaOff(void)
{
  ClearBitMask(TxControlReg, 0x03);
}


/*
 * Function：ResetMFRC522
 * Description:Resets the RC522
 * Input values：
 * Output value：
 */
void MFRC522_Reset(void)
{
    Write_MFRC522(CommandReg, PCD_RESETPHASE);
}


/*
 * Function：InitMFRC522
 * Description：初始化RC522
 * Input values：
 * Output value：
 */
void MFRC522_Init(void)
{
  digitalWrite(NRSTPD,HIGH);

  MFRC522_Reset();
    
  //Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
    Write_MFRC522(TModeReg, 0x8D);    //Tauto=1; f(Timer) = 6.78MHz/TPreScaler
    Write_MFRC522(TPrescalerReg, 0x3E); //TModeReg[3..0] + TPrescalerReg
    Write_MFRC522(TReloadRegL, 30);           
    Write_MFRC522(TReloadRegH, 0);
  
   Write_MFRC522(TxAutoReg, 0x40);   //100%ASK
   Write_MFRC522(ModeReg, 0x3D);   //CRC initial value of 0x6363 ???

  //ClearBitMask(Status2Reg, 0x08);   //MFCrypto1On=0
  //Write_MFRC522(RxSelReg, 0x86);    //RxWait = RxSelReg[5..0]
  //Write_MFRC522(RFCfgReg, 0x7F);      //RxGain = 48dB

  AntennaOn();    // Power on the antenna
}


/*
 * Function：MFRC522_Request
 * Description：Find card and read its type
 * Input values：reqMode--way to find the card;
 *       TagType--return type of card:
 *        0x4400 = Mifare_UltraLight
 *        0x0400 = Mifare_One(S50)
 *        0x0200 = Mifare_One(S70)
 *        0x0800 = Mifare_Pro(X)
 *        0x4403 = Mifare_DESFire
 * Output value：successful return MI_OK
 */
uchar MFRC522_Request(uchar reqMode, uchar *TagType)
{
  uchar status;  
  uint backBits;      // The received data bits

  Write_MFRC522(BitFramingReg, 0x07);   //TxLastBists = BitFramingReg[2..0] ???
  
  TagType[0] = reqMode;
  status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

  if ((status != MI_OK) || (backBits != 0x10))
  {    
    status = MI_ERR;
  }
   
  return status;
}


/*
 * Function：MFRC522_ToCard
 * Description：RC522 and ISO14443 card communication
 * Input values：command--MF522 command，
 *       sendData--RC522 data, 
 *       sendLen--Send data length     
 *       backData--Received data is returns，
 *       backLen--Returns the length of the data bits
 * Output value：Successful return MI_OK
 */
uchar MFRC522_ToCard(uchar command, uchar *sendData, uchar sendLen, uchar *backData, uint *backLen)
{
    uchar status = MI_ERR;
    uchar irqEn = 0x00;
    uchar waitIRq = 0x00;
    uchar lastBits;
    uchar n;
    uint i;

    switch (command)
    {
        case PCD_AUTHENT:   // certification card secret
    {
      irqEn = 0x12;
      waitIRq = 0x10;
      break;
    }
    case PCD_TRANSCEIVE:  // Transmit FIFO data
    {
      irqEn = 0x77;
      waitIRq = 0x30;
      break;
    }
    default:
      break;
    }
   
    Write_MFRC522(CommIEnReg, irqEn|0x80);  // Allow the interrupt request
    ClearBitMask(CommIrqReg, 0x80);     // Clear interrupt request bit
    SetBitMask(FIFOLevelReg, 0x80);     //FlushBuffer=1, initialise FIFO
    
  Write_MFRC522(CommandReg, PCD_IDLE);  //No action; cancel the current command ???

  // Write data to FIFO
    for (i=0; i<sendLen; i++)
    {   
    Write_MFRC522(FIFODataReg, sendData[i]);    
  }

  // Execute the command
  Write_MFRC522(CommandReg, command);
    if (command == PCD_TRANSCEIVE)
    {    
    SetBitMask(BitFramingReg, 0x80);    //StartSend=1,transmission of data starts  
  }   
    
  // Wait to receive data
  i = 2000; //i is for clock frequency adjustment, M1 card maximum waiting time is 25ms ???
    do 
    {
    //CommIrqReg[7..0]
    //Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
        n = Read_MFRC522(CommIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitIRq));

    ClearBitMask(BitFramingReg, 0x80);      //StartSend=0
  
    if (i != 0)
    {    
        if(!(Read_MFRC522(ErrorReg) & 0x1B))  //BufferOvfl Collerr CRCErr ProtecolErr
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {   
        status = MI_NOTAGERR;     //??   
      }

            if (command == PCD_TRANSCEIVE)
            {
                n = Read_MFRC522(FIFOLevelReg);
                lastBits = Read_MFRC522(ControlReg) & 0x07;
                if (lastBits)
                {   
          *backLen = (n-1)*8 + lastBits;   
        }
                else
                {   
          *backLen = n*8;   
        }

                if (n == 0)
                {   
          n = 1;    
        }
                if (n > MAX_LEN)
                {   
          n = MAX_LEN;   
        }
        
        // Read received FIFO data
                for (i=0; i<n; i++)
                {   
          backData[i] = Read_MFRC522(FIFODataReg);    
        }
            }
        }
        else
        {   
      status = MI_ERR;  
    }
        
    }
  
    //SetBitMask(ControlReg,0x80);           //timer stops
    //Write_MFRC522(CommandReg, PCD_IDLE); 

    return status;
}


/*
 * Function：MFRC522_Anticoll
 * Description：Anti-collision detection and card serial number reading
 * Input values：serNum--return the four byte card serial number, the first five bytes of the checksum
 * Output value：Successful return MI_OK
 */
uchar MFRC522_Anticoll(uchar *serNum)
{
    uchar status;
    uchar i;
  uchar serNumCheck=0;
    uint unLen;
    

    //ClearBitMask(Status2Reg, 0x08);   //TempSensclear
    //ClearBitMask(CollReg,0x80);     //ValuesAfterColl
  Write_MFRC522(BitFramingReg, 0x00);   //TxLastBists = BitFramingReg[2..0]
 
    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == MI_OK)
  {
    // Check card serial number
    for (i=0; i<4; i++)
    {   
      serNumCheck ^= serNum[i];
    }
    if (serNumCheck != serNum[i])
    {   
      status = MI_ERR;    
    }
    }

    //SetBitMask(CollReg, 0x80);    //ValuesAfterColl=1

    return status;
} 


/*
 * Function：CalulateCRC
 * Description：Calculate MF522 CRC
 * Input values:：pIndata--read CRC data,len--length of data,pOutData--calculated CRC results
 * Output value：
 */
void CalulateCRC(uchar *pIndata, uchar len, uchar *pOutData)
{
    uchar i, n;

    ClearBitMask(DivIrqReg, 0x04);      //CRCIrq = 0
    SetBitMask(FIFOLevelReg, 0x80);     // Clear FIFO pointer
    //Write_MFRC522(CommandReg, PCD_IDLE);

  // Write data to FIFO 
    for (i=0; i<len; i++)
    {   
    Write_MFRC522(FIFODataReg, *(pIndata+i));   
  }
    Write_MFRC522(CommandReg, PCD_CALCCRC);

  // Wait for CRC calculation
    i = 0xFF;
    do 
    {
        n = Read_MFRC522(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));      //CRCIrq = 1

  // Read CRC calcuation results
    pOutData[0] = Read_MFRC522(CRCResultRegL);
    pOutData[1] = Read_MFRC522(CRCResultRegM);
}


/*
 * Function：MFRC522_SelectTag
 * Description：Election card, read memory capacity
 * Input values：serNum--incoming card serial number
 * Output value：Successful return of card capacity
 */
uchar MFRC522_SelectTag(uchar *serNum)
{
    uchar i;
    uchar status;
    uchar size;
    uint recvBits;
    uchar buffer[9]; 

  //ClearBitMask(Status2Reg, 0x08);     //MFCrypto1On=0

    buffer[0] = PICC_SElECTTAG;
    buffer[1] = 0x70;
    for (i=0; i<5; i++)
    {
      buffer[i+2] = *(serNum+i);
    }
  CalulateCRC(buffer, 7, &buffer[7]);   //??
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);
    
    if ((status == MI_OK) && (recvBits == 0x18))
    {   
    size = buffer[0]; 
  }
    else
    {   
    size = 0;    
  }

    return size;
}


/*
 * Function：MFRC522_Auth
 * Description：Verify card password
 * Input values：authMode--Passowrd authentication mode
                 0x60 = Verify A key
                 0x61 = Verify B key
             BlockAddr--Block address
             Sectorkey--Sector's password
             serNum--Card serial number, 4 bytes
 * Output value：Successful return MI_OK
 */
uchar MFRC522_Auth(uchar authMode, uchar BlockAddr, uchar *Sectorkey, uchar *serNum)
{
    uchar status;
    uint recvBits;
    uchar i;
  uchar buff[12]; 

  // Verify instruction block address, sector's password and card serial number
    buff[0] = authMode;
    buff[1] = BlockAddr;
    for (i=0; i<6; i++)
    {    
    buff[i+2] = *(Sectorkey+i);   
  }
    for (i=0; i<4; i++)
    {    
    buff[i+8] = *(serNum+i);   
  }
    status = MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

    if ((status != MI_OK) || (!(Read_MFRC522(Status2Reg) & 0x08)))
    {   
    status = MI_ERR;   
  }
    
    return status;
}


/*
 * Function：MFRC522_Read
 * Description：Read block data
 * Input values：blockAddr--block address;recvData--read a block of data
 * Output value：Successful return MI_OK
 */
uchar MFRC522_Read(uchar blockAddr, uchar *recvData)
{
    uchar status;
    uint unLen;

    recvData[0] = PICC_READ;
    recvData[1] = blockAddr;
    CalulateCRC(recvData,2, &recvData[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

    if ((status != MI_OK) || (unLen != 0x90))
    {
        status = MI_ERR;
    }
    
    return status;
}


/*
 * Function：MFRC522_Write
 * Description：Write block data
 * Input values：blockAddr--block address;writeData--16 bytes of data to be written to block
 * Output value：Successful return MI_OK
 */
uchar MFRC522_Write(uchar blockAddr, uchar *writeData)
{
    uchar status;
    uint recvBits;
    uchar i;
    uchar buff[18]; 
    
    buff[0] = PICC_WRITE;
    buff[1] = blockAddr;
    CalulateCRC(buff, 2, &buff[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
    {   
    status = MI_ERR;   
  }
        
    if (status == MI_OK)
    {
        for (i=0; i<16; i++)    //Write 16 bytes of data to the FIFO
        {    
          buff[i] = *(writeData+i);   
        }
        CalulateCRC(buff, 16, &buff[16]);
        status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);
        
    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
        {   
      status = MI_ERR;   
    }
    }
    
    return status;
}


/*
 * Function：MFRC522_Halt
 * Description： Send card to sleep
 * Input values：
 * Output value：
 */
void MFRC522_Halt(void)
{
    uchar status;
    uint unLen;
    uchar buff[4]; 

    buff[0] = PICC_HALT;
    buff[1] = 0;
    CalulateCRC(buff, 2, &buff[2]);
 
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff,&unLen);
}

