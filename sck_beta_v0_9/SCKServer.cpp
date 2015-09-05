/*

  SCKServer.cpp
  
*/


#include "Constants.h"
#include "SCKServer.h"
#include "SCKBase.h"
#include "SCKAmbient.h"
#include <Wire.h>
#include <EEPROM.h>

#define debugServer   true

SCKBase base__;
SCKServer server__;
SCKAmbient ambient__;

#define TIME_BUFFER_SIZE 20 

boolean SCKServer::time(char *time_) 
{
  boolean ok=false;
  uint8_t count = 0;
  byte retry=0;
  while (retry<5)
  {
   retry++;
   if (base__.enterCommandMode()) 
    {
      if (base__.open(WEB[0][0], 80))
       {
        for(byte i = 0; i<3; i++) Serial1.print(WEBTIME[i]); //Requests to the server time
        if (base__.findInResponse("UTC:", 2000)) 
          {
              char newChar;
              byte offset = 0;
              unsigned long time = millis();
              while (offset < TIME_BUFFER_SIZE) {
                if (Serial1.available())
                {
                  newChar = Serial1.read();
                  time = millis();
                  if (newChar == '#') {
                    ok = true;
                    time_[offset] = '\x00';
                    return ok;
                  } 
                  else if (newChar != -1) {
                    if (newChar==',') 
                    {
                      if (count<2) time_[offset]='-';
                      else if (count>2) time_[offset]=':';
                      else time_[offset]=' ';
                      count++;
                    }
                    else time_[offset] = newChar;
                    offset++;
                  }
                }
                else if((millis()-time)>1000)
                {
                  ok = false;
                  break; 
                }
              }
           }
         base__.close();
       }
    }
  }
  time_[0] = '#';
  time_[1] = 0x00;
  base__.exitCommandMode();
  return ok;
}

void SCKServer::json_update(uint16_t updates, long *value, char *time, boolean isMultipart,byte numWEB)
{  
    #if debugServer
       Serial.print(F("["));
    #endif
    Serial1.print(F("["));  
    for (int i = 0; i< updates;i++)
      {
        readFIFO(numWEB);
        if ((i< (updates - 1)) || (isMultipart))
          {
            Serial1.print(F(","));
            #if debugServer
              Serial.print(F(","));
            #endif
          }
      }
 
    if (isMultipart)
     {
        byte i;
        for (i = 0; i<SENSORS; i++)
          {
            Serial1.print(SERVER[i]);
            Serial1.print(value[i]); 
          }  
        Serial1.print(SERVER[i]);  
        Serial1.print(time);
        Serial1.print(SERVER[i+1]);
        Serial1.println(F("]"));
        Serial1.println();
        
        #if debugServer
           for (i = 0; i<SENSORS; i++)
             {
               Serial.print(SERVER[i]);
               Serial.print(value[i]);
             }
           Serial.print(SERVER[i]);
           Serial.print(time);
           Serial.print(SERVER[i+1]);
        #endif
     }
    Serial1.println(F("]"));
    Serial1.println();
    #if debugServer
       Serial.println(F("]"));
    #endif
}  

void SCKServer::addFIFO(byte numWEB, long *value, char *time)
{
  int eeaddress = base__.readData(EE_ADDR_NUMBER_WRITE_MEASURE+(4*numWEB), INTERNAL);
  
  for (int i = 0; i<SENSORS; i++)
    {
      base__.writeData(eeaddress, value[i], EXTERNAL);
      eeaddress = eeaddress + (4*WEBS);
    } 
  base__.writeData(eeaddress, 0, time, EXTERNAL);
  eeaddress = eeaddress + (TIME_BUFFER_SIZE*WEBS);
  base__.writeData(EE_ADDR_NUMBER_WRITE_MEASURE+(4*numWEB), eeaddress, INTERNAL);
}


void SCKServer::readFIFO(byte numWEB)
{   
  int i = 0;
  int eeaddress = base__.readData(EE_ADDR_NUMBER_READ_MEASURE + (4*numWEB), INTERNAL); //
  for (i = 0; i<SENSORS; i++)
    {
      Serial1.print(SERVER[i]);
      Serial1.print(base__.readData(eeaddress + (i*4*WEBS), EXTERNAL)); //SENSORS
    }  
  Serial1.print(SERVER[i]);  
  Serial1.print(base__.readData(eeaddress + (i*4*WEBS), 0, EXTERNAL)); //TIME
  Serial1.print(SERVER[i+1]);
  
  #if debugServer
    for (i = 0; i<SENSORS; i++)
     {
       Serial.print(SERVER[i]);
       Serial.print(base__.readData(eeaddress + (i*4*WEBS), EXTERNAL)); //SENSORS
     }
    Serial.print(SERVER[i]);
    Serial.print(base__.readData(eeaddress + (i*4*WEBS), 0, EXTERNAL)); //TIME
    Serial.print(SERVER[i+1]);
  #endif

  eeaddress = eeaddress + (((SENSORS*4) + TIME_BUFFER_SIZE) * WEBS);
  if (eeaddress == base__.readData(EE_ADDR_NUMBER_WRITE_MEASURE + (4*numWEB), INTERNAL))
    {
      base__.writeData(EE_ADDR_NUMBER_WRITE_MEASURE + (4*numWEB), 0, INTERNAL);
      base__.writeData(EE_ADDR_NUMBER_READ_MEASURE + (4*numWEB), 0, INTERNAL);
    }
  else base__.writeData(EE_ADDR_NUMBER_READ_MEASURE + (4*numWEB), eeaddress, INTERNAL);
}  
  
#define numbers_retry 5

boolean SCKServer::update(long *value, char *time_)
{
  value[8] = base__.scan();  //Wifi Nets
  byte retry = 0;
  if (time(time_)) //Update server time
  {  
    if (base__.checkRTC())
    {
      while (!base__.RTCadjust(time_)&&(retry<numbers_retry)) retry++;
    }
  }
  else if (base__.checkRTC())
         {
           base__.RTCtime(time_);
           #if debugEnabled
              if (!ambient__.debug_state()) Serial.println(F("Fail server time!!"));
           #endif
         }
  else 
    {
      time_ = "#";
      return false;
    }
  return true; 
}

boolean SCKServer::connect(char *theWEB[8])
{
  int retry = 0;
  while (true){
    if (base__.open(theWEB[0], 80)) break;
    else 
    {
      retry++;
      if (retry >= numbers_retry) return false;
    }
  }    
  for (byte i = 1; i<5; i++) Serial1.print(theWEB[i]);
  Serial1.println(base__.readData(EE_ADDR_MAC, 0, INTERNAL)); //MAC ADDRESS
  Serial1.print(theWEB[5]);
  Serial1.println(base__.readData(EE_ADDR_APIKEY, 0, INTERNAL)); //Apikey
  Serial1.print(theWEB[6]);
  Serial1.println(FirmWare); //Firmware version
  Serial1.print(theWEB[7]);
  return true; 
}


void SCKServer::send(boolean sleep, boolean *wait_moment, long *value, char *time) {  
  *wait_moment = true;
  uint16_t updates;
  uint16_t NumUpdates;
  int num_post=0;
  int cycles=0;
  for(byte j = 0; j < WEBS ; j++)
  {
    updates = (base__.readData(EE_ADDR_NUMBER_WRITE_MEASURE+(4*j), INTERNAL)-base__.readData(EE_ADDR_NUMBER_READ_MEASURE+(4*j), INTERNAL))/(((SENSORS)*4 + TIME_BUFFER_SIZE) * WEBS);
    NumUpdates = base__.readData(EE_ADDR_NUMBER_UPDATES, INTERNAL); // Number of readings before batch update
    if (updates>=(NumUpdates - 1))
      { 
        if (sleep and j==0)
          {
            #if debugEnabled
                if (!ambient__.debug_state()) Serial.println(F("SCK Waking up..."));
            #endif
            digitalWrite(AWAKE, HIGH);
          }
        if (base__.connect())  //Wifi connect
          {
            #if debugEnabled
                if (!ambient__.debug_state()) Serial.println(F("SCK Connected!!")); 
            #endif   
            if (update(value, time)) //Update time and nets
            {
                #if debugEnabled
                    if (!ambient__.debug_state())
                    {
                      Serial.print(F("updates = "));
                      Serial.println(updates + 1);
                    }
                #endif
                num_post = updates;
                cycles = updates/POST_MAX;
                if (updates > POST_MAX) 
                  {
                    for (int i=0; i<cycles; i++)
                    {
                      connect(WEB[j]);
                      json_update(POST_MAX, value, time, false, j);
                    }
                    num_post = updates - cycles*POST_MAX;
                  }
                connect(WEB[j]);
                json_update(num_post, value, time, true, j);
                #if debugEnabled
                      if (!ambient__.debug_state()) 
                        {
                           Serial.print(F("Posted to Server "));
                           Serial.println(WEB[j][0]);
                        }
                #endif            
            }
            else 
            {
              #if debugEnabled
                  if (!ambient__.debug_state())
                  {
                    Serial.println(F("Error updating time Server..!"));
                  }
              #endif
            }
            #if debugEnabled
                if (!ambient__.debug_state()) Serial.println(F("Closing active connection..."));
            #endif
            base__.close();
          }
        else //No connect
          {
            addFIFO(j, value, time);
            #if debugEnabled
                if (!ambient__.debug_state()) Serial.println(F("Error in connection!!"));
            #endif
          }
        if (sleep and j==WEBS-1)
          {
            base__.sleep();
            #if debugEnabled
                if (!ambient__.debug_state())
                {
                  Serial.println(F("SCK Sleeping")); 
                  Serial.println(F("*******************"));
                }
            #endif
            digitalWrite(AWAKE, LOW); 
         }
      }
    else
      {
          //value[8] = 0;  //Wifi Nets
          if (base__.checkRTC()) base__.RTCtime(time);
          else time = "#";
          addFIFO(j,value, time);
          #if debugEnabled
            if (!ambient__.debug_state()) Serial.println(F("Saved in memory!!"));
          #endif
      }
  }
  *wait_moment = false;
}

