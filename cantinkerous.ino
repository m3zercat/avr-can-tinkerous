#include <heartbeat.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>

#define cereal Serial
#define serialWriteLed 23
#define canReadLed 8
#define serialReadLed 9
#define canWriteLed 10
#define multiLedA A2
#define multiLedB 11

unsigned int rxId;
unsigned char rxLen = 0;
unsigned char rxBuf[8];
volatile unsigned char canData = 0;

String input = "";

MCP_CAN CAN0(17);           // Set CS to pin 17

unsigned char stmp[8] = {0, 1, 2, 3, 4, 5, 6, 7};

bool canAvailable = false;

Heartbeat *statusHb = new Heartbeat(multiLedA);

void setup()
{
   input.reserve(200);
  
   pinMode(A1, OUTPUT);
   pinMode(A0, OUTPUT);
   pinMode(12, OUTPUT);
   digitalWrite(A1, LOW);
   digitalWrite(A0, LOW);
   digitalWrite(12, LOW);
   pinMode(serialWriteLed, OUTPUT);
   pinMode(canReadLed, OUTPUT);
   pinMode(serialReadLed, OUTPUT);
   pinMode(canWriteLed, OUTPUT);
   pinMode(multiLedA, OUTPUT);
   pinMode(multiLedB, OUTPUT);
   
   digitalWrite(multiLedA, HIGH);
   delay(3000);
   digitalWrite(multiLedA, LOW);
   cereal.begin(9600);
   sprintline("CANCAN");
   // init can bus, baudrate: 500k
   while(!canAvailable)
   {
       if(CAN0.begin(CAN_500KBPS) == CAN_OK)
       {
           sprintline("CAN OK");
           attachInterrupt(4, can_receive, FALLING);      // Pin D7 is Interrupt4 on Leonardo
           canAvailable = true;
           clearError();
       }
       else
       {
           sprintline("CAN FAIL");
           setError();
           delay(1000);
       }
   }
}

void sprint(String str)
{
    digitalWrite(serialWriteLed, HIGH);
    cereal.print(str);
    digitalWrite(serialWriteLed, LOW);
}

void sprint(int in, int mode)
{
    digitalWrite(serialWriteLed, HIGH);
    cereal.print(in, mode);
    digitalWrite(serialWriteLed, LOW);
}

void sprintline()
{
    digitalWrite(serialWriteLed, HIGH);
    cereal.print("\r\n");
    digitalWrite(serialWriteLed, LOW);
}

void sprintline(String str2)
{
    sprint(str2);
    sprintline();
}

void setError()
{
    digitalWrite(multiLedB, HIGH);
}

void clearError()
{
    digitalWrite(multiLedB, LOW);
}

void loop()
{
   statusHb->Pulse();
   checkCan();
   checkSerial();
   try_clear_faults();
}

void can_send()
{   // send data:  id = 0x00, standard frame, data len = 8, stmp: data buf
   digitalWrite(canWriteLed, HIGH);   // turn the LED on   
   CAN0.sendMsgBuf(0x00, 0, 8, stmp);  
   delay(100);
   digitalWrite(canWriteLed, LOW);   // turn the LED off  
   delay(900);// send data every second
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

bool cleared_faults = false;

void try_clear_faults()
{
    if(millis()>61000)
    {
        digitalWrite(canWriteLed, LOW);
        return;
    }
    if(millis()<60000 || cleared_faults)
    {
        return;
    }
    digitalWrite(canWriteLed, HIGH);
    CAN0.sendMsgBuf(0x04, 0, 0, stmp);
    cleared_faults = true;
}

void runCommand(String command)
{
    if(command == "reset")
    {
        resetFunc();
    }
    else
    {
        sprintline("Command Unrecognized: "+command);
    }
}

void checkSerial()
{
    while(cereal.available())
    {
        digitalWrite(serialReadLed, HIGH);
        char inChar = (char)cereal.read();
        if(inChar == '\n')
        {
            runCommand(input);
            input = "";
        }
        if(inChar == '\r')
        {
            // do nothing
        }
        else
        {
            input += inChar;
        }
        digitalWrite(serialReadLed, LOW);
    }
}

void checkCan()
{
   if(canData)
   {
      digitalWrite(canReadLed, HIGH);
      canData=0;
      CAN0.readMsgBuf(&rxLen, rxBuf);            // Read data: len = data length, buf = data byte(s)
      rxId = CAN0.getCanId();                    // Get message ID
      digitalWrite(canReadLed, LOW);

      sprint("ID: ");
      sprint(rxId, HEX);
      sprint("  Data: ");
      for(int i = 0; i < rxLen; i++)                // Print each byte of the data
      {
         if(rxBuf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
         {
            sprint("0");
         }
         sprint(rxBuf[i], HEX);
         sprint(" ");
      }
      sprintline();
   }
}

void can_receive()
{
   // CAN Receive Interrupt  
   canData = 1;
}


