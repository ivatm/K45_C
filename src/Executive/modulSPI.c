/*
 * modulSPI.c
 *
 *  Communication to additional module RealTime operations
 *
 *  Created on: 23 זמגע. 2020 נ.
 *      Author: Oliva
 */

#include <stdint.h>
#include <time.h>
#include <math.h>
#include <bcm2835.h>
#include <stdio.h>

#include "modulSPI.h"
#include "defines.h"
#include "ThermoControlHeaders.h"
#include "globalvarheads.h"

// extern procedure ---------------------------------------------------------------------------

extern uint16_t GetHeaterSetpoint(void);
extern uint16_t GetCoolerSetpoint(void);
extern uint_fast32_t deltaMilis(struct timespec* specStatic);

// global procedure ---------------------------------------------------------------------------
void executeModulControl(void);

//Local procedures ----------------------------------------------------------------------------
uint16_t LimitPWM(uint16_t Input);

 /* Transmit message structure
 ByteNumber  | Byte 0   |  Byte 1       | Byte 2       |  Byte 3     | Byte 4   | Byte 5   |
 ByteValue   |  '>'     |   nn          |   dd         |   dd        |   chck   |   '<'    |
 ByteMeaning | StartSym |  Value Number | Value Byte 0 | Value Byte 1| CheckSum | EndSym   |
  */

 /* Received message structure
 ByteNumber  | Byte 0   |  Byte 1       | Byte 2       |  Byte 3     |   Byte 4     | Byte 5   |
 ByteValue   |  xx      |   '>'         |   nn         |   dd        |     dd       | chck     |
 ByteMeaning | Free     |  StartSym     | Value Number | Value Byte 0| Value Byte 1 | CheckSum |
  */

void Module_GPIOConfig(void);
uint8_t transferAndWait(const uint8_t what);
void transferMsG(uint8_t* pMsgRec , uint8_t* pMsgTr);
uint8_t Tx_CheckSumCalcul(fExecutive_TxObj_Union* pTransmitObject);
uint8_t Rx_CheckSumCalcul(fExecutive_RxObj_Union* pReceiveObject);
boolean ProcessReceived(fExecutive_RxObj_Union* pReceivedObject);

uint8_t RecBuf[kMsgLength];                           // Reception array
uint8_t TransmBuf[kMsgLength] = {'>',0,0,3,0,'<'};    // Transmission array


void Module_GPIOConfig(void)
{
    //output
    bcm2835_gpio_fsel(MODUL_ISP_PIN, BCM2835_GPIO_FSEL_OUTP);
}


/*
  Procedure to transfer/receive 1 uint8_t
*/
uint8_t transferAndWait(const uint8_t what)
{
  uint8_t a = bcm2835_aux_spi_transfer(what);
  delay (1);
  return a;
} // end of transferAndWait

/*
  Procedure to transfer/receive Message
*/
void transferMsG(uint8_t* pMsgRec , uint8_t* pMsgTr)
{
  bcm2835_gpio_clr(MODUL_ISP_PIN);

  (void)transferAndWait(*pMsgTr); // First uint8_t without receiving

  //Serial.print(*pMsgTr); Serial.print("; "); Serial.println(*pMsgRec);
///  delay (1);
  pMsgTr++;
  for (uint8_t Index = 1; Index < kMsgLength; Index++, pMsgRec++, pMsgTr++)
  {
    *pMsgRec = transferAndWait(*pMsgTr);
    //Serial.print(*pMsgTr); Serial.print("; "); Serial.println(*pMsgRec);
  }
  *pMsgRec = transferAndWait(77);// Last uint8_t only for receiving

  bcm2835_gpio_set(MODUL_ISP_PIN);

  return;
}

/* boolean setExecuteModule(void)
 * The function prepares
 * */
boolean setExecuteModule(void)
{
   fExecutive_TxObj_Union fLocalTxObject;
   fExecutive_RxObj_Union fLocalRxObject;

   uint16_t iLocalWork;

   boolean Result;

   // FALSE if error
   Result = FALSE;

   for (iLocalWork = 0; iLocalWork < kMsgLength; iLocalWork++)
   {
      fLocalTxObject.cTxData[iLocalWork] = 0;
      fLocalRxObject.cRxData[iLocalWork] = 0;
   }

   if (eSystemState != keIdle)
   {
      // If the State is not "Idle" the setting is allowed
      fLocalTxObject.TxObj.FlagCommand.sFlagCommand.bStartSetting = TRUE;
   }
   else
   {
      fLocalTxObject.TxObj.FlagCommand.sFlagCommand.bStartSetting = FALSE;
   }

   iLocalWork = GetHeaterSetpoint();

   fLocalTxObject.TxObj.HeaterCommand = LimitPWM(iLocalWork);

   iLocalWork = GetCoolerSetpoint();

   fLocalTxObject.TxObj.CoolerCommand = LimitPWM(iLocalWork);

#ifdef VALVE_CONTROL
   iLocalWork = GetCoolerSetpoint();
   fLocalTxObject.TxObj.ValveCommand = LimitPWM(iLocalWork);

#endif

   fLocalTxObject.TxObj.startSymbol = '>';
   fLocalTxObject.TxObj.checkSum = Tx_CheckSumCalcul(&fLocalTxObject);
   fLocalTxObject.TxObj.stopSymbol = '<';

   transferMsG(&fLocalRxObject.cRxData[0], &fLocalTxObject.cTxData[0]);

   Result = ProcessReceived(&fLocalRxObject);

#ifdef debugmode_SPI
   //printf("Communication state --------------------------\r\n");
   for (iLocalWork = 0; iLocalWork < kMsgLength; iLocalWork++)
   {
      //printf("%hhx",fLocalTxObject.cTxData[iLocalWork]);//printf(";");
   }
   //printf("\r\n");

   for (iLocalWork = 0; iLocalWork < kMsgLength; iLocalWork++)
   {
      //printf("%hhx",fLocalRxObject.cRxData[iLocalWork]);//printf(";");
   }
   //printf("\r\n");

   //printf("SetMode = %hhx\r\n",fPowerModulStatus.sStatus.bSetMode);
   //printf("bErrorValue1 = %hhx\r\n",fPowerModulStatus.sStatus.bErrorValue1);
   //printf("bErrorValue2 = %hhx\r\n",fPowerModulStatus.sStatus.bErrorValue2);
   //printf("bErrorValue3 = %hhx\r\n",fPowerModulStatus.sStatus.bErrorValue3);

#endif

   return(Result);
}

boolean ProcessReceived(fExecutive_RxObj_Union* pReceivedObject)
{
   boolean Result;
   uint16_t ReceivedCheckSum;
   fStatusUnion fLocalStatusVar;

   ReceivedCheckSum = Rx_CheckSumCalcul(pReceivedObject);

   // if error or not received
   if ((ReceivedCheckSum != 0)&&(ReceivedCheckSum == pReceivedObject->RxObj.checkSum))
   {
      Result = TRUE;

      // Status of power plate
      fLocalStatusVar.cStatusByte = pReceivedObject->RxObj.Status.cStatusByte;

   }
   else
   {
      Result = FALSE;
      fLocalStatusVar.cStatusByte = 0;
      fLocalStatusVar.sStatus.bNotFoundErr = TRUE;
   }

   fPowerModulStatus = fLocalStatusVar;

   return(Result);
}

uint8_t Tx_CheckSumCalcul(fExecutive_TxObj_Union* pTransmitObject)
{
   uint8_t  cCheckSum;
   uint16_t iIndex;

   cCheckSum = 0;

   for (iIndex = 0; iIndex < kMsgLength - 2; iIndex++)
   {
      cCheckSum += pTransmitObject->cTxData[iIndex];
   }

   return(cCheckSum);
};

uint8_t Rx_CheckSumCalcul(fExecutive_RxObj_Union* pReceiveObject)
{
   uint8_t  cCheckSum;
   uint16_t iIndex;

   cCheckSum = 0;

   for (iIndex = 0; iIndex < kMsgLength-1; iIndex++)
   {
      cCheckSum += pReceiveObject->cRxData[iIndex];
   }

   return(cCheckSum);
};

/* *****************************************************************************
 * void executeModulControl(void)
 * ******************************************************************************/
void executeModulControl(void)
{
 //  static struct timespec TimerLocal; // Local timer for save last state
 //  printf("dTime of Power calling %d\n\r", deltaMilis(&TimerLocal));
   setExecuteModule();

  // printf("Duration Power calling %d\n\r", deltaMilis(&TimerLocal));

}

/* *****************************************************************************
 * uint16_t LimitPWM(uint16_t Input)
 * ******************************************************************************/
uint16_t LimitPWM(uint16_t Input)
{

   /*
    * The range of PWM should be limited:
    * On Low side  5 ms   - 5/142 = 3%
    * On High side 25 ms  - 25/142 = 18%
    * */
   #define kLowLimitPWM    3ul //% Low level limitation
   #define kHighLimitPWM   18ul //% High level limitation

   uint16_t iResult;
   uint32_t lLocalWorkVar;

   iResult = Input;
   lLocalWorkVar = ((uint32_t)kMaxVarValue*kLowLimitPWM)/100;

   #if (kLowLimitPWM > 0)
   if ((uint32_t)Input < lLocalWorkVar)
   {
      iResult = (uint16_t)lLocalWorkVar;
   }
   else
   #endif
   {
      lLocalWorkVar = ((uint32_t)kMaxVarValue*(100ul-kHighLimitPWM))/100;
      if ((uint32_t)Input > lLocalWorkVar)
      {
         iResult = lLocalWorkVar;
      }
   }

   return(iResult);
}
