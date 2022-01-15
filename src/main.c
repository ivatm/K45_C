//

#include <stdlib.h>
#include <signal.h>
#include <time.h>

#include <string.h>
#include <sys/timeb.h>

#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>

#include "ADS1256.h"
#include <i2c_lcd.h>

#include "ADCHeader.h"

#include <pthread.h>
#include <stdio.h>

#include "defines.h"

#include "modulSPI.h"

#include "user_types.h"
#include "globalvarheads.h"
#include "uart.h"

// Local defines


// Declaration for threads of the system
pthread_t ADC_Thread;              // the ADC data Flow thread
pthread_t TemperatureRegulator;    // Temperature control calculations
pthread_t Interface;               // Indication/Keyboard
pthread_t Keypad_Thread;           // Keyboard
pthread_t PowerEquipment;          // Communication with Co-Processor plate
pthread_t UARTCommThread;          // Communication with PC via Serial Protocol


// Extern procedures -------------------------------------------------------------
extern void iniTemperaturController(void);
extern boolean periferal_SPI1_Init(void);
extern void CalculRegulator(void);

extern void executeModulControl(void);
extern void updateCurrentVoltages(void);

extern void KeyboardProcess(void);
extern void AutoSettingTemperature(void);

extern void lcd_Init(void);
extern void lcd_update(void);
extern void HelloShow(void);

extern char init_keypad(void);

extern int uart_init(void);
extern int uart_read(void);
extern boolean uart_data_receive(void);
extern int uart_send(void);
extern void ADC_Calibration(void);

// Local procedures
uint16_t K45GlobalInit(void);
void* Measurements(void *parm);
void* PowerEquipment_service(void *parm);
uint16_t SetThreadPriority(pthread_t pthread_id, uint16_t priority);


/* ===============================================================================
 * The thread for Output user interfaces.
 *
 * =============================================================================== */
void* Interface_Process()
{
   static uint16_t iLocalCounter;

   lcd_Init();
   delay(500);

   LCDI2C_backlight();

   HelloShow();
// ToDo: Not delay but counter of program cycles!
   delay(3000);

   while(fSystemThreadControl.s.bInterfaceOn)
   {

      // Cycle counter
      iLocalCounter++;

      lcd_update();

      if (bUART_Active)
      {
         if (!(iLocalCounter % 4))
         {
            // every 2 seconds
            iLocalCounter = 0;
            uart_send();
         }
      }

      delay(kIndicationTimeUpdate);
   }
   /* the function must return something - NULL will do */
   return NULL;
}

/******************************************************************************
function:  The thread for Output user interfaces.
parameter:
Info:
******************************************************************************/
void* Keypad_service()
{
   while(fSystemThreadControl.s.bKeypad_ThreadOn)
   {
      KeyboardProcess();
      delay(500);
   }
/* the function must return something - NULL will do */
return NULL;
}

/* Variables for controlling the measuring and temperature regulating
   If the flag is set, the ADC-conversion is started and the flag again goes to FALSE
*/
boolean         ADC_needed_flag,         // Flag
                ADC_ready_flag;
pthread_cond_t  ADC_start_cv,            // Conditional variable
                ADC_stop_cv;
pthread_mutex_t ADC_thread_flag_mutex;   // Thread mutex

/******************************************************************************
function:  Temperature Regulator main engine
parameter:
Info:
******************************************************************************/
void* TemperatureRegulator_service()
{

   int retVal;
   pthread_attr_t attr;
   struct sched_param schedParam;

   retVal = pthread_attr_init(&attr);
   if (retVal)
   {
       fprintf(stderr, "pthread_attr_init error %d\n", retVal);
       exit(1);
   }

   retVal = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
   if (retVal)
   {
       fprintf(stderr, "pthread_attr_setinheritsched error %d\n", retVal);
       exit(1);
   }

   retVal = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
   if (retVal)
   {
       fprintf(stderr, "pthread_attr_setschedpolicy error %d\n", retVal);
       exit(1);
   }

   schedParam.sched_priority = 55;
   retVal = pthread_attr_setschedparam(&attr, &schedParam);
   if (retVal)
   {
       fprintf(stderr, "pthread_attr_setschedparam error %d\n", retVal);
       exit(1);
   }

   pthread_create(&ADC_Thread,   &attr, Measurements, NULL);

   iniTemperaturController();

   // First initialization to FALSE that means "no calculation needed"
   sSensorData.bMeasurementReady = FALSE;

   while(fSystemThreadControl.s.bTemperatureRegulatorOn)
   {
      pthread_join( ADC_Thread, NULL);

      // The new state preparation
      CalculRegulator();
      //executeModulControl();
      //updateCurrentVoltages();
      pthread_create(&ADC_Thread,   &attr, Measurements, NULL);
      pthread_join( PowerEquipment, NULL);
      pthread_create(&PowerEquipment, &attr, PowerEquipment_service, NULL);

      #ifdef SCANNING
         AutoSettingTemperature();
      #endif

      delay(mNormTime(kTemperaturControlPeriod));
   }
   /* the function must return something - NULL will do */
   return NULL;
}

/******************************************************************************
function:  The thread for measurements
parameter:
Info:
******************************************************************************/
void* Measurements(void *parm)
{
   if (bADCCalibrationProcess)
   {
      // In case of Calibration Flag is On, the according procedure
      ADC_Calibration();
      // The Flag goes Off again, since the calibration is fulfilled
      bADCCalibrationProcess = FALSE;
   }
   else
   {
      // If no calibration necessary, simple measurement
      updateCurrentVoltages();
   }

   /* the function must return something - NULL will do */
   pthread_exit(NULL);
}

/******************************************************************************
function:  The thread for power switches.
parameter:
Info:
******************************************************************************/
void* PowerEquipment_service(void *parm)
{
   executeModulControl();

   return(NULL);
}


/******************************************************************************
function:  The thread for communication with PC
parameter:
Info:
******************************************************************************/
void* UARTCommThread_service()
{

   if (bUART_Active)
   {
      while(fSystemThreadControl.s.bUARTCommThreadOn)
      {

         if (!uart_read())
         {
         }
         else
         {
            #ifdef debugmode
             //  printf(" Error on reception\r\n");
            #endif
            // Error on reception
         }

         uart_data_receive();

         //
         delay(kReceptionCheck);
      }
   }

   /* the function must return something - NULL will do */
   return(NULL);
}

/******************************************************************************
function:  uint16_t Inits(void)
parameter:
Info:
******************************************************************************/
uint16_t Inits(void)
{

   pthread_t  pt_id;
   int Err;

   if (K45GlobalInit())
   {
      printf("Something goes wrong on init...\n");
      return(FALSE);
   }

   // Threads are active (possible to create)
   fSystemThreadControl.cAllThreadStates = 0xFF;

   // Low Priority & low speed
   pt_id = pthread_create(&Interface,    NULL, Interface_Process, NULL);
   delay(500);
   if (pt_id != 0)
   {
      Err = SetThreadPriority(pt_id, 10);

      if (Err != 0)
      {
         printf("\nError on Interface priority setting: %d, %d!\n\r", (int)pt_id, Err);
      }
   }

   // Lowest Priority but highest speed
   pt_id = pthread_create(&UARTCommThread, NULL, UARTCommThread_service, NULL);
   delay(5);
   if (pt_id != 0)
   {
      Err = SetThreadPriority(pt_id, 5);

      if (Err != 0)
      {
         printf("\nError on UARTCommThread priority setting: %d, %d!\n\r", (int)pt_id, Err);
      }
   }

   pt_id = pthread_create(&Keypad_Thread,  NULL, Keypad_service,         NULL);
   delay(5);
   if (pt_id != 0)
   {
      Err = SetThreadPriority(pt_id, 1);

      if (Err != 0)
      {
         printf("\nError on Keypad_Thread priority setting: %d, %d!\n\r", (int)pt_id, Err);
      }
   }

   // Critical reaction time -> consider  - FIFO mode
   pt_id = pthread_create(&TemperatureRegulator, NULL, TemperatureRegulator_service, NULL);
   delay(5);
   if (pt_id != 0)
   {
      Err = SetThreadPriority(pt_id, 50);

      if (Err != 0)
      {
         printf("\nError on TemperatureRegulator priority setting: %d, %d!\n\r", (int)pt_id, Err);
      }
   }

   return(TRUE);
}

uint16_t SetThreadPriority(pthread_t pthread_id, uint16_t priority)
{
   uint16_t result;

   struct sched_param param;
   int policy = SCHED_FIFO; //kind of policy desired, either SCHED_FIFO or SCHED_RR, otherwise Linux uses SCHED_OTHER

   result =pthread_getschedparam(pthread_id, &policy, &param);

   printf("pthread_id: %d\n\r", (int)pthread_id);
   printf("param.sched_priority: %d\n\r", param.sched_priority);

   /* set the priority; others are unchanged */
   param.sched_priority = priority;
   /* setting the new scheduling param */
   result = pthread_setschedparam(pthread_id, policy, &param);


   return(result);
}

/******************************************************************************
function:  uint16_t K45GlobalInit(void)
parameter:
Info:
******************************************************************************/
uint16_t K45GlobalInit(void)
{
   uint16_t Result;
   boolean SPIconfigured; // Common SPI interface for Indicator and RTmodule

   //
   DEV_ModuleInit();

   SPIconfigured = periferal_SPI1_Init();

 //  bcm2835_gpio_set(IND_CS_PIN);
   bcm2835_gpio_set(MODUL_ISP_PIN);

   if (SPIconfigured)
   {
      Result = FALSE;
   }
   else
   {
      Result = TRUE;
      printf("SPI can`t configure ...\n");
   }

   if (ADC_Init())
   {
      printf("ADS1256 inited\n");
      Result = FALSE;
   }
   else
   {
      printf("ADS1256 wrong\n");
      Result = TRUE;
   }

   // Initialisation LCD via i2c connection
   lcd_Init();

   //Matrix keypad
   init_keypad();

   // UART initialisation
   if (uart_init())
   {
      printf("UART wrong\n");
      bUART_Active = FALSE;
      Result = TRUE;
   }
   else
   {
      printf("UART inited\n");
      bUART_Active = TRUE;
      Result = FALSE;
   }

   // FALSE - Allright
   // TRUE - something wrong
   return Result;
}


int main(void)
{

    if (Inits())
    {
            #ifdef debugmode
             //  printf(" Inited successful\r\n");
            #endif
    }

    // All Threads stop
    pthread_join( TemperatureRegulator, NULL);
    pthread_join( Interface,    NULL);
    pthread_join( PowerEquipment,       NULL);
    pthread_join( UARTCommThread,       NULL);

   return 0;
}
