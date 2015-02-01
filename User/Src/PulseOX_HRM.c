
#include "PulseOx_HRM.h"
#include "stdio.h"

int voltageCodeCounter = 0; //Counter for how many times DRDY is received


const  long SCALE = 33554432;
const double SOS [3][6] = {
  {1,  0,  -1,  1,  -1.9725464130456409,  0.97381705538051955},
  {1,  0,  -1,  1,  -1.9950869055123981,  0.99513037435061724},
  {1,  0,  -1,  1,  -1.9688341956177746,  0.96906741719379641}};

//Variables for SpO2 measurement
long maxIRValue = 0;
long minIRValue = 1000000000;
long maxRedValue = 0;
long minRedValue = 1000000000;

int pulseStarted = 0; //Flag to indicate if at least 1 pulse is completed

long previousValue = 0;
long currentValue = 0;


//Variables for heart rate measurement
int transitions[2] = {-1,-1};
static unsigned int decision_hist[4] = {0, 0, 0, 0};
static unsigned int hist_sum = 0;
static unsigned int hist_count = 0;
static unsigned int delta_hr = 0;

int heartRateCalc = 0;

unsigned int heartRate = 0;
unsigned int heartRate2 = 10;
unsigned int heartReport = 10;
unsigned int pulseOx = 0;

extern long collectIR(void);
extern long collectRED(void);
extern long collectIRMinusAMBIR(void);
extern long collectAMBIR(void);
extern long collectAMBRED(void);

/**
* @brief Bandpass filter between 0.5 and 3Hz
*
* @param  sample a long
*
* @return  long
*/

long filter(long sample)
{

  long b_int [3][3] = {0,0,0,0,0,0,0,0,0};
  long a_int [3][3] = {0,0,0,0,0,0,0,0,0};

  int k;
  int j;

  for(k=0; k<3; k++)
  {
    for(j=0; j<3;j++)
    {
      b_int[k][j] = (long) (SOS [k][j] * SCALE);
      a_int[k][j] = (long) (SOS [k][j+3] * SCALE);
    }
  }

  long s_int[4] = {0,0,0,0};

  for(j=0;j<4;j++)
  {
    s_int[j] = (long) (s[j] * SCALE);
  }

  
    static long dly [STAGES][2] = {{0,0}, {0,0}, {0,0}};
    long result, wn;
    long mysample = sample;
    long wa1, wa2, wa3;
    long wb1, wb2, wb3;

    int i;
    for (i = 0; i < STAGES; i++)
    {
            //2nd-order LCCDE code
            //(eqn 8)

            wa1 =  ((long long)mysample * s_int[i]) >> (TRUNC_BITS);
            wa2 = ((long long)a_int[i][1] * dly[i][0]) >> TRUNC_BITS;
            wa3 = ((long long)a_int[i][2] * dly[i][1]) >> TRUNC_BITS;
            wn = wa1 - wa2 - wa3;
          
            //(eqn 9)
            wb1 = ((long long)b_int[i][0] * wn) >> TRUNC_BITS;
            wb2 = ((long long)b_int[i][1] * dly[i][0]) >> TRUNC_BITS;
            wb3 = ((long long)b_int[i][2] * dly[i][1]) >> TRUNC_BITS;

            result = wb1 + wb2 + wb3;
           
            //Update filter buffers for stage i
            dly[i][1] = dly[i][0];
            dly[i][0] = wn;
            mysample = result; //in case we have to loop again


    }
    
 return (long)result;

}


/**
* @brief Calculate heart rate given start and end of pulse
*
* @param  start an int
*
* @param  end an int
*
* @return  unsigned int
*/

unsigned int calcHeartRate(int start, int end)
{
  int pulseLength = 0;
  
  if(start > end) end += 3000; //In case end index of pulse wrapped around 6 -second window
  pulseLength = end - start; //Calculate length of pulse based on start and end indices

  //Check if this is reasonable pulse length
  if((pulseLength >= sampleRate/4) && (pulseLength < sampleRate*3)) 
  {
    
    double tempHeartRate = (60.00 * sampleRate)/ pulseLength;
    
    tempHeartRate *= 100; //Multiply by 100 to maintain 2 decimal points when casting to integer
    
    int heartRate = (int)tempHeartRate; //Cast down to integer to send over BLE
    
    return heartRate;
  }
  
  return 0; //Return 0 for invalid pulse length

}

/**
* @brief Calculate SpO2 given max and min Red and IR values
*
* @param  maxIR a long
*
* @param  minIR a long
*
* @param  maxRed a long
*
* @param  minRed a long
*
* @return  unsigned int
*/

unsigned int calcPulseOx(long maxIR, long minIR, long maxRed, long minRed)
{
  double irDC = minIR;
  double irAC = maxIR - minIR;
  double redDC = minRed;
  double redAC = maxRed - minRed;
  
  double ratio = (redAC/redDC)/(irAC/irDC);
  
  spO2 = 110 - 25*ratio;
  
  double tempSpO2 = 100*spO2; //Multiply by 100 to maintain 2 decimal points when casting down to integer
  
  int pulseOx = (int)tempSpO2; //Cast to int to transfer over BLE
  
  return pulseOx;
  
}


#define MX (1.2 / 2097152)

/**
* @brief Processes the collected Red and IR values
*
* @param  None
*
* @return  None
*/
void processData(void)
{
     voltageCodeCounter++;
     
     //long irSample = collectIR();
     //long ambirSample = collectAMBIR();
     //long redSample = collectRED();
     //long ambreadSample = collectAMBRED();
     //long irminusarm = collectIRMinusAMBIR();

    //printf("\r\n0x%8x, %9d", irSample, irSample);
    //printf("\r\n%9d,%9d,%9d,%9d,%9d,%9d", irSample, ambirSample, redSample, ambreadSample, irminusarm, HAL_GetTick());
    //printf("\r\n%d", HAL_GetTick());
    //printf("\r\n%d,%d", irSample, HAL_GetTick());
    //return;
    
    //Start calculating data after skipping 2 seconds
    if(voltageCodeCounter > 1000)
    {
       int index = voltageCodeCounter - 1001;
       
       //Read Red and IR values from AFE4400
       long irSample = collectIR();
       //long ambirSample = collectAMBIR();
       long redSample = collectRED();
       //long ambreadSample = collectAMBRED();

       //Filter the IR value
       long filtIRSample = filter(irSample);
       //printf("\r\n%d,%d,%d", irSample, filtIRSample, HAL_GetTick());
       //return;
#if 0       
       if(checkStreamStatus()) //Send data for graphing only in streaming mode
       {
            //Send 1 of every 5 samples for graphing
            if(voltageCodeCounter % 5 == 0) 
             {
                 if(filtered) //If filtered flag is set stream filtered data
                 {
                       //Do a signed shift to get lower 2 bytes of data
                       voltageCodeBuffer[bufferCounter++] = filtIRSample >> 8;
                       voltageCodeBuffer[bufferCounter++] = filtIRSample;
                       
                       //Pack data in chunks of 20 bytes
                       if(bufferCounter >= 20)
                       {
                         unsigned char tempBuffer[20];
                         
                         //Transfer data and clear buffer
                         int i;
                         for(i=0; i<20; i++)
                         {
                           tempBuffer[i] = voltageCodeBuffer[i];
                           voltageCodeBuffer[i] = 0;
                         }
                         
                         bufferCounter = 0; //Reset index counter
                         
                         sendBLEBuffer(tempBuffer); //Call BLE function to send the data
                       }
                 }
                 
                 //Else stream unfiltered data
                 else
                 {
                     //Do a signed shift to get lower 2 bytes of data
                     voltageCodeBuffer[bufferCounter++] = irSample >> 9;
                     voltageCodeBuffer[bufferCounter++] = irSample >> 1;
                     if(bufferCounter >= 20)
                     {
                       unsigned char tempBuffer[20];
                       
                       //Transfer data and clear buffer
                       int i;
                       for(i=0; i<20; i++)
                       {
                         tempBuffer[i] = voltageCodeBuffer[i];
                         voltageCodeBuffer[i] = 0;
                       }
                       
                       bufferCounter = 0; //Reset index counter
                       
                       sendBLEBuffer(tempBuffer); //Call BLE function to send the data
                     }
                 }
              
             }
       }
#endif

       //Find max and min IR and Red values in the current pulse
       if(pulseStarted)
       {
         if(irSample > maxIRValue) maxIRValue = irSample;
         if(irSample < minIRValue) minIRValue = irSample;
         if(redSample > maxRedValue) maxRedValue = redSample;
         if(redSample < minRedValue) minRedValue = redSample;
       }

       previousValue = currentValue;
       currentValue = filtIRSample;
       
       //If there was a transition from negative to positive in the filtered data
       if(previousValue < 0 && currentValue > 0)
       {
         if(transitions[0] == -1) transitions[0] = index; //If this is the first transition
         else if(transitions[1] == -1) //If this is the second transition
         {
           transitions[1] = index;
           pulseStarted = 1; //Set pulse started flag only after 2 confirmed transitions
         }
         else
         {
           //Keep indices of last two transitions to estimate length of pulse
           transitions[0] = transitions[1];
           transitions[1] = index;
           
           //Call signal functions to calculate heart rate and SpO2
           heartRate = calcHeartRate(transitions[0], transitions[1]);
           pulseOx = calcPulseOx(maxIRValue, minIRValue, maxRedValue, minRedValue);
            
           //Average heart rate with previous history of values
           if(hist_count < 4)
           {
              decision_hist[hist_count] = heartRate;
              hist_count++;
              hist_sum = hist_sum + heartRate;
              
              heartRate2 = heartRate;
           }
           else
           {
              if (heartRate > (hist_sum/4+300))
              {
                heartRate2 = decision_hist[3] + (delta_hr)*100;
                
                if(delta_hr <3)
                  delta_hr++;
              }
              else if(heartRate < (hist_sum/4-300)) 
              {
                heartRate2 = decision_hist[3] - (300 - (delta_hr)*100);
                
                if(delta_hr > 0)
                  delta_hr--;            
              }
              else
              {
                heartRate2 = heartRate;
                
              } 
              
              hist_sum = hist_sum - decision_hist[0] + heartRate2;              
              decision_hist[0] = decision_hist[1];
              decision_hist[1] = decision_hist[2];
              decision_hist[2] = decision_hist[3];   
              decision_hist[3] = heartRate2;
              
              heartReport = hist_sum/4;

           }
           
           //Reset maximum and minimum defaults for comparison in next cycle
           maxIRValue = 0;
           minIRValue = 1000000000;
           maxRedValue = 0;
           minRedValue = 1000000000;
         }
       }
       
       if(voltageCodeCounter >= 1500 && voltageCodeCounter%500 == 0) //Send heart rate and SpO2 values every 1 seconds
       {
          //Send values only if they are within reasonable range
          if((heartReport/100) > 40 && (heartReport/100) < 150)
          {
            printf("\r\nHeartRate: %d", heartReport);
          }
          
          if(((pulseOx/100) <= 100))
          {
            printf("\r\nPulseOx: %d", pulseOx);
          }
               
          if(voltageCodeCounter >= 4000)
          { 
            voltageCodeCounter = 1000;
          }
       }
      
    }
    
    //Build a filter history for the initial few samples
    else
    {
      filter(collectIRMinusAMBIR());
    }
}

