/*
 * EKG Heart Rate Monitor and Arrythmia Detector
 * 
 * Uses ADC interrupts to take in an EKG signal and
 * many digital filters and other DSP elements to
 * smooth the signal for data extraction.
 * 
 * Authors:
   Kalvin Hallesy
   Jacob Chong
*/

#include "SPI.h"
#include "ILI9341_t3.h"
#include <XPT2046_Touchscreen.h>

#define TFT_DC  9
#define TFT_CS 10
#define CS_PIN 4

#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01

XPT2046_Touchscreen ts(CS_PIN);

// For the Adafruit shield, these are the default.
// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

uint16_t samples[30];

int oldValue;
int oldDelayedAdcValue;
int adcValue;
int tempAdcValue;
int delayedAdcValue;
int squaredValue;
int pixel;
int iter;
int stabIndex;
int bufferIndex;
int bufferResult[7500];
int qrsDelay;
int beat;
int timeCount;
int adapThreshold;
int adapThresholdInterval;
int qrsCount;
int qrsDelayCount;
int qrsTotalTime;
int qrsTimeCount;
boolean started;
boolean stabilized;
boolean completed;
boolean isBeat;
boolean qStarted;
boolean rStarted;
boolean sStarted;
boolean PACWar;

#define PDB_CONFIG    (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE \
                       | PDB_SC_CONT | PDB_SC_PRESCALER(7) | PDB_SC_MULT(1))

#define INPUT_PIN     8

#define delayCount 0
#define stabilizationDelay 1250
#define qrsThreshold -50000


// 48 MHz / 128 / 10 / 1 Hz = 37500
#define PDB_PERIOD (F_BUS / 128 / 10 / 250)

// bluetooth settings
#include <Arduino.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

int32_t hrmServiceId;
int32_t hrmMeasureCharId;
int32_t hrmLocationCharId;

void setup() {
  pinMode(INPUT_PIN, INPUT);
  analogReadResolution(12);
  tft.begin();
  tft.setRotation(1);
  LoadingScreen();
  adcValue = 0;
  tempAdcValue = 0;
  delayedAdcValue = 0;
  pixel = 0;
  oldValue = 0;
  iter = 0;
  stabIndex = 0;
  bufferIndex = 0;
  beat = 0;
  timeCount = 0;
  qrsDelay = 0;
  adapThreshold = 40000;
  adapThresholdInterval = -5000;
  qrsCount = 0;
  qrsDelayCount = 0;
  qrsTotalTime = 0;
  qrsTimeCount = 0;
  PACWar = 0;

  isBeat = false;
  stabilized = false;
  completed = false;
  started = false;
  qStarted = false;
  rStarted = false;
  sStarted = false;

  adcInit();
  pdbInit();
  dmaInit();
  

  boolean success;
  randomSeed(micros());

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ) {
    error(F("Couldn't factory reset"));
  }
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
  Serial.println(F("Setting device name to 'Bluefruit HRM': "));

  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Bluefruit HRM")) ) {
    error(F("Could not set device name?"));
  }

  /* Add the Heart Rate Service definition */
  /* Service ID should be 1 */
  Serial.println(F("Adding the Heart Rate Service definition (UUID = 0x180D): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x180D"), &hrmServiceId);
  if (! success) {
    error(F("Could not add HRM service"));
  }

  /* Add the Heart Rate Measurement characteristic */
  /* Chars ID for Measurement should be 1 */
  Serial.println(F("Adding the Heart Rate Measurement characteristic (UUID = 0x2A37): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40"), &hrmMeasureCharId);
  if (! success) {
    error(F("Could not add HRM characteristic"));
  }

  /* Add the Body Sensor Location characteristic */
  /* Chars ID for Body should be 2 */
  Serial.println(F("Adding the Body Sensor Location characteristic (UUID = 0x2A38): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A38, PROPERTIES=0x02, MIN_LEN=1, VALUE=3"), &hrmLocationCharId);
  if (! success) {
    error(F("Could not add BSL characteristic"));
  }

  /* Add the Heart Rate Service to the advertising data (needed for Nordic apps to detect the service) */
  Serial.print(F("Adding Heart Rate Service UUID to the advertising payload: "));
  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );

  /* Reset the device for the new service setting changes to take effect */
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();

  Serial.println();

  startScreen();
}

void loop() {
  if (!completed) {

    boolean pressed = ts.touched();

    if (pressed) {
      started = true;
    }

    if (started) {
      if (!stabilized) {
        stabilization();
        stabilized = true;
        stabIndex++;
      } else if (stabIndex <= stabilizationDelay) {
        stabIndex++;
      } else if (stabIndex > stabilizationDelay) {
        if (bufferIndex <= delayCount) {
          noInterrupts();
          tempAdcValue = adcValue;
          interrupts();
          bufferResult[bufferIndex] = tempAdcValue;
          bufferIndex++;

        } else if (bufferIndex < 7500) {
          timeCount++;

          noInterrupts();
          tempAdcValue = adcValue;
          interrupts();
          bufferResult[bufferIndex] = tempAdcValue;

          delayedAdcValue = bufferResult[bufferIndex - delayCount];

          tempAdcValue -= 2048;
          delayedAdcValue -= 2048;

          //lowpass butterworth 4-pole IIR filter (21Hz)
          tempAdcValue = lowPassFilter_qrs(tempAdcValue);
          delayedAdcValue = lowPassFilter(delayedAdcValue);

          //highpass butterworth 3-pole IIR filter (0.5Hz)
          tempAdcValue = highPassFilter_qrs(tempAdcValue);
          delayedAdcValue = highPassFilter(delayedAdcValue);


          // bandpass butterworth 3-pole IIR filter (7Hz - 20Hz)
          tempAdcValue = qrsFilter(tempAdcValue);

          if (tempAdcValue < 0) {
            squaredValue = 0;
          } else {
            squaredValue = sq(tempAdcValue);
          }

          if (squaredValue > adapThreshold && (qrsDelay > 60 || qrsDelay == 0)) {
            adapThreshold = squaredValue * 0.75;
            tft.drawLine(pixel, 0, pixel, 30, ILI9341_YELLOW);
            qStarted = true;
            rStarted = true;
            qrsDelay = 0;
            if (!isBeat) {
              beat++;
              isBeat = true;
            }
          } else {
            isBeat = false;
            qrsDelay ++;
          }

          int countPerBeat = 60 / getHeartRate() * 250 * 0.65;

          // detecting qrs interval by finding adaptive min threshold on negative values
          if (qrsDelayCount > countPerBeat) {
            if (tempAdcValue < 0) {
              int squaredNegAdcValue = sq(tempAdcValue) * -1 ;
              if (squaredNegAdcValue < adapThresholdInterval) {
                adapThresholdInterval = squaredNegAdcValue * 0.8;

                // detects where q starts
                if (qStarted == false) {
                  qStarted = true;
                  qrsCount = 0;
                }

                // when s wave starts
                if (qStarted && rStarted) {
                  sStarted = true;
                }

              } else if (qStarted) {
                rStarted = true;

                // when s finishes, count the elapsed time between starting of q and ending of s
                if (sStarted) {
                  qrsDelayCount = 0;
                  int qrsTime = qrsCount * 4;
                  qStarted = false;
                  rStarted = false;
                  sStarted = false;

                  if (qrsTime < 400) {
                    qrsTotalTime += qrsTime;
                    qrsTimeCount++;
                  }
                }
              }
            }
          }
          if (adapThreshold > 15000)
            adapThreshold *= .99;
          else if (adapThreshold > 13000)
            adapThreshold -= 20;
          else
            adapThreshold = 13000;


          if (adapThresholdInterval < -10000)
            adapThresholdInterval *= 0.99;
          else if (adapThresholdInterval < -8000)
            adapThresholdInterval += 15;
          else
            adapThresholdInterval = -8000;

          qrsCount++;
          qrsDelayCount++;

          tempAdcValue += 2048;
          delayedAdcValue += 2048;

          tempAdcValue /= (4096 / 240);
          delayedAdcValue /= (4098 / 240);

          // amplification
          //tempAdcValue = ((tempAdcValue - 120) * 1.5) + 120;

          if (pixel == 0) {
            drawGrid();
            drawBeat();
            tft.drawPixel(pixel, 240 - delayedAdcValue, ILI9341_BLACK);
            tft.drawPixel(pixel, 170 - tempAdcValue, ILI9341_BLUE);
            int heart_rate = getHeartRate();
            ble.print( F("AT+GATTCHAR=") );
            ble.print( hrmMeasureCharId );
            ble.print( F(",00-") );
            ble.println(heart_rate, HEX);

            /* Check if command executed OK */
            if ( !ble.waitForOK() )
            {
              Serial.println(F("Failed to get response!"));
            }
          } else {
            tft.drawLine(pixel - 1, 240 - oldDelayedAdcValue, pixel, 240 - delayedAdcValue, ILI9341_BLACK);
            tft.drawLine(pixel - 1, 170 - oldValue, pixel, 170 - tempAdcValue, ILI9341_BLUE);
          }

          oldDelayedAdcValue = delayedAdcValue;
          oldValue = tempAdcValue;
          pixel++;
          pixel = pixel % 320;
          iter++;
          bufferIndex ++;

        } else {

          /* Command is sent when \n (\r) or println is called */
          /* AT+GATTCHAR=CharacteristicID,value */
          completed = true;
          finishScreen();
        }
      }
    }
  }
  delay(4);
}

static const uint8_t channel2sc1a[] = {
  5, 14, 8, 9, 13, 12, 6, 7, 15, 4,
  0, 19, 3, 21, 26, 22
};

/*
  ADC_CFG1_ADIV(2)         Divide ratio = 4 (F_BUS = 48 MHz => ADCK = 12 MHz)
  ADC_CFG1_MODE(2)         Single ended 10 bit mode
  ADC_CFG1_ADLSMP          Long sample time
*/
#define ADC_CONFIG1 (ADC_CFG1_ADIV(1) | ADC_CFG1_MODE(1) | ADC_CFG1_ADLSMP)

/*
  ADC_CFG2_MUXSEL          Select channels ADxxb
  ADC_CFG2_ADLSTS(3)       Shortest long sample time
*/
#define ADC_CONFIG2 (ADC_CFG2_MUXSEL | ADC_CFG2_ADLSTS(3))

void adcInit() {
  ADC0_CFG1 = ADC_CONFIG1;
  ADC0_CFG2 = ADC_CONFIG2;
  // Voltage ref vcc, hardware trigger, DMA
  ADC0_SC2 = ADC_SC2_REFSEL(0) | ADC_SC2_ADTRG | ADC_SC2_DMAEN;

  // Enable averaging, 4 samples
  ADC0_SC3 = ADC_SC3_AVGE | ADC_SC3_AVGS(0);

  adcCalibrate();
  Serial.println("calibrated");

  // Enable ADC interrupt, configure pin
  ADC0_SC1A = ADC_SC1_AIEN | channel2sc1a[9];
  NVIC_ENABLE_IRQ(IRQ_ADC0);
}

void adcCalibrate() {
  uint16_t sum;

  // Begin calibration
  ADC0_SC3 = ADC_SC3_CAL;
  // Wait for calibration
  while (ADC0_SC3 & ADC_SC3_CAL);

  // Plus side gain
  sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
  sum = (sum / 2) | 0x8000;
  ADC0_PG = sum;

  // Minus side gain (not used in single-ended mode)
  sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
  sum = (sum / 2) | 0x8000;
  ADC0_MG = sum;
}

void pdbInit() {

  // Enable PDB clock
  SIM_SCGC6 |= SIM_SCGC6_PDB;
  // Timer period
  PDB0_MOD = PDB_PERIOD;
  // Interrupt delay
  PDB0_IDLY = 0;
  // Enable pre-trigger
  PDB0_CH0C1 = PDB_CH0C1_TOS | PDB_CH0C1_EN;
  // PDB0_CH0DLY0 = 0;
  PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
  // Software trigger (reset and restart counter)
  PDB0_SC |= PDB_SC_SWTRIG;
  // Enable interrupt request
  NVIC_ENABLE_IRQ(IRQ_PDB);
}

void dmaInit() {
  // Enable DMA, DMAMUX clocks
  SIM_SCGC7 |= SIM_SCGC7_DMA;
  SIM_SCGC6 |= SIM_SCGC6_DMAMUX;

  // Use default configuration
  DMA_CR = 0;

  // Source address
  DMA_TCD1_SADDR = &ADC0_RA;
  // Don't change source address
  DMA_TCD1_SOFF = 0;
  DMA_TCD1_SLAST = 0;
  // Destination address
  DMA_TCD1_DADDR = samples;
  // Destination offset (2 byte)
  DMA_TCD1_DOFF = 2;
  // Restore destination address after major loop
  DMA_TCD1_DLASTSGA = -sizeof(samples);
  // Source and destination size 16 bit
  DMA_TCD1_ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
  // Number of bytes to transfer (in each service request)
  DMA_TCD1_NBYTES_MLNO = 2;
  // Set loop counts
  DMA_TCD1_CITER_ELINKNO = sizeof(samples) / 2;
  DMA_TCD1_BITER_ELINKNO = sizeof(samples) / 2;
  // Enable interrupt (end-of-major loop)
  DMA_TCD1_CSR = DMA_TCD_CSR_INTMAJOR;

  // Set ADC as source (CH 1), enable DMA MUX
  DMAMUX0_CHCFG1 = DMAMUX_DISABLE;
  DMAMUX0_CHCFG1 = DMAMUX_SOURCE_ADC0 | DMAMUX_ENABLE;

  // Enable request input signal for channel 1
  DMA_SERQ = 1;

  // Enable interrupt request
  NVIC_ENABLE_IRQ(IRQ_DMA_CH1);
}

void pdb_isr() {
  if (!completed) {
    adcValue = samples[iter % 30];
  }
  // Clear interrupt flag
  PDB0_SC &= ~PDB_SC_PDBIF;
}

void adc0_isr() {
}

void dma_ch1_isr() {
  // Clear interrupt request for channel 1
  DMA_CINT = 1;
}

void drawGrid() {
  tft.fillScreen(ILI9341_WHITE);
  for (int i = 1; i < 32; i++) {
    if (i % 5 == 0) {
      tft.drawLine(i * 10 + 1, 0, i * 10 + 1, 240, ILI9341_RED);
      tft.drawLine(i * 10 - 1, 0, i * 10 - 1, 240, ILI9341_RED);
    }
    tft.drawLine(i * 10, 0, i * 10, 240, ILI9341_RED);
  }
  for (int i = 1; i < 24; i++) {
    if (i % 5 == 0) {
      tft.drawLine(0, i * 10 + 1, 320, i * 10 + 1, ILI9341_RED);
      tft.drawLine(0, i * 10 - 1, 320, i * 10 - 1, ILI9341_RED);
    }
    tft.drawLine(0, i * 10, 320, i * 10, ILI9341_RED);
  }
}

void stabilization() {
  tft.fillScreen(ILI9341_WHITE);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println();
  tft.println();
  tft.println();
  tft.println();
  tft.println(" PLACE YOUR FINGERS FIRMLY");
  tft.println();
  tft.println("       STABILIZATION");
  tft.println("           IN");
  tft.println("        PROGRESS ...");
}

void startScreen() {
  tft.fillScreen(ILI9341_WHITE);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println();
  tft.println();
  tft.println();
  tft.println();
  tft.println(" TOUCH ANYWHERE ON SCREEN");
  tft.println("        TO START");
  tft.println();
  tft.println("           AND");
  tft.println();
  tft.println("   PLACE YOUR INDEX AND");
  tft.println(" MIDDLE FINGER ON DOG TAGS");
}

void LoadingScreen() {
  tft.fillScreen(ILI9341_WHITE);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println();
  tft.println();
  tft.println();
  tft.println();
  tft.println(" Initializing...");
}

void finishScreen() {
  tft.fillScreen(ILI9341_WHITE);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(4);
  tft.println();
  tft.println("  COMPLETED");
  tft.setTextSize(2);
  tft.println();
  int heartRate = getHeartRate();

  if (heartRate >= 100) {
    tft.println(" HR:              " + (String) heartRate + " bpm");
  } else {
    tft.println(" HR:               " + (String) heartRate + " bpm");
  }
  if (bradycardia(heartRate)) {
    tft.println(" Bradycardia:     WARNING");
  } else {
    tft.println(" Bradycardia:        SAFE");
  }
  if (tachycardia(heartRate)) {
    tft.println(" Tachycardia:     WARNING");
  } else {
    tft.println(" Tachycardia:        SAFE");
  }
  tft.println();

  tft.println(" PR INT:      ");
  int avgQrsTime = qrsTotalTime / qrsTimeCount;
  if (avgQrsTime >= 100) {
    tft.print(" QRS INT:          "); tft.print(avgQrsTime); tft.println(" ms");
  } else {
    tft.print(" QRS INT:           "); tft.print(avgQrsTime); tft.println(" ms");
  }

  if (PVC(avgQrsTime)) {
    tft.println(" PVC:             WARNING");
  } else {
    tft.println(" PVC:                SAFE");
  }
  tft.println();

  if (PACWar)
    tft.println(" PAC:             WARNING");
  else
    tft.println(" PAC:                SAFE");
}

boolean bradycardia(int heartRate) {
  if (heartRate < 60) {
    return true;
  }
  return false;
}

boolean tachycardia(int heartRate) {
  if (heartRate > 100) {
    return true;
  }
  return false;
}

boolean PVC(int avgQrsTime) {
  if (avgQrsTime > 120) {
    return true;
  }
  return false;
}
void drawBeat() {
  tft.setCursor(200, 200);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  int heartRate = getHeartRate();
  tft.println("HR: " + (String) heartRate + "bpm");
}

int getHeartRate() {
  return beat * 250 * 60 / timeCount;
}

#define NZEROS4 6
#define NPOLES4 6
#define GAIN4   4.901202593e+02

static float xv4[NZEROS4 + 1], yv4[NPOLES4 + 1];

static int qrsFilter(int adcValue) {
  xv4[0] = xv4[1]; xv4[1] = xv4[2]; xv4[2] = xv4[3]; xv4[3] = xv4[4]; xv4[4] = xv4[5]; xv4[5] = xv4[6];
  xv4[6] = adcValue / GAIN4;
  yv4[0] = yv4[1]; yv4[1] = yv4[2]; yv4[2] = yv4[3]; yv4[3] = yv4[4]; yv4[4] = yv4[5]; yv4[5] = yv4[6];
  yv4[6] =   (xv4[6] - xv4[0]) + 3 * (xv4[2] - xv4[4])
             + ( -0.5742350941 * yv4[0]) + (  3.5488048361 * yv4[1])
             + ( -9.3807500154 * yv4[2]) + ( 13.5562895890 * yv4[3])
             + (-11.2911624520 * yv4[4]) + (  5.1399496514 * yv4[5]);
  return yv4[6];

}

#define NZEROS3 3
#define NPOLES3 3
#define GAIN3   1.012645743e+00

static float xv3[NZEROS3 + 1], yv3[NPOLES3 + 1];

static int highPassFilter(int adcValue) {
  xv3[0] = xv3[1]; xv3[1] = xv3[2]; xv3[2] = xv3[3];
  xv3[3] = adcValue / GAIN3;
  yv3[0] = yv3[1]; yv3[1] = yv3[2]; yv3[2] = yv3[3];
  yv3[3] =   (xv3[3] - xv3[0]) + 3 * (xv3[1] - xv3[2])
             + (  0.9751802955 * yv3[0]) + ( -2.9500496793 * yv3[1])
             + (  2.9748674241 * yv3[2]);
  return yv3[3];
}

#define NZEROS 4
#define NPOLES 4
#define GAIN   3.775352055e+02

static float xv[NZEROS + 1], yv[NPOLES + 1];

static int lowPassFilter(int adcValue) {
  xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4];
  xv[4] = adcValue / GAIN;
  yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4];
  yv[4] =   (xv[0] + xv[4]) + 4 * (xv[1] + xv[3]) + 6 * xv[2]
            + ( -0.2469881426 * yv[0]) + (  1.3267482152 * yv[1])
            + ( -2.7499775953 * yv[2]) + (  2.6278373692 * yv[3]);
  return yv[4];
}



#define NZEROS_QRS 3
#define NPOLES_QRS 3
#define GAIN_QRS   1.046281860e+00

static float xv_qrs[NZEROS_QRS + 1], yv_qrs[NPOLES_QRS + 1];

static int highPassFilter_qrs(int adcValue) {
  xv_qrs[0] = xv_qrs[1]; xv_qrs[1] = xv_qrs[2]; xv_qrs[2] = xv_qrs[3];
  xv_qrs[3] = adcValue / GAIN_QRS;
  yv_qrs[0] = yv_qrs[1]; yv_qrs[1] = yv_qrs[2]; yv_qrs[2] = yv_qrs[3];
  yv_qrs[3] =   (xv_qrs[3] - xv_qrs[0]) + 3 * (xv_qrs[1] - xv_qrs[2])
                + (  0.9134874992 * yv3[0]) + ( -2.8231058661 * yv3[1])
                + (  2.9095298327 * yv3[2]);
  return yv_qrs[3];
}

#define NZEROS_QRS2 4
#define NPOLES_QRS2 4
#define GAIN_QRS2   3.775352055e+02

static float xv_qrs2[NZEROS_QRS2 + 1], yv_qrs2[NPOLES_QRS2 + 1];

static int lowPassFilter_qrs(int adcValue) {
  xv_qrs2[0] = xv_qrs2[1]; xv_qrs2[1] = xv_qrs2[2]; xv_qrs2[2] = xv_qrs2[3]; xv_qrs2[3] = xv_qrs2[4];
  xv_qrs2[4] = adcValue / GAIN_QRS2;
  yv_qrs2[0] = yv_qrs2[1]; yv_qrs2[1] = yv_qrs2[2]; yv_qrs2[2] = yv_qrs2[3]; yv_qrs2[3] = yv_qrs2[4];
  yv_qrs2[4] =   (xv_qrs2[0] + xv_qrs2[4]) + 4 * (xv_qrs2[1] + xv_qrs2[3]) + 6 * xv_qrs2[2]
                 + ( -0.2469881426 * yv_qrs2[0]) + (  1.3267482152 * yv_qrs2[1])
                 + ( -2.7499775953 * yv_qrs2[2]) + (  2.6278373692 * yv_qrs2[3]);
  return yv_qrs2[4];
}

#define NZEROS_P 3
#define NPOLES_P 3
#define GAIN_P   1.133994877e+00

static float xv_p[NZEROS_QRS + 1], yv_p[NPOLES_QRS + 1];

static int highPassFilter_p(int adcValue) {
  xv_p[0] = xv_p[1]; xv_p[1] = xv_p[2]; xv_p[2] = xv_p[3];
  xv_p[3] = adcValue / GAIN_P;
  yv_p[0] = yv_p[1]; yv_p[1] = yv_p[2]; yv_p[2] = yv_p[3];
  yv_p[3] =   (xv_p[3] - xv_p[0]) + 3 * (xv_p[1] - xv_p[2])
                + (  0.7776385602 * yv_p[0]) + ( -2.5282312191 * yv_p[1])
                     + (  2.7488358092 * yv_p[2]);
  return yv_p[3];
}

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}
