#include <credentials.h> //put your own MYSSID and MYPWD Wifi credentials
#include <M5Unified.h>
#include <M5AtomS3.h>
#include <MadgwickAHRS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <math.h>
#include <vector>
#include <algorithm>

/*
  Wave + Slamming + Collision + Attitude + NMEA
  w/ Band-Pass filtering and optional simulation

  Key Points:
  1. All functionality is retained:
     - measureWaveFrequency
     - measureWaveHeights
     - computeJerkStats
     - measureCollisionParams
     - measureAttitudeAmplitude & RMS
     - NMEA XDR output
     - Madgwick for tilt
     - Real or simulation data
  2. The band-pass filter corners are set by:
     LOW_CUTOFF_HZ  (removes drift below this freq)
     HIGH_CUTOFF_HZ (removes noise above this freq)
  3. For real data, we filter + integrate => stable displacement.
  4. For simulation, we use the same pipeline, so we can debug easily.
  5. We add strategic serial prints after each step: raw accel, band-pass result, integrated displacement, wave freq/height, etc.

  Adjust wavePeriodSec & waveHeightMeters for your simulation,
  or set simulationMode=false to read the real IMU.
*/

// ------------------- SETTINGS -------------------
#define BUF_SIZE         4096
#define SAMPLE_FREQ      50.0      // 50Hz
#define GRAVITY_MS2      9.80665

// Wi-Fi
const char* WIFI_SSID     = "MYSSID";
const char* WIFI_PASSWORD = "PYPWD";
const int   WIFI_UDP_PORT = 10120;
const int   WIFI_RETRY_MS = 2000;  // check wifi every 2s

// Band-pass corners
#define LOW_CUTOFF_HZ   0.03   // remove drift below ~0.03 Hz (~33s period)
#define HIGH_CUTOFF_HZ  1.0    // remove high freq above 1Hz

// Simulation toggles
bool   simulationMode   = false; // set true to test wave
double wavePeriodSec    = 20.0;  // wave period
double waveHeightMeters = 1.0;   // crest->trough
unsigned long simStartMs= 0;

// --------------- GLOBALS ---------------
Madgwick madgwickFilter;
WiFiUDP  wifiUdp;

static double accelBuf[BUF_SIZE];
static double velBuf[BUF_SIZE];
static double dispBuf[BUF_SIZE];
static double pitchBuf[BUF_SIZE];
static double rollBuf[BUF_SIZE];
static double yawBuf[BUF_SIZE];

int  bufferIndex       = 0;
bool bufferFilled      = false;

double currentVelocity = 0.0;
double currentDisplacement = 0.0;

unsigned long previousMicros = 0;
unsigned long sampleInterval = 0;
unsigned long lastUpdateMs   = 0;
unsigned long lastWiFiMs     = 0;

// optional 1D Kalman for tilt filtering
double kalmanZ=0, kalmanP=1;
const double kalmanQ=0.01, kalmanR=0.1;

// wave stats struct
struct WaveStats {
  double avgHeight;
  double sigHeight;
};

// ------------- BIQUAD -------------
typedef struct {
  double a0, a1, a2, b1, b2;
  double z1, z2;
} Biquad;

static Biquad biqLowCut, biqHighCut; // two 2nd-order filters => band-pass

// forward declarations
void initBandPass();
double processBiquad(Biquad &bq, double in);
void setHighPass(Biquad &bq, double freqHz, double Q, double sampFreq);
void setLowPass(Biquad &bq,  double freqHz, double Q, double sampFreq);

// -------------- WIFI --------------
void connectWiFi(){
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start= millis();
  while(WiFi.status()!=WL_CONNECTED && (millis()-start<20000)){
    delay(250);
  }
}

void checkWiFi(){
  if(WiFi.status()!=WL_CONNECTED){
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

// -------------- KALMAN --------------
static double kalmanFilter1D(double in){
  kalmanP += kalmanQ;
  double K= kalmanP/(kalmanP + kalmanR);
  kalmanZ += K*(in - kalmanZ);
  kalmanP *= (1.0 - K);
  return kalmanZ;
}

// -------------- SIM / REAL ACCEL --------------
static double simulateWaveAccel(){
  // wave freq=1/period => omega=2*pi*(1/T)
  double T= wavePeriodSec;
  if(T<1e-9) T=1.0;
  double freqHz= 1.0/T;
  double omega= 2.0*M_PI* freqHz;
  double amplitude= 0.5 * waveHeightMeters; // in meters
  unsigned long msNow= millis() - simStartMs;
  double tSec= msNow* 0.001;
  // acceleration= - amplitude*(omega^2) sin(omega t)
  double acc= - amplitude*(omega*omega)* sin(omega*tSec);
  return acc; 
}

static double readVerticalAccelMPS2(){
  if(simulationMode){
    double a= simulateWaveAccel();
    return a;
  } else {
    // real IMU
    float ax, ay, az;
    float gx, gy, gz;
    M5.Imu.getAccelData(&ax,&ay,&az);
    M5.Imu.getGyroData(&gx,&gy,&gz);

 // debugged! not for this library, the input is in deg/s   
 //   gx*=DEG_TO_RAD; gy*=DEG_TO_RAD; gz*=DEG_TO_RAD;

    Serial.printf("[DBG] IMU raw accel: %.3f, %.3f, %.3f gyro: %.3f, %.3f, %.3f\n",ax, ay, az, gx, gy, gz);

    madgwickFilter.updateIMU(gx,gy,gz, ax,ay,az);

    Serial.printf("[DBG] Madgwick angles: pitch=%.2f deg, roll=%.2f deg, yaw=%.2f deg\n",
              madgwickFilter.getPitch(),
              madgwickFilter.getRoll(),
              madgwickFilter.getYaw());


    double rollRad=  madgwickFilter.getRoll()*DEG_TO_RAD;
    double pitchRad= madgwickFilter.getPitch()*DEG_TO_RAD;

    double gravX= sin(pitchRad);
    double gravY= -sin(rollRad)*cos(pitchRad);
    double gravZ= cos(rollRad)*cos(pitchRad);

    double linZg= az - gravZ; // in g
    double filteredZg= kalmanFilter1D(linZg);
    return filteredZg* GRAVITY_MS2;
  }
}

// -------------- INTEGRATION --------------
void integrateData(double accelBP, double dt){
  currentVelocity += accelBP * dt;
  if(fabs(currentVelocity)>50) currentVelocity=0;
  currentDisplacement += currentVelocity* dt;
  if(fabs(currentDisplacement)>50) currentDisplacement=0;
}

// -------------- measureWaveFrequency --------------
static double measureWaveFrequency(const double* arr, int length){
  if(length<2)return 0;
  double mean=0;
  for(int i=0;i<length;i++){
    mean+= arr[i];
  }
  mean/= length;

  std::vector<int> crossing;
  bool wasNeg= (arr[0]-mean<0);
  for(int i=1;i<length;i++){
    double cv= arr[i]-mean;
    bool isNeg= (cv<0);
    if(wasNeg && !isNeg){
      crossing.push_back(i);
      if(crossing.size()>300) break;
    }
    wasNeg= isNeg;
  }
  if(crossing.size()<2)return 0.0;
  double total=0; int c=0;
  for(size_t k=1;k<crossing.size();k++){
    int d= crossing[k]-crossing[k-1];
    if(d>0){ total+=d; c++; }
  }
  if(!c)return 0.0;
  double avgSamp= total/c;
  double periodSec= avgSamp/ SAMPLE_FREQ;
  if(periodSec<1e-9) return 0.0;
  return 1.0/ periodSec;
}



static WaveStats measureWaveHeights(const double* disp, int length){
  WaveStats w= {0.0, 0.0};
  if(length<2)return w;
  double mean=0;
  for(int i=0;i<length;i++){
    mean+= disp[i];
  }
  mean/= length;

  std::vector<int> zc;
  bool wasNeg=(disp[0]-mean<0);
  for(int i=1;i<length;i++){
    double cv= disp[i]-mean;
    bool neg=(cv<0);
    if(wasNeg && !neg){
      zc.push_back(i);
      if(zc.size()>300) break;
    }
    wasNeg= neg;
  }
  if(zc.size()<2)return w;

  std::vector<double> waveHeights;
  waveHeights.reserve(zc.size());
  for(size_t wv=0; wv<zc.size()-1; wv++){
    int s= zc[wv], e= zc[wv+1];
    if(e<=s) continue;
    double localMin= disp[s]-mean;
    double localMax= localMin;
    for(int j=s;j<e;j++){
      double val= disp[j]- mean;
      if(val<localMin)localMin= val;
      if(val>localMax)localMax= val;
    }
    waveHeights.push_back(localMax-localMin);
  }
  if(waveHeights.empty())return w;

  double sumH=0;
  for(double hh: waveHeights) sumH+= hh;
  double avgH= sumH / waveHeights.size();

  std::sort(waveHeights.begin(), waveHeights.end(), std::greater<double>());
  int topCount= (int)ceil(waveHeights.size()/3.0);
  if(topCount<1) topCount=1;
  double sumTop=0;
  for(int i=0; i<topCount; i++){
    sumTop+= waveHeights[i];
  }
  double sigH= sumTop/topCount;
  w.avgHeight= avgH;
  w.sigHeight= sigH;
  return w;
}

// -------------- Slamming / Jerk --------------
static void computeJerkStats(const double* arr,int length,double &jerkInst,double &jerkInteg){
  jerkInst=0; jerkInteg=0;
  if(length<2) return;
  int i2= (bufferIndex -1 + length)%length;
  int i1= (i2 -1 + length)%length;
  double la= arr[i2], sc= arr[i1];
  jerkInst= (la- sc)* SAMPLE_FREQ;

  double sumAbs=0.0;
  for(int i=1; i<length; i++){
    double jj= (arr[i]- arr[i-1])* SAMPLE_FREQ;
    sumAbs += fabs(jj);
  }
  jerkInteg= sumAbs*(1.0/SAMPLE_FREQ);
}

// -------------- beaufort --------------
static int beaufortScale(double waveH){
  if(waveH<0.1) return 0; 
  else if(waveH<0.5) return 1;
  else if(waveH<1.25)return 2;
  else if(waveH<2.5) return 3;
  else if(waveH<4)   return 4;
  else if(waveH<6)   return 5;
  else if(waveH<9)   return 6;
  else if(waveH<12)  return 7;
  else if(waveH<16)  return 8;
  else if(waveH<20)  return 9;
  else if(waveH<25)  return 10;
  else if(waveH<30)  return 11;
  else return 12;
}

// -------------- ATTITUDE --------------
static void measureAttitudeAmplitude(const double* pB,const double* rB,const double* yB,int length,
                                     double &pAmp,double &rAmp,double &yAmp){
  if(length<1){ pAmp=rAmp=yAmp=0; return;}
  double pMin=pB[0], pMax=pB[0];
  double rMin=rB[0], rMax=rB[0];
  double yMin=yB[0], yMax=yB[0];
  for(int i=1;i<length;i++){
    if(pB[i]<pMin)pMin=pB[i];
    if(pB[i]>pMax)pMax=pB[i];
    if(rB[i]<rMin)rMin=rB[i];
    if(rB[i]>rMax)rMax=rB[i];
    if(yB[i]<yMin)yMin=yB[i];
    if(yB[i]>yMax)yMax=yB[i];
  }
  pAmp= pMax- pMin;
  rAmp= rMax- rMin;
  yAmp= yMax- yMin;
}

static void measureAttitudeRMS(const double* pB,const double* rB,const double* yB,int length,
                               double &pR,double &rR,double &yR){
  if(length<1){pR=rR=yR=0;return;}
  double sp=0, sr=0, sy=0;
  for(int i=0;i<length;i++){
    sp+= pB[i]; sr+= rB[i]; sy+= yB[i];
  }
  double mp= sp/length, mr= sr/length, my= sy/length;
  double sqP=0, sqR=0, sqY=0;
  for(int i=0;i<length;i++){
    double dp=pB[i]- mp, dr=rB[i]- mr, dy=yB[i]- my;
    sqP+= dp*dp; sqR+= dr*dr; sqY+= dy*dy;
  }
  pR= sqrt(sqP/length);
  rR= sqrt(sqR/length);
  yR= sqrt(sqY/length);
}

// -------------- Collision-like --------------
static void measureCollisionParams(const double* arr,int length,double &peakAccel,double &peakJerk){
  peakAccel=0; peakJerk=0;
  if(length<2)return;
  int checkSamp= (int)round(SAMPLE_FREQ);
  if(checkSamp>length) checkSamp= length;
  int eIdx= (bufferIndex -1 + length)%length;
  int sIdx= eIdx- checkSamp+1;
  while(sIdx<0) sIdx+= length;
  int prevI= sIdx;
  for(int i=1;i<checkSamp;i++){
    int ix= (sIdx+i)%length;
    double aVal= fabs(arr[ix]);
    if(aVal> peakAccel) peakAccel=aVal;
    double jerkVal= (arr[ix]- arr[prevI])* SAMPLE_FREQ;
    double aj= fabs(jerkVal);
    if(aj>peakJerk) peakJerk= aj;
    prevI= ix;
  }
}

// -------------- Display --------------
static void displayAllParams(
  double freqHz, double periodSec,
  double avgH,   double sigH,
  double jerkInst,double jerkInteg,
  int beau,
  double pAmp, double rAmp, double yAmp,
  double pRms, double rRms, double yRms,
  double pNow, double rNow, double yNow,
  double peakAccel, double peakJerk
)
{
  M5.Lcd.setCursor(0,0);
  M5.Lcd.setTextSize(1);
  M5.Lcd.fillScreen(TFT_BLACK);

  M5.Lcd.setTextColor(TFT_CYAN,TFT_BLACK);
  M5.Lcd.printf("Filter: %.3f-%.1f Hz\n", LOW_CUTOFF_HZ, HIGH_CUTOFF_HZ);

  M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);
  M5.Lcd.printf("Wav F=%.2f T=%.1f s\n", freqHz, periodSec);

  M5.Lcd.setTextColor(TFT_GREEN,TFT_BLACK);
  M5.Lcd.printf("Wav H=%.2f 1/3=%.2f\n", avgH, sigH);
  M5.Lcd.setTextColor(TFT_RED,TFT_BLACK);
  M5.Lcd.printf("Z-Jerk =%.1f / %.1f \n", jerkInst, jerkInteg);

  M5.Lcd.setTextColor(TFT_YELLOW,TFT_BLACK);
  M5.Lcd.printf("Beaufort= %d scale\n", beau);

  M5.Lcd.setTextColor(TFT_MAGENTA,TFT_BLACK);
  M5.Lcd.printf("PitchA=%.1f RMS=%.1f \n", pAmp,pRms);
  M5.Lcd.printf("RollA =%.1f RMS=%.1f \n", rAmp,rRms);
  M5.Lcd.printf("YawA  =%.1f RMS=%.1f \n", yAmp,yRms);


  M5.Lcd.setTextColor(TFT_WHITE,TFT_BLACK);
  M5.Lcd.printf("Pitch=%.1f deg\n", pNow);
  M5.Lcd.printf("Roll =%.1f deg\n", rNow);
  M5.Lcd.printf("Yaw  =%.1f deg\n", yNow);

  M5.Lcd.setTextColor(TFT_ORANGE,TFT_BLACK);
  M5.Lcd.printf("Coll. Acc.=%.1f m/s2\n", peakAccel);
  M5.Lcd.printf("Coll. Jerk=%.1f m/s3\n", peakJerk);
}

// -------------- NMEA XDR --------------
static void sendNmeaXDR(const char* label,double val,const char* unit){
  char s[100];
  snprintf(s,sizeof(s),"XDR,A,%.2f,%s,%s", val, unit, label);
  uint8_t c=0;
  for(int i=0; s[i]; i++){
    c^=(uint8_t)s[i];
  }
  char buf[120];
  snprintf(buf,sizeof(buf),"$%s*%02X\r\n", s,c);
  wifiUdp.beginPacket(IPAddress(255,255,255,255), WIFI_UDP_PORT);
  wifiUdp.write((uint8_t*)buf, strlen(buf));
  wifiUdp.endPacket();
}

// -------------- SETUP --------------
void setup(){
  M5.begin();
  M5.Imu.init();
  Serial.begin(115200);


  simStartMs= millis();

  madgwickFilter.begin(SAMPLE_FREQ);

  connectWiFi();
  wifiUdp.begin(WIFI_UDP_PORT);

  sampleInterval= (unsigned long)(1e6 / SAMPLE_FREQ);
  previousMicros= micros();

  // init band-pass filter
  initBandPass();

  // LCD
  M5.Lcd.setRotation(0);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(0,0);
  M5.Lcd.println("Wave+Collision + BandPass");
}

// -------------- LOOP --------------
void loop(){
  M5.update();
  if(millis()- lastWiFiMs >= WIFI_RETRY_MS){
    lastWiFiMs= millis();
    checkWiFi();
  }

  unsigned long nowUs= micros();
  if((nowUs - previousMicros) >= sampleInterval){
    previousMicros += sampleInterval;
    double rawAccel= readVerticalAccelMPS2();
    double dt= sampleInterval *1e-6;

    // band-pass => apply lowCut, then highCut
    double afterLowCut = processBiquad(biqLowCut, rawAccel);
    double bandAccel   = processBiquad(biqHighCut, afterLowCut);

    // Debug prints
    Serial.printf("[DBG] i=%d raw=%.4f band=%.4f\n", bufferIndex, rawAccel, bandAccel);

    // integrate
    integrateData(bandAccel, dt);

    Serial.printf("       disp=%.4f vel=%.4f\n", currentDisplacement, currentVelocity);

    // store ring buffers
    accelBuf[bufferIndex]= bandAccel;
    velBuf[bufferIndex]  = currentVelocity;
    dispBuf[bufferIndex] = currentDisplacement;

    // orientation from Madgwick
    double pDeg= madgwickFilter.getPitch();
    double rDeg= madgwickFilter.getRoll();
    double yDeg= madgwickFilter.getYaw();
    pitchBuf[bufferIndex]= pDeg;
    rollBuf[bufferIndex] = rDeg;
    yawBuf[bufferIndex]  = yDeg;

    bufferIndex++;
    if(bufferIndex>=BUF_SIZE){
      bufferIndex=0;
      bufferFilled= true;
    }
  }

  // once per second => wave analysis
  if(millis()- lastUpdateMs>= 1000){
    lastUpdateMs= millis();
    if(bufferFilled){
      double freq= measureWaveFrequency(accelBuf, BUF_SIZE);
      double period= (freq>1e-9)? (1.0/freq): 0.0;

      WaveStats ws= measureWaveHeights(dispBuf, BUF_SIZE);

      double jInst=0, jInteg=0;
      computeJerkStats(accelBuf, BUF_SIZE, jInst,jInteg);

      int beau= beaufortScale(ws.sigHeight);

      double pAmp=0, rAmp=0, yAmp=0;
      measureAttitudeAmplitude(pitchBuf,rollBuf,yawBuf,BUF_SIZE, pAmp,rAmp,yAmp);

      double pR=0, rR=0, yR=0;
      measureAttitudeRMS(pitchBuf,rollBuf,yawBuf,BUF_SIZE, pR,rR,yR);

      int lastI=(bufferIndex -1 + BUF_SIZE)%BUF_SIZE;
      double pNow= pitchBuf[lastI];
      double rNow= rollBuf[lastI];
      double yNow= yawBuf[lastI];

      double pkA=0, pkJ=0;
      measureCollisionParams(accelBuf, BUF_SIZE, pkA,pkJ);

      // Display on LCD
      displayAllParams(freq, period,
                       ws.avgHeight, ws.sigHeight,
                       jInst, jInteg,
                       beau,
                       pAmp,rAmp,yAmp,
                       pR,rR,yR,
                       pNow,rNow,yNow,
                       pkA,pkJ);

      // NMEA
      sendNmeaXDR("Frequency", freq, "HZ");
      sendNmeaXDR("Period",    period,"S");
      sendNmeaXDR("AvgWaveH",  ws.avgHeight,"M");
      sendNmeaXDR("SigWaveH",  ws.sigHeight,"M");
      sendNmeaXDR("InstJerk",  jInst,"M/S3");
      sendNmeaXDR("IntJerk",   jInteg,"M/S2");
      sendNmeaXDR("Beaufort",  (double)beau,"");

      sendNmeaXDR("PitchAmp",  pAmp,"D");
      sendNmeaXDR("RollAmp",   rAmp,"D");
      sendNmeaXDR("YawAmp",    yAmp,"D");
      sendNmeaXDR("PitchRMS",  pR,"D");
      sendNmeaXDR("RollRMS",   rR,"D");
      sendNmeaXDR("YawRMS",    yR,"D");
      sendNmeaXDR("PitchNow",  pNow,"D");
      sendNmeaXDR("RollNow",   rNow,"D");
      sendNmeaXDR("YawNow",    yNow,"D");
      sendNmeaXDR("PeakAccel", pkA,"M/S2");
      sendNmeaXDR("PeakJerk",  pkJ,"M/S3");

      // debug wave freq & height
      Serial.printf("[1Hz Debug] freq=%.3fHz period=%.2fs AH=%.2f SH=%.2f\n",
                    freq, period, ws.avgHeight, ws.sigHeight);
    } else {
      M5.Lcd.fillScreen(TFT_BLACK);
      M5.Lcd.setCursor(0,0);
      M5.Lcd.setTextColor(TFT_YELLOW,TFT_BLACK);
      M5.Lcd.println("Filling buffer...");
    }
  }
}

// ---------------- BIQUAD CODE ----------------
void initBandPass(){
  // let Q=0.707
  double Q=0.707;
  setHighPass(biqLowCut,  LOW_CUTOFF_HZ,  Q, SAMPLE_FREQ);
  setLowPass(biqHighCut, HIGH_CUTOFF_HZ, Q, SAMPLE_FREQ);

  biqLowCut.z1=0; biqLowCut.z2=0;
  biqHighCut.z1=0;biqHighCut.z2=0;

  Serial.printf("[INFO] BandPass: Low=%.3fHz High=%.3fHz\n", LOW_CUTOFF_HZ, HIGH_CUTOFF_HZ);
}

double processBiquad(Biquad &bq, double in){
  // Direct Form 2
  double out= in*bq.a0 + bq.z1;
  bq.z1= in*bq.a1 + bq.z2 - bq.b1*out;
  bq.z2= in*bq.a2           - bq.b2*out;
  return out;
}

void setHighPass(Biquad &bq, double freqHz, double Q, double sampFreq){
  bq.z1=0; bq.z2=0;
  double omega= 2.0*M_PI* freqHz/sampFreq;
  double sin_ = sin(omega);
  double cos_ = cos(omega);
  double alpha= sin_/(2.0*Q);

  double b0= (1.0+ cos_)*0.5;
  double b1= -(1.0+ cos_);
  double b2= b0;
  double a0= 1.0+ alpha;
  double a1= -2.0* cos_;
  double a2= 1.0- alpha;

  double a0Inv= 1.0/a0;
  bq.a0= b0*a0Inv;
  bq.a1= b1*a0Inv;
  bq.a2= b2*a0Inv;
  bq.b1= a1*a0Inv;
  bq.b2= a2*a0Inv;
}

void setLowPass(Biquad &bq, double freqHz, double Q, double sampFreq){
  bq.z1=0; bq.z2=0;
  double omega= 2.0*M_PI* freqHz/sampFreq;
  double sin_ = sin(omega);
  double cos_ = cos(omega);
  double alpha= sin_/(2.0*Q);

  double b0= (1.0- cos_)*0.5;
  double b1= 1.0- cos_;
  double b2= b0;
  double a0= 1.0+ alpha;
  double a1= -2.0* cos_;
  double a2= 1.0- alpha;

  double a0Inv= 1.0/a0;
  bq.a0= b0*a0Inv;
  bq.a1= b1*a0Inv;
  bq.a2= b2*a0Inv;
  bq.b1= a1*a0Inv;
  bq.b2= a2*a0Inv;
}
