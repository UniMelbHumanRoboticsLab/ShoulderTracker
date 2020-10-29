/** ShoulderTracking device firmware
 * 
 * Copyright Vincent Crocher - Unimelb - 2016, 2020
 * License MIT license
 *
 * Communication protocol: Serial - 8N1 - 19200bps
 * 	*Reception commands:
 *		-CDQ: Device check query. Response: OKST
 *		-CDP: Pause device (no feedback, no log).
 *		-CDR: Run (unpause).
 *		-CDT: Test mode (log, no feedback).
 *		-CDS: Switch to STATIC mode (feedback on log are angles).
 *		-CDD: Switch to DYNAMIC mode (feedback on angular velocity).
 *   Response in the form OKxy with x=[S/D] the current/applied mode and y=[R/T/P] the current state.
 *	* Log: when not in pause, in simple logging (not binary) device will continously send a trame of the following values:
 * 			[S/D][R/T]time,angle1,angle2,velocity1,velocity2,threshold1,threshold2\n\r
 *		ex:	ST12.32,32.2,6.3,36.1,6.5 in STATIC mode, testing. time is time since initiation in seconds. Angles are in degrees, velocities in deg.s-1 and m.s-1.
 *		
 *
 */

#include <Wire.h>
#include <EEPROM.h> //Used to store magnetometer calibration values
#include <LowPower.h> //For sleep mode after inactivity

//#define V1_IMU02A //For use with MinIMU-9 v3: https://www.pololu.com/product/2468/
#define V2_ALTIMUv10 //For use with AltIMU-10 v5: https://www.pololu.com/product/2739

#include "AdaptiveThresholding.h"
#include "StaticParam.h"
#include "Filter.h"

//#define MUTE //Sound is annoying when debugging...
#define LOG //Send values over serial
//#define BINARY_LOG //Optimised faster (binary) log


unsigned long int t, Dt;

Static_param Static;
Dynamic_param Dynamic;
MODE Mode;
Filter hp_filter;

#ifdef V1_IMU02A
L3G gyro; //Gyroscopes
LSM303 compass; //Magnetometers and acclererometers
#endif

#ifdef V2_ALTIMUv10
LSM6 gyro; //Gyroscopes and accelerometers
LIS3MDL compass; //Magnetometers
#endif

int Sensitivity=90; //0-100%: the higher the less sensitive
AdaptiveThresholding AdaptThresh[2];
float MinimalThresh[2]; //Minimal values: threshold c'ant be lower than these: see InitDynamic / InitStatic for values

bool Pause;
bool Testing;

//Sleep mode
unsigned long int LastActivityInS = 0;
unsigned long int MaxInactivityBeforeSleepInS = 15*60; //Time of inactivity before device goes to sleep forever (in S)


//###################################################################################
//                              ACTION FUNCTIONS 
//###################################################################################

//PWM intensity from 0 to 1
void Vibrate(float intensity)
{
  if(intensity>1)
    intensity=1;
  
  //Motor doesn't move under 80 so min value if !=0
  if(intensity==0)
    analogWrite(BuzzPin, 0);
  else
    analogWrite(BuzzPin, 200+(intensity*(255-200)));
}


//Single beep (BLOCKING)
void Bip()
{
  #ifndef MUTE
    analogWrite(BeepPin, 20);
    delay(100);
  #endif
  analogWrite(BeepPin, 0);
}

//Double beep (BLOCKING)
void BipBip()
{
  #ifndef MUTE
    analogWrite(BeepPin, 20);
    delay(50);
    analogWrite(BeepPin, 0);
    delay(100);
    analogWrite(BeepPin, 20);
    delay(50);
  #endif
  analogWrite(BeepPin, 0);
}

void Bip(unsigned long int f)
{ 
  #ifndef MUTE
    float T_2=1/(float)f/2.*1000000; //Half period in us
    //Play the freq for 5ms
    int nb_it=5000/T_2;
    for(int i=0; i<nb_it; i++)
    {
      analogWrite(BeepPin, 255);
      delayMicroseconds(T_2);
      analogWrite(BeepPin, 0);
      delayMicroseconds(T_2);
    }
  #endif
}

void Beep()
{
	#ifndef MUTE
		analogWrite(BeepPin, 10);
	#endif
}
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------




//###################################################################################
//                                SLEEP FUNCTIONS 
//###################################################################################
void GoToSleep()
{
  //Sleep forever
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------


//###################################################################################
//                             FILTERING FUNCTIONS 
//###################################################################################
float filt(Dynamic_param *d, float x)
{
  for(int k=0; k<FILT_ORDER_LP-1; k++)
  {
    d->v[k]=d->v[k+1];
    d->vf[k]=d->vf[k+1];
  }
  d->v[FILT_ORDER_LP-1]=x;
  d->vf[FILT_ORDER_LP-1]=0;
  
  for(int k=0; k<FILT_ORDER_LP; k++)
    d->vf[FILT_ORDER_LP-1]+=FILT_COEFS_b[k]*d->v[FILT_ORDER_LP-k-1];
 
  for(int k=1; k<FILT_ORDER_LP; k++)
    d->vf[FILT_ORDER_LP-1]-=FILT_COEFS_a[k]*d->vf[FILT_ORDER_LP-k-1];

  return d->vf[FILT_ORDER_LP-1];
}
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------




//###################################################################################
//                         SENSOR PROCESSING FUNCTIONS 
//###################################################################################
//Vertical angle computation from Accelerometers
float GetAngleAcc(Static_param *s)
{
	//Store new values (cleaner but can be optimized)
	#ifdef V1_IMU02A
		s->A[0]=compass.a.x;
		s->A[1]=compass.a.y;
		s->A[2]=compass.a.z;
	#endif
	#ifdef V2_ALTIMUv10
		s->A[0]=gyro.a.x;
		s->A[1]=gyro.a.y;
		s->A[2]=gyro.a.z;
	#endif

  //Angle between X and Y projections of gravity
  return atan2(s->A[0], -s->A[2])*180/PI;
}

float GetHeading(Static_param *s)
{
	#ifdef V1_IMU02A
		return compass.heading();
	#endif
	#ifdef V2_ALTIMUv10 //
		//Compute heading based on accelerometer and magnetometer values
		//(heading not directly provided by library (no accelero))
		LIS3MDL::vector<int32_t> temp_m = {compass.m.x, compass.m.y, compass.m.z};

		// subtract offset (average of min and max) from magnetometer readings
		temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
		temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
		temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;

		// compute E and N
		LIS3MDL::vector<float> E;
		LIS3MDL::vector<float> N;
		LIS3MDL::vector<float> a = {gyro.a.x, gyro.a.y, gyro.a.z};
		LIS3MDL::vector_cross(&temp_m, &a, &E);
		LIS3MDL::vector_normalize(&E);
		LIS3MDL::vector_cross(&a, &E, &N);
		LIS3MDL::vector_normalize(&N);

		// compute heading
		LIS3MDL::vector<int16_t> from = {s->HeadingSign, 0, 0};
		float heading = atan2(LIS3MDL::vector_dot(&E, &from), LIS3MDL::vector_dot(&N, &from)) * 180 / PI;
		return heading;
	#endif
}


//Horizontal angle from magnetometer
float GetAngleMag(Static_param *s)
{
	return GetHeading(s)-s->MAngleRef;
}



//Linear velocity: integrate and filter acceleration
float GetLinVel(Dynamic_param *d)
{
	float a[3];
	#ifdef V1_IMU02A
		a[0]=compass.a.x;
		a[1]=compass.a.y;
		a[2]=compass.a.z;
	#endif
	
	#ifdef V2_ALTIMUv10
		a[0]=gyro.a.x;
		a[1]=gyro.a.y;
		a[2]=gyro.a.z;
	#endif

	for(int i=0; i<3; i++)
	{
		a[i]*=9.81*ACC_2_MS2;
	}
	d->A[0]=d->A[1];
	d->A[1]=d->A[2];
	d->A[2]=sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);

	//and compute linear velocity
	//d->v_c += (d->A[0]+(d->A[1]-d->A[0])/2.)*Dt/1000000.; //Integration w/ conversion from us to s
	d->v_c += ((d->A[0]+4*d->A[1]+d->A[0])/2.)/6 * 2*Dt/1000000.; //Integration w/ conversion from us to s
	//Serial.print(10000*V);Serial.print(" , ");
  float v=filt(d, d->v_c);
	return abs(v);//WARNING: need to be in two lines as Arduino has a crappy abs() function implementation
}

//Angular vel from gyro
float GetAngVel()
{
  return sqrt(gyro.g.x*GYRO_2_DPS*gyro.g.x*GYRO_2_DPS+gyro.g.y*GYRO_2_DPS*gyro.g.y*GYRO_2_DPS+gyro.g.z*GYRO_2_DPS*gyro.g.z*GYRO_2_DPS);
}
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------



//###################################################################################
//                                INIT FUNCTIONS 
//###################################################################################
//Select init depending on mode
void Init()
{
  compass.init();
  compass.enableDefault();
  gyro.init();
  gyro.enableDefault();
  
	switch(Mode)
	{
		case STATIC:
			InitStatic();
			break;
		case DYNAMIC:
			InitDynamic();
			break;
	}

  LastActivityInS = millis()/1000.;
}

//Initialization procedure for static mode: info sound, take reference vectors
void InitStatic()
{
	//Beep
	Bip();

	//Take reference heading angle
  gyro.read();
  compass.read();
  //ensure we have in a safe quadrant (switch sign of reference vector to do so)
  Static.HeadingSign=1;//default
  Static.MAngleRef=0;
  if(fabs(GetAngleMag(&Static)>90))
  {
    Static.HeadingSign=-1;
  }
  Static.MAngleRef=GetAngleMag(&Static);

	//Init threshold arbitrarily to 20
	Static.AngleThresh=20;

	//Reinit adaptive threshold
	AdaptThresh[0].Reset(1);
	AdaptThresh[1].Reset(1);

	//Reset minimal threshold values
	MinimalThresh[0]=10.;//Ensure above noise level
	MinimalThresh[1]=10.;//Ensure above noise level

	//Done
	BipBip();
}

//Initialization procedure for dynamic mode: info sound, take reference vectors
void InitDynamic()
{
	//Beep
	BipBip();

	Bip();

	//Reinit adaptive threshold
	AdaptThresh[0].Reset(10);
	AdaptThresh[1].Reset(10);

	//Reset minimal threshold values
	MinimalThresh[0]=0.3;//Ensure above noise level
	MinimalThresh[1]=0.04;//Ensure above noise level
	BipBip();
}

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------





//###################################################################################
//                                PRINTING FUNCTIONS 
//###################################################################################
void PrintUInt8(unsigned int val)
{
  char val8 = (abs(val)>255)?255:abs(val);
  Serial.write(val8);
}

void PrintInt8(int val)
{
  signed char val8;
  if(val>0)
    val8 = (abs(val)>127)?127:abs(val);
  else
    val8 = (abs(val)>127)?-127:val;
  Serial.write(val8);
}

void PrintUInt16(unsigned int val)
{
  word val16 = (abs(val)>65535)?65535:abs(val);
  Serial.write(lowByte(val16));
  Serial.write(highByte(val16));
}

void PrintInt16(int val)
{
  word val16;
  if(val>0)
    val16 = (abs(val)>32767)?32767:abs(val);
  else
    val16 = (abs(val)>32767)?-32767:val;
  
  Serial.write(lowByte(val16));
  Serial.write(highByte(val16));
}

void PrintUInt32(unsigned long int val)
{
  unsigned long int val32 = (abs(val)>65535*65535)?65535*65535:abs(val);
  unsigned int tmp = val32 & 0xFFFF; //lower 16 bits
  PrintInt16(tmp);
  tmp = val32 >> 16; //higher 16 bits
  PrintInt16(tmp);
}
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------



void setup()
{
	//IMU init
	Wire.begin();

	#ifdef V1_IMU02A
	compass.m_min = (LSM303::vector<int16_t>){-185,   -689,  -1835};
	compass.m_max = (LSM303::vector<int16_t>){+1221,   +680,   -610};
	#endif
	#ifdef V2_ALTIMUv10
	//Read values from the EEPROM (use CalibrateMag first)
  int eeAddress = 0;
  EEPROM.get(eeAddress, m_min);
  eeAddress += sizeof(m_min);
  EEPROM.get(eeAddress, m_max);
  if(m_min.x==0 || m_max.x==0)
  {
    Serial.println("No magnetometer calibration found!");
  }
	#endif

	//Action pins
	pinMode(BuzzPin, OUTPUT);
	pinMode(BeepPin, OUTPUT);
	//Input pin
	pinMode(ModePin, INPUT_PULLUP);

  //Init both modes at startup
  Mode=STATIC;Init();
	//And keep Dynamic mode as default
	Mode=DYNAMIC;Init();

  //Serial com
  Serial.begin(19200);
  while (!Serial) {
    ; // wait for serial port to connect.
  }

	Pause=true;
  t=micros();
}



void loop()
{
	//Update values from IMU
	compass.read();
	gyro.read();
  //Fixed update rate of 100Hz
  const unsigned long int period_us=10000;
  while((micros()-t)<period_us) { //delayMicroseconds doesn't work...
    ;
  }
  Dt=micros()-t;
  t=micros();

	//Processing and values depend on the mode
	float current_val[2]={0,0};
	char header_letters[2]={'0','0'};
	char logBeep='0', ErrorFlag='0';

  //Retrieve values from sensors
  int CoronalPlaneAngle=(int)GetAngleAcc(&Static);
  int TransversePlaneAngle=(int)GetAngleMag(&Static);
  float LinearVelocity=GetLinVel(&Dynamic);
  float AngularVelocity=GetAngVel();
  float diff[2], thresh[2];

  //Check if some movement or inactive (gyro based, above noise level)
  if( AngularVelocity>0.04 )
  {
    LastActivityInS = millis()/1000.;
  }
  
	switch(Mode)
	{
		case STATIC:
			//Get the two angles
			current_val[0]=CoronalPlaneAngle;
			current_val[1]=TransversePlaneAngle;
			header_letters[0]='S';
			break;
		case DYNAMIC:
			current_val[0]=LinearVelocity;
			current_val[1]=AngularVelocity;
			header_letters[0]='D';
			break;
	}

	//If not paused (anyway keep tacking IMU values to avoid Dt problems when back from pause)
	if(!Pause)
	{
		//Says it's running (feedback is on) or in testing
		header_letters[1]='R';
		if(Testing)
			header_letters[1]='T';

		//Add values in the stored list
		AdaptThresh[0].Store(current_val[0]);
		AdaptThresh[1].Store(current_val[1]);
	
		//Apply feedback if needed
		thresh[0]=fmax(AdaptThresh[0].GetThreshold(Sensitivity), MinimalThresh[0]);
		thresh[1]=fmax(AdaptThresh[1].GetThreshold(Sensitivity), MinimalThresh[1]);
		diff[0]=(current_val[0]-thresh[0])/thresh[0];
		diff[1]=(current_val[1]-thresh[1])/thresh[1];
		float m_diff=fmax(diff[0], diff[1]);
		//No feedback when in TESTING
		if(m_diff>0 && !Testing) 
		{
			logBeep=1;
			Vibrate(m_diff);
			Beep();
		}
		else
		{
			logBeep=0; 
			Vibrate(0);
			analogWrite(BeepPin, 0);
		}
	}
	else
	{
		//Says it's paused (no feedback, no update of the thresholds)
		header_letters[1]='P';

		//Turn off feedback
		logBeep=0;
		Vibrate(0);
		analogWrite(BeepPin, 0);
	}

	//Send values over serial
	#ifdef LOG
	if(!Pause)
	{
		Serial.print(header_letters[0]);
    Serial.print(header_letters[1]);
    #ifdef BINARY_LOG
      PrintUInt32(millis());
      PrintInt8(CoronalPlaneAngle);
      PrintInt8(TransversePlaneAngle);
      PrintUInt16((int)(LinearVelocity*1000));
      PrintUInt16((int)(AngularVelocity*1000));
      PrintUInt16((int)(thresh[0]*100));
      PrintUInt16((int)(thresh[1]*100));
      Serial.println("");
    #else
      Serial.print((float)(millis()/1000.), 3);
  		Serial.print(',');
  		Serial.print(CoronalPlaneAngle);
  		Serial.print(',');
  		Serial.print(TransversePlaneAngle);
  		Serial.print(',');
      Serial.print(LinearVelocity, 3);
      Serial.print(',');
      Serial.print(AngularVelocity, 3);
      Serial.print(',');
  		Serial.print(thresh[0], 3);
  		Serial.print(',');
  		Serial.println(thresh[1], 3);
      if(CoronalPlaneAngle>255 || TransversePlaneAngle>255 || LinearVelocity>65 || AngularVelocity>65)
        Serial.println("WARNING");
    #endif
	}

	//Check for serial message: run/pause
	//each message has the format: CDx with x=R (run) or x=P (pause)
	if(Serial.available()>2)
	{
		char msg[3];
		msg[0]=Serial.read();
		msg[1]=Serial.read();
		msg[2]=Serial.read();

		if(msg[0]=='C' && msg[1]=='D')
		{
			switch(msg[2])
			{
				//Device check query
				case 'Q':
					Serial.println("OKST");
					break;
				case 'P':
					Testing=false;
					Pause=true;
					Serial.print("OK");
					Serial.print(header_letters[0]);
					Serial.println('P');
					break;
				case 'R':
					Testing=false;
					Pause=false;
					Serial.print("OK");
					Serial.print(header_letters[0]);
					Serial.println('R');
					break;
				case 'T':
					Pause=false;
					Testing=true;
					Serial.print("OK");
					Serial.print(header_letters[0]);
					Serial.println('T');
					break;
				case 'S':
					Mode=STATIC;
					Serial.print("OK");
					Serial.print('S');
					Serial.println(header_letters[1]);
					Init();
					break;
				case 'D':
					Mode=DYNAMIC;
					Serial.print("OK");
					Serial.print('D');
					Serial.println(header_letters[1]);
					Init();
					break;
				default:
					Serial.println("E2");
			}
		}
		else
		{
			//Error
			Serial.println("E1");
		}
		//Flush serial buffer
    Serial.flush();
		while(Serial.available()>0)
			Serial.read();

    LastActivityInS = millis()/1000.;
	}
	#endif

  //Goes to sleep if inactive for too long
  if( (millis()/1000.) - LastActivityInS > MaxInactivityBeforeSleepInS )
  {
    GoToSleep();
  }
}
