// Copyright (c) 2024 makerspet.com
// Based on
//   Arduino PID Library - Version 1.0.1
//   by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
//   This Library is licensed under a GPLv3 License
#include "PID_v1_0_0.h"
#include <esp_timer.h>

static uint64_t millis() {
  return esp_timer_get_time() / 1000;
}

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID_v1::PID_v1() {
  //SetOutputLimits(0, 255);    //default output limit corresponds to 
                              //the arduino pwm limits
  SampleTime = 100;           //default Controller Sample Time is 0.1 seconds
}

void PID_v1::init(float* Input, float* Output, float* Setpoint,
        float Kp, float Ki, float Kd, int ControllerDirection)
{
	
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
	inAuto = false;
	
  SetControllerDirection(ControllerDirection);
  SetTunings(Kp, Ki, Kd);

  lastTime = millis()-SampleTime;        
}

 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
bool PID_v1::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
	  float input = *myInput;
      float error = *mySetpoint - input; // 1.32
      ITerm+= (ki * error); // 0.00132
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      float dInput = (input - lastInput);
 
      /*Compute PID Output*/
      float output = kp * error + ITerm- kd * dInput; // 0.003*1.32+0.00132
      
	  if (output > outMax) output = outMax;
    else if(output < outMin) output = outMin;
	  *myOutput = output;
	  
      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
	  return true;
   }
   else return false;
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void PID_v1::SetTunings(float Kp, float Ki, float Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
 
   dispKp = Kp; dispKi = Ki; dispKd = Kd;
   
   float SampleTimeInSec = ((float)SampleTime)/1000;  
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
 
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PID_v1::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = (float)NewSampleTime
                      / (float)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID_v1::SetOutputLimits(float Min, float Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
 
   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;
	 
	   if(ITerm > outMax) ITerm= outMax;
	   else if(ITerm < outMin) ITerm= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void PID_v1::SetMode(int Mode) {
  bool newAuto = (Mode == AUTOMATIC);
  if (newAuto == !inAuto) {
    // we just went from manual to auto
    Initialize();
  }
  inAuto = newAuto;
}
 
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void PID_v1::Initialize() {
  ITerm = *myOutput;
  lastInput = *myInput;
  if (ITerm > outMax)
    ITerm = outMax;
  else
    if (ITerm < outMin)
      ITerm = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID_v1::SetControllerDirection(int Direction) {
   if(inAuto && Direction !=controllerDirection) {
	  kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }   
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float PID_v1::GetKp(){ return  dispKp; }
float PID_v1::GetKi(){ return  dispKi;}
float PID_v1::GetKd(){ return  dispKd;}
int PID_v1::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID_v1::GetDirection(){ return controllerDirection;}
