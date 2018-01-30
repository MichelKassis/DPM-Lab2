/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;


public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;

  // initializing color/light sensor
 static Port lightSensor = LocalEV3.get().getPort("S1");
 static SensorModes ls = new EV3ColorSensor(lightSensor);
 static SampleProvider myColorSample = ls.getMode("Red");
 static float[] sampleColor = new float[ls.sampleSize()];
  
 //counter for black lines along X and Y axis

 
  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();

  }

  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    int xCounter, yCounter;
    xCounter = yCounter = -1;

    while (true) {
      correctionStart = System.currentTimeMillis();
      myColorSample.fetchSample(sampleColor,0); 
      if(sampleColor[0] < 0.3){
    	  Sound.beep(); // beep when the robot crosses any of the lines
    	  double theta = odometer.getTheta();
    	  double widthOftheTile = 30.48;  
		    	  if(theta >= 345 && theta < 359.999 || theta >= 0 && theta < 10) {	
		    		yCounter++;
		    		odometer.setY(yCounter*widthOftheTile);
		    	  }	 
		    	  else if(theta >= 170 && theta <185){
			    		 
			     odometer.setY(yCounter*widthOftheTile);
			      yCounter--;
			  }	
		    	  else if(theta >= 80 && theta < 95){
		    		 xCounter++;
		    		 odometer.setX(xCounter*widthOftheTile);
		    		  
		    	  }	    	      	  
		    	  else {     		 
		     	 odometer.setX(xCounter*widthOftheTile);
		     	 xCounter--;
		     		  
		     	  }
      }
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}