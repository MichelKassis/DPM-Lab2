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
  
  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    int xAxisLinesCounter, yAxisLinesCounter;
    xAxisLinesCounter = yAxisLinesCounter = -1;

    while (true) {
      correctionStart = System.currentTimeMillis();
      myColorSample.fetchSample(sampleColor,0); 
      if(sampleColor[0] < 0.3){
    	  Sound.beep(); // beep when the robot crosses any of the lines
    	  double thetaAngle = odometer.getTheta();
    	  double widthOftheTile = 30.48;  
		    	  if( thetaAngle >= 0 && thetaAngle < 10 || thetaAngle >= 345 && thetaAngle < 359.999) {	
		    		yAxisLinesCounter++;
		    		odometer.setY(widthOftheTile*yAxisLinesCounter);
		    	  }	 
		    	  else if( thetaAngle <185 && thetaAngle >= 170){		    		 
			     odometer.setY(widthOftheTile*yAxisLinesCounter);
			      yAxisLinesCounter--;
			  }	
		    	  else if( thetaAngle < 95 && thetaAngle >= 80){
		    		 xAxisLinesCounter++;
		    		 odometer.setX(widthOftheTile*xAxisLinesCounter);	    		  
		    	  }	    	      	  
		    	  else {     		 
		     	 odometer.setX(widthOftheTile*xAxisLinesCounter);
		     	 xAxisLinesCounter--;     		  
		     	  }
      }
      // this ensure the odometry correction occurs only once every period
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