 package ca.mcgill.ecse211.odometer;
 
import lejos.hardware.motor.EV3LargeRegulatedMotor;
 
public class Odometer extends OdometerData implements Runnable {
 
  private static Odometer odo = null; // Returned as singleton
 
  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private int lastTachoL, lastTachoR;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private Object lock;
  private final double TRACK;
  private final double WHEEL_RAD;
 
  private double[] position;
 
 
  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms
 
  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   *
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
 
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
 
    // Reset the values of x, y and z to 0
    this.setXYT(0, 0, 0);
    lock = new Object();
    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;
 
    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;
 
  }
 
 
 
 
 
  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   *
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }
 
  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   *
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {
 
    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");
 
    }
    return odo;
  }
 
  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
  public void run() {
     
    long updateStart, updateEnd;
   
    double distL, distR, deltaD, deltaT, dX, dY;
   
    position = odo.getXYT();
   
    leftMotor.resetTachoCount();
    rightMotor.resetTachoCount();
   
    lastTachoL = leftMotor.getTachoCount();
    lastTachoR = rightMotor.getTachoCount();
   
    while (true) {
       
      updateStart = System.currentTimeMillis();
 
      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();
 
      // TODO Calculate new robot position based on tachometer counts
     
    distL = 3.14159 * WHEEL_RAD * (leftMotorTachoCount - lastTachoL )/180; // compute wheel
    distR = 3.14159 * WHEEL_RAD * (rightMotorTachoCount - lastTachoR  )/180; // displacements
   
    lastTachoL = leftMotorTachoCount; // save tacho counts for next iteration
    lastTachoR = rightMotorTachoCount;
   
    deltaD = 0.5*( distL + distR); // compute vehicle displacement
    deltaT = ( distL - distR )/TRACK; // compute change in heading
   
    synchronized (lock) {
   
        position[2] += deltaT;  
   
        if ( position[2] >= 2*Math.PI) {
            position[2] -= 2*Math.PI;
        }
    else if (position[2] < 0 ) {
            position[2] += 2*Math.PI;
    }
   
    dX = deltaD * Math.sin (position[2]); // compute X component of displacement
    dY = deltaD * Math.cos (position[2]); // compute Y component of displacement
 
    }
      // TODO Update odometer values with new calculated values
      odo.update(dX, dY, Math.toDegrees(deltaT));
 
      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }
 
 
 
 
}