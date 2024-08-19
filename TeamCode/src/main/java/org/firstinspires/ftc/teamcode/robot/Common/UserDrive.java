package org.firstinspires.ftc.teamcode.robot.Common;

import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.DrivePowers;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;

public class UserDrive implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   public DrivePowers driveP;
   double driveSpeed, driveAngle, rotate;
   boolean useFieldCentricDrive = false;
   boolean useHeadingHold = true;
   boolean useHoldPosition = true;
   public double storedHeading = 0;
   public double deltaHeading = 0;
   double maxSpeed = 1;
   long idleDelay = System.currentTimeMillis();
   long headingDelay = System.currentTimeMillis();

   /* Constructor */
   public UserDrive(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
      driveP = new DrivePowers();
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
   }

   public void stop() {
   }

   // Determine motor speeds when under driver control
   public void userDrivePower () {
//      if (idleDelay > System.currentTimeMillis()) {  // todo:bring this back
//         setTargetToCurrentPosition();
//      }

//      driveSpeed = Math.pow(driveSpeed, 1);
      driveP.v0 = driveSpeed * (Math.cos(Math.toRadians(driveAngle)) - Math.sin(Math.toRadians(driveAngle))) + rotate;
      driveP.v2 = driveSpeed * (Math.cos(Math.toRadians(driveAngle)) + Math.sin(Math.toRadians(driveAngle))) + rotate;
      driveP.v1 = driveSpeed * (Math.cos(Math.toRadians(driveAngle)) + Math.sin(Math.toRadians(driveAngle))) - rotate;
      driveP.v3 = driveSpeed * (Math.cos(Math.toRadians(driveAngle)) - Math.sin(Math.toRadians(driveAngle))) - rotate;

      // scale so average motor speed is not more than maxSpeed, but only if maxspeed <> 1
      if (maxSpeed != 1) {
         driveP.scaleAverage(maxSpeed);
      }

      // scale to no higher than 1
      driveP.scaleMax(1);
   }

   public void setUserDriveSettings(double driveSpeed, double driveAngle, double rotate) {
//      if (!(driveSpeed == 0 && rotate == 0)) {   //todo: get rid of some of this old stuff
//         idleDelay = System.currentTimeMillis() + 500;  //was 250 to match rotate?
////         if (useAutoDistanceActivation) parts.sensors.readDistSensors(false);
//      }

      this.driveSpeed = driveSpeed;
      this.driveAngle = driveAngle;
      this.rotate = rotate;

      // Skip the fancy driving if no position is available
      if (parts.autoDrive.robotPosition == null) return;

      handleDriveIdle();  // todo: left off here
      handleRotateIdle();

      // Modify for field centric Drive
      if (useFieldCentricDrive) {
         this.driveAngle = driveAngle - storedHeading + deltaHeading;  //todo:verify this
      }
//      // Modify for Snap to Angle
//      if (useSnapToAngle) {
//         snapToAngle();
//      }
      // Modify for Hold Angle
      if (useHeadingHold) {  // || useSnapToAngle) {
         // Correct the heading if not currently being controlled
         // this should probably be incorporated into autodrive
         if (headingDelay <= System.currentTimeMillis()) {  // shouldn't need to check if == 0
            this.rotate = parts.autoDrive.getHeadingError(storedHeading) / -15 * (driveSpeed + 0.2);   // base this on speed?
         }
      }
   }

   public void handleDriveIdle() {
      if (parts.autoDrive.robotPosition == null) return;
      if (!(driveSpeed == 0 && rotate == 0)) {
         idleDelay = System.currentTimeMillis() + 500;  //was 250 to match rotate?
//         setTargetToCurrentPosition(); ?????
         //deactivate autodrive???  But may still want autoheading???
      }
      if (idleDelay < System.currentTimeMillis() && useHoldPosition) {
//         setTargetToCurrentPosition();
         //activate autodrive???
      }
   }

   public void handleRotateIdle() { //(double rotate) {
      if (parts.autoDrive.robotPosition == null) return;
      // overall plan here is to deal with IMU latency
      if (rotate != 0) {
         storedHeading = parts.autoDrive.robotPosition.R;
         headingDelay = System.currentTimeMillis() + 250;  // going to ignore the possibility of overflow
      }
      else if (headingDelay > System.currentTimeMillis()) {
         // keep re-reading until delay has passed
         storedHeading = parts.autoDrive.robotPosition.R;
      }
   }

   public void setMaxSpeed(double maxSpeed) {
      this.maxSpeed = maxSpeed;
   }

}