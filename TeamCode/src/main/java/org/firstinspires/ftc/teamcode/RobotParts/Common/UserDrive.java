package org.firstinspires.ftc.teamcode.RobotParts.Common;

import org.firstinspires.ftc.teamcode.Tools.DataTypes.DriveData;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.DrivePowers;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr.Category;

public class UserDrive implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   public DrivePowers drivePowers;
   public double driveSpeed, driveAngle, rotate;
   public boolean useFieldCentricDrive = false;
   public boolean useHeadingHold = true;
   public boolean useHoldPosition = true;
   public boolean isDriving = false;
   boolean useHoldOK = false;
   public double storedHeading = 0;
   public double deltaHeading = 0;
   public double speedMaximum = 1;
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
      drivePowers = new DrivePowers();
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
      userDrivePower();
   }

   public void stop() {
   }

   // Determine motor speeds when under driver control
   public void userDrivePower () {

      if (parts.autoDrive.isNavigating) {
         TelemetryMgr.message(Category.DRIVETRAIN, "dt-usr : autoDrive is Navigating");
         if (parts.positionMgr.hasPosition()) storedHeading = parts.positionMgr.robotPosition.R;
         return;
      }

      handleDriveIdle();
      handleRotateIdle();

      if (useHoldPosition) {
         if (idleDelay <= System.currentTimeMillis()) {
            // no driver input, so let autoDrive take over and do nothing else
            if (useHoldOK) {
               parts.autoDrive.setTargetToCurrentPosition();
               useHoldOK = false;
            }
            TelemetryMgr.message(Category.DRIVETRAIN, "dt-usr : Not Driving");
            return;
         }
      }
      if (useHeadingHold) {
         // Correct the heading if not currently being controlled
         if (headingDelay <= System.currentTimeMillis()) {
//            this.rotate = parts.autoDrive.getHeadingError(storedHeading) / -15 * (driveSpeed + 0.2);   // base this on speed?
            // start with the same proportional in autodrive
            this.rotate = parts.autoDrive.PIDrotate.p * parts.autoDrive.getHeadingError(storedHeading);
            // and then scale it (determined experimentally, scaled like old code)
            this.rotate *= -2.5 * (driveSpeed + 0.2);
         }
      }

      drivePowers.v0 = driveSpeed * (Math.cos(Math.toRadians(driveAngle)) - Math.sin(Math.toRadians(driveAngle))) + rotate;
      drivePowers.v2 = driveSpeed * (Math.cos(Math.toRadians(driveAngle)) + Math.sin(Math.toRadians(driveAngle))) + rotate;
      drivePowers.v1 = driveSpeed * (Math.cos(Math.toRadians(driveAngle)) + Math.sin(Math.toRadians(driveAngle))) - rotate;
      drivePowers.v3 = driveSpeed * (Math.cos(Math.toRadians(driveAngle)) - Math.sin(Math.toRadians(driveAngle))) - rotate;

      // scale so average motor speed is not more than maxSpeed, but only if maxspeed <> 1
      if (speedMaximum != 1) {
         drivePowers.scaleAverage(speedMaximum);
      }
      // scale to no higher than 1
      drivePowers.scaleMax(1);
      parts.drivetrain.setDrivePowers(drivePowers);
      TelemetryMgr.message(Category.DRIVETRAIN, "dt-usr", drivePowers.toString(2));
   }

   public void setUserDriveSettings(DriveData driveData) {
      setUserDriveSettings(driveData.driveSpeed, driveData.driveAngle, driveData.rotate);
   }

   public void setUserDriveSettings(double driveSpeed, double driveAngle, double rotate) {
      // Controls uses this method to set drive parameters
      this.driveSpeed = driveSpeed;
      this.driveAngle = driveAngle;
      this.rotate = rotate;

      isDriving = !(driveSpeed == 0 && rotate == 0);
      TelemetryMgr.message(Category.USERDRIVE, "isDriving", isDriving);
      if (isDriving) {
         parts.autoDrive.cancelNavigation();
      }

      // Skip the fancy driving if no position is available
      if (parts.positionMgr.noPosition()) return;

      // Modify for field centric Drive
      if (useFieldCentricDrive) {
         this.driveAngle = driveAngle - storedHeading + deltaHeading;  //todo: verify this (rework based on IMUmgr?)
      }
   }

   public void handleDriveIdle() {  //todo: finish this method
      // The purpose of this is to hold the robot position
      // delayed 500ms after any movement input
      if (!(driveSpeed == 0 && rotate == 0)) {
         idleDelay = System.currentTimeMillis() + 500;  //was 250 to match rotate?
         useHoldOK = true;
      }
      if (parts.positionMgr.noPosition()) return;
//      if (idleDelay < System.currentTimeMillis() && useHoldPosition) {
//      }
   }

   public void handleRotateIdle() { //(double rotate) {
      // The purpose of this is to deal with IMU latency (and robot inertia)
      // by storing the heading delayed 250ms after any rotational input
      // Will use the img heading if position not available //todo: deal with cases where imu not being read
      if (rotate != 0) {
         storedHeading = parts.positionMgr.hasPosition() ? parts.positionMgr.robotPosition.R : parts.imuMgr.imuRobotHeading;
         headingDelay = System.currentTimeMillis() + 250;
      }
      else if (headingDelay > System.currentTimeMillis()) {
         // keep re-reading until delay has passed
         storedHeading = parts.positionMgr.hasPosition() ? parts.positionMgr.robotPosition.R : parts.imuMgr.imuRobotHeading;
      }
   }

   public void setSpeedMaximum(double speedMaximum) {
      this.speedMaximum = speedMaximum;
   }

   /* Everything below pasted blindly, need to doublecheck */  //todo: review all this

   public boolean toggleFieldCentricDrive() {
      useFieldCentricDrive = !useFieldCentricDrive;
      return useFieldCentricDrive;
   }

   public boolean toggleHeadingHold() {
      if (parts.positionMgr.noPosition()) {
         useHeadingHold = false;
         return useHeadingHold;
      }
      useHeadingHold = !useHeadingHold;
      storedHeading = parts.positionMgr.robotPosition.R;
      return useHeadingHold;
   }

   public void setDeltaHeading() {
      deltaHeading = storedHeading;
   }

   public boolean togglePositionHold() {
      useHoldPosition = !useHoldPosition;
      if (useHoldPosition) parts.autoDrive.setTargetToCurrentPosition();
      return useHoldPosition;
   }

}