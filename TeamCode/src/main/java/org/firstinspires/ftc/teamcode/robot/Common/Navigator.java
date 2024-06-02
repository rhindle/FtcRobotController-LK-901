package org.firstinspires.ftc.teamcode.robot.Common;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.Functions;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.Position;

public class Navigator implements PartsInterface {

   Parts parts;
//   Telemetry telemetry;

   Position robotPosition = new Position();

   public double v0, v2, v1, v3;
   double driveSpeed, driveAngle, rotate;
   double maxSpeed = 1;
   public double storedHeading = 0;
   public double deltaHeading = 0;
   boolean useFieldCentricDrive = false; //true
   boolean useHeadingHold = true;
   boolean useHoldPosition = true;
   boolean useAutoDistanceActivation = true;
   boolean useSnapToAngle = false;
   long headingDelay = System.currentTimeMillis();
   long idleDelay = System.currentTimeMillis();
   double motorMinPower = 0.025;
   double motorMaxPower = 1;

   boolean onTargetByAccuracy = false;

   Position targetPos = new Position(0,0,0);
   public int navigate = 0;
   int accurate = 1;  // 0 is loose, 1 is tight, more later?
   int navStep = 0;
   private ElapsedTime navTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

   public PIDCoefficients PIDmovement = new PIDCoefficients(0.06,0.0012,0.006); //.12 0 .035
   public PIDCoefficients PIDrotate = new PIDCoefficients(0.026,0.01,0.00025);  // .03 0 0
   PIDCoefficients PIDmovement_calculated = new PIDCoefficients(0,0,0);
   PIDCoefficients PIDrotate_calculated = new PIDCoefficients(0,0,0);
   long PIDTimeCurrent;
   long PIDTimeLast;
   double errorDistLast;
   double errorRotLast;
   double navAngleLast;

//   Position testPosition = null;

   /* Constructor */
   public Navigator(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize() {
      v0 = 0.0;
      v1 = 0.0;
      v2 = 0.0;
      v3 = 0.0;

//      if (!parts.useODO) {
//         targetPos = new Position(0,0,0);
//      } else {
//         targetPos = parts.odometry.odoFieldStart.clone();
//      }
      if (parts.fieldStartPosition!=null) targetPos=parts.fieldStartPosition.clone();
      else targetPos= new Position();

      storedHeading = targetPos.R;
      deltaHeading = targetPos.R;
      updateRobotPosition();
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {

      updateRobotPosition();

      if (!parts.useODO) {
         userDrive();
         parts.drivetrain.setDrivePowers(new double[] {v0, v1, v2, v3});
         return;
      }

      TelemetryMgr.Message(7,"PIDmov", "PID = %.4f, %.4f, %.4f", PIDmovement.p, PIDmovement.i, PIDmovement.d);
      TelemetryMgr.Message(7, "PIDrot", "PID = %.4f, %.4f, %.4f", PIDrotate.p, PIDrotate.i, PIDrotate.d);

      // State machine-ish.  Needs embetterment.  This will determine motor powers then set them
      if (navigate == 1) {
         autoDrive();
      }
      else {
         userDrive();
         if (idleDelay < System.currentTimeMillis() && useHoldPosition) {
            autoDrive();
         }
      }

      parts.drivetrain.setDrivePowers(new double[] {v0, v1, v2, v3});
   }

   public void stop() {
   }

   // Determine motor speeds when under automatic control
   public void autoDrive () {
      double errorDist, deltaX, deltaY, errorRot, pDist, pRot, navAngle;
      deltaX = targetPos.X - robotPosition.X;  // error in x
      deltaY = targetPos.Y - robotPosition.Y;  // error in y
      TelemetryMgr.Message(7, "DeltaX", JavaUtil.formatNumber(deltaX, 2));
      TelemetryMgr.Message(7, "DeltaY", JavaUtil.formatNumber(deltaY, 2));
      errorRot = getError(targetPos.R);  // error in rotation   //20221222 added deltaheading!?
      errorDist = Math.sqrt(Math.pow(deltaX,2) + Math.pow(deltaY,2));  // distance (error) from xy destination

      //exit criteria if destination has been adequately reached
      onTargetByAccuracy = false;
      if (accurate==0 && errorDist<2) {  // no rotation component here
         onTargetByAccuracy = true;
      }
      if (errorDist<0.5 && Math.abs(errorRot)<0.2) {
         onTargetByAccuracy = true;
      }
      if (accurate==2 && errorDist<1 && Math.abs(errorRot)<1) {
         onTargetByAccuracy = true;
      }
      if (accurate==3 && errorDist<2 && Math.abs(errorRot)<5) {  // ~ like 0 but still gets proportional
         onTargetByAccuracy = true;
      }
      if (onTargetByAccuracy) {
         navStep++;
         return;
      }

      navAngle = Math.toDegrees(Math.atan2(deltaY,deltaX));  // angle to xy destination (vector when combined with distance)

      PIDTimeCurrent = System.currentTimeMillis();

      /*
      i += k_i * (current_error * (current_time - previous_time))

      if i > max_i:
      i = max_i
      elif i < -max_i:
      i = -max_i
      */

      // Still need to reset I if we're going to use it.

      if (Math.abs(navAngle-navAngleLast)>45) PIDmovement_calculated.i = 0;  // test - zero the I if we pass through an inflection

      PIDmovement_calculated.p = PIDmovement.p * errorDist;
      PIDmovement_calculated.i += PIDmovement.i * errorDist * ((PIDTimeCurrent - PIDTimeLast) / 1000.0);
      PIDmovement_calculated.i = Math.max(Math.min(PIDmovement_calculated.i,1),-1);
      PIDmovement_calculated.d = PIDmovement.d * (errorDist - errorDistLast) / ((PIDTimeCurrent - PIDTimeLast) / 1000.0);

      if (parts.useDriveEncoders) {
         pDist = PIDmovement_calculated.p;
      } else {
         pDist = PIDmovement_calculated.p +  PIDmovement_calculated.i +  PIDmovement_calculated.d;
      }
      pDist = Math.max(Math.min(pDist,motorMaxPower),motorMinPower);
      if (accurate==0) pDist = 1;  // don't bother with proportional when hitting transitional destinations

      PIDrotate_calculated.p = PIDrotate.p * errorRot;
      PIDrotate_calculated.i += PIDrotate.i * errorRot * ((PIDTimeCurrent - PIDTimeLast) / 1000.0);
      PIDrotate_calculated.i = Math.max(Math.min(PIDrotate_calculated.i,1),-1);
      PIDrotate_calculated.d = PIDrotate.d * (errorDist - errorRotLast) / ((PIDTimeCurrent - PIDTimeLast) / 1000.0);

      if (parts.useDriveEncoders) {
         pRot =  PIDrotate_calculated.p;
      } else {
         pRot =  PIDrotate_calculated.p +  PIDrotate_calculated.i +  PIDrotate_calculated.d;
      }
      pRot = Math.max(Math.min(Math.abs(pRot),motorMaxPower),motorMinPower)*Math.signum(pRot)*-1;

      PIDTimeLast = PIDTimeCurrent;
      errorDistLast = errorDist;
      errorRotLast = errorRot;
      navAngleLast = navAngle;

      TelemetryMgr.Message(7, "NavDistance", JavaUtil.formatNumber(errorDist, 2));
      TelemetryMgr.Message(7, "NavAngle", JavaUtil.formatNumber(navAngle, 2));
      TelemetryMgr.Message(7, "NavRotation", JavaUtil.formatNumber(errorRot, 2));
      TelemetryMgr.Message(7, "pDist", JavaUtil.formatNumber(pDist, 2));
      TelemetryMgr.Message(7, "pRot", JavaUtil.formatNumber(pRot, 2));

      navAngle -= robotPosition.R;  // need to account for how the robot is oriented
      double autoSpeed = pDist * 1;  // 1 here is maxspeed; could be turned into a variable
      // the following adds the mecanum X, Y, and rotation motion components for each wheel
      v0 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) - Math.sin(Math.toRadians(navAngle))) + pRot;
      v2 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) + Math.sin(Math.toRadians(navAngle))) + pRot;
      v1 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) + Math.sin(Math.toRadians(navAngle))) - pRot;
      v3 = autoSpeed * (Math.cos(Math.toRadians(navAngle)) - Math.sin(Math.toRadians(navAngle))) - pRot;

      // scale to no higher than 1
      double highValue = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v2), Math.abs(v1), Math.abs(v3), 1));
      v0 /= highValue;
      v2 /= highValue;
      v1 /= highValue;
      v3 /= highValue;
   }

   // Determine motor speeds when under driver control
   public void userDrive () {
      if (idleDelay > System.currentTimeMillis()) {
         setTargetToCurrentPosition();
      }

      driveSpeed = Math.pow(driveSpeed, 1);
      v0 = driveSpeed * (Math.cos(driveAngle / 180 * Math.PI) - Math.sin(driveAngle / 180 * Math.PI)) + rotate;
      v2 = driveSpeed * (Math.cos(driveAngle / 180 * Math.PI) + Math.sin(driveAngle / 180 * Math.PI)) + rotate;
      v1 = driveSpeed * (Math.cos(driveAngle / 180 * Math.PI) + Math.sin(driveAngle / 180 * Math.PI)) - rotate;
      v3 = driveSpeed * (Math.cos(driveAngle / 180 * Math.PI) - Math.sin(driveAngle / 180 * Math.PI)) - rotate;

      // scale so average motor speed is not more than maxSpeed
      // but only if maxspeed <> 1
      if (maxSpeed != 1) {
         double averageValue = JavaUtil.averageOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v2), Math.abs(v1), Math.abs(v3)));
         averageValue = averageValue / maxSpeed;
         if (averageValue > 1) {
            v0 /= averageValue;
            v2 /= averageValue;
            v1 /= averageValue;
            v3 /= averageValue;
         }
      }

      // scale to no higher than 1
      double highValue = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(v0), Math.abs(v2), Math.abs(v1), Math.abs(v3), 1));
      v0 /= highValue;
      v2 /= highValue;
      v1 /= highValue;
      v3 /= highValue;
   }

   // Get heading error
   public double getError(double targetAngle) {
      double robotError;
      robotError = targetAngle - robotPosition.R;
      return Functions.normalizeAngle(robotError);
   }

   public void setMaxSpeed(double maxSpeed) {
      this.maxSpeed = maxSpeed;
   }

   public void beginScriptedNav() {
      navigate=2;
      navStep=0;
      navTime.reset();
   }

   public void beginAutoDrive() {
      navigate=1;
   }

   public void setAutoDrive(boolean boo) {
      if (boo) navigate = 1; else navigate = 0;
   }

   public void setAccuracy(int A){
      accurate = A;
   }

   public boolean isOnTargetByAccuracy() {
      return onTargetByAccuracy;
   }

   public void setTargetToZeroPosition() {
      targetPos = new Position(0,0,0);
      resetPID();
   }

   public void cancelAutoNavigation() {
      navigate=0;
      storedHeading = robotPosition.R;
   }

   public void setTargetToCurrentPosition() {
      targetPos = new Position (
         Math.round(robotPosition.X),
         Math.round(robotPosition.Y),
         Math.round(robotPosition.R)
      );
      resetPID();
   }

   public void setDeltaHeading() {
      deltaHeading = storedHeading;
   }

   public void toggleFieldCentricDrive() {
      useFieldCentricDrive = !useFieldCentricDrive;
   }

   public void toggleHeadingHold() {
      useHeadingHold = !useHeadingHold;
      storedHeading = robotPosition.R;
   }

   public void setUseHeadingHold(boolean boo) {
      useHeadingHold = boo;
      storedHeading = robotPosition.R;
   }

   public void setUseHoldPosition(boolean boo) {
      useHoldPosition = boo;
   }

   public void setUseAutoDistanceActivation(boolean boo) {
      useAutoDistanceActivation = boo;
   }

   public void togglePositionHold() {
      useHoldPosition = !useHoldPosition;
      if (useHoldPosition) setTargetToCurrentPosition();
   }

   public void toggleSnapToAngle() {
      useSnapToAngle = !useSnapToAngle;
   }

   public void setTargetByDelta(double X, double Y, double R) {
      //TODO: revamp this to account for field centric operation accounting for starting orientation
      targetPos.X += X;
      targetPos.Y += Y;
      targetPos.R += R;
      storedHeading = targetPos.R;  //20230910 fix?
      resetPID();
   }

   public boolean setTargetAbsolute(double X, double Y, double R) {
      // check if within bounds
      //if (R < -180 || R > 180) return false;
      if (X < -63 || X > 63) return false;  //63
      if (Y < -63 || Y > 63) return false;  //63
      targetPos = new Position(X, Y, Functions.normalizeAngle(R));
      resetPID();
      return true;
   }

   public void setTargetRotBySnapRelative(double R) {
      targetPos.R += R;
      targetPos.R = Math.round(targetPos.R/45)*45;
      storedHeading = targetPos.R;
      resetPID();
   }

   public void setTargetByDeltaRelative(double X, double Y, double R) {
      double rot = robotPosition.R;
      targetPos.X = targetPos.X + (X * Math.cos(Math.toRadians(rot)) - Y * Math.sin(Math.toRadians(rot)));
      targetPos.Y = targetPos.Y + (X * Math.sin(Math.toRadians(rot)) + Y * Math.cos(Math.toRadians(rot)));
      targetPos.R += R;
      resetPID();
   }

   public void setUserDriveSettings(double driveSpeed, double driveAngle, double rotate) {
      if (!(driveSpeed == 0 && rotate == 0)) {
         idleDelay = System.currentTimeMillis() + 500;  //was 250 to match rotate?
         if (useAutoDistanceActivation) parts.sensors.readDistSensors(false);
      }
      this.driveSpeed = driveSpeed;
      this.driveAngle = driveAngle;
      this.rotate = rotate;
      // Modify for field centric Drive
      if (useFieldCentricDrive) {
         this.driveAngle = driveAngle - storedHeading + deltaHeading;
      }
      // Modify for Snap to Angle
      if (useSnapToAngle) {
         snapToAngle();
      }
      // Modify for Hold Angle
      if (useHeadingHold || useSnapToAngle) {
         // Correct the heading if not currently being controlled
         // this should probably be incorporated into autodrive
         if (headingDelay <= System.currentTimeMillis()) {  // shouldn't need to check if == 0
            this.rotate = getError(storedHeading) / -15 * (driveSpeed + 0.2);   // base this on speed?
         }
      }

   }

   void snapToAngle() {
      storedHeading = Math.round(storedHeading/45)*45;
      targetPos.R = storedHeading;
      resetPID();
   }

   public void handleRotate(double rotate) {
      // overall plan here is to deal with IMU latency
      if (rotate != 0) {
         storedHeading = robotPosition.R;
         headingDelay = System.currentTimeMillis() + 250;  // going to ignore the possibility of overflow
      } else if (headingDelay > System.currentTimeMillis()) {
         // keep re-reading until delay has passed
         storedHeading = robotPosition.R;
      }
   }

   public void resetPID() {
      PIDmovement_calculated.i = 0;
      PIDrotate_calculated.i = 0;
   }

   private void updateRobotPosition() {
      robotPosition = parts.robotPosition.clone();
   }

   public void setRobotPosition(Position pos) {
      robotPosition = pos.clone();
   }
}
