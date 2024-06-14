package org.firstinspires.ftc.teamcode.robot.Common;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.Functions;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.Position;

public class AutoDrive implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   Position robotPosition = new Position();
   Position targetPos = new Position();
   int accurate = 1;  // 0 is loose, 1 is tight, more later?

   Status status;
   public double v0, v2, v1, v3;
   double maxSpeed = 1;
   double motorMinPower = 0.025;
   double motorMaxPower = 1;
   public PIDCoefficients PIDmovement = new PIDCoefficients(0.06,0.0012,0.006); //.12 0 .035
   public PIDCoefficients PIDrotate = new PIDCoefficients(0.026,0.01,0.00025);  // .03 0 0
   PIDCoefficients PIDmovement_calculated = new PIDCoefficients(0,0,0);
   PIDCoefficients PIDrotate_calculated = new PIDCoefficients(0,0,0);
   long PIDTimeCurrent;
   long PIDTimeLast;
   double errorDistLast;
   double errorRotLast;
   double navAngleLast;
   boolean onTargetByAccuracy = false;

   /* Constructor */
   public AutoDrive(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
      v0 = 0.0;
      v1 = 0.0;
      v2 = 0.0;
      v3 = 0.0;
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
      updateRobotPosition();
      TelemetryMgr.Message(4, "AutoDrive", status);
      if (status==Status.DRIVING) {

      }
   }

   public void stop() {
   }

   // Determine motor speeds when under automatic control
   public void autoDrivePower () {
      if (robotPosition==null) return;   //todo: make sure this doesn't leave motors running
      double errorDist, deltaX, deltaY, errorRot, pDist, pRot, navAngle;
      deltaX = targetPos.X - robotPosition.X;  // error in x
      deltaY = targetPos.Y - robotPosition.Y;  // error in y
      TelemetryMgr.Message(7, "DeltaX", JavaUtil.formatNumber(deltaX, 2));
      TelemetryMgr.Message(7, "DeltaY", JavaUtil.formatNumber(deltaY, 2));
      errorRot = getHeadingError(targetPos.R);  // error in rotation   //20221222 added deltaheading!?
      errorDist = Math.sqrt(Math.pow(deltaX,2) + Math.pow(deltaY,2));  // distance (error) from xy destination

      //exit criteria if destination has been adequately reached   //todo: replace all this
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
//         navStep++;
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

   public void setMaxSpeed(double maxSpeed) {
      this.maxSpeed = maxSpeed;
   }

   public void resetPID() {
      PIDmovement_calculated.i = 0;
      PIDrotate_calculated.i = 0;
   }

   // Get heading error
   public double getHeadingError(double targetAngle) {
      if (robotPosition==null) return 0;  //todo: check if necessary
      double robotError;
      robotError = targetAngle - robotPosition.R;
      return Functions.normalizeAngle(robotError);
   }

   private void updateRobotPosition() {
      // todo: consider... if position is null, should targetposition be reset?
      if (parts.positionMgr.hasPosition()) robotPosition = parts.positionMgr.robotPosition.clone();
      else robotPosition = null;
   }

   public class PositionError {
      Position errorXYR;
      Vector errorXR;
      public PositionError() {
         errorXYR = new Position();
         errorXR = new Vector();
      }
      public PositionError(Position pos, Vector vec) {
         errorXYR = pos;
         errorXR = vec;
      }
      public PositionError(Vector vec, double heading) {
         errorXR = vec;
         errorXYR = new Position( vec.X*Math.cos(Math.toRadians(vec.A)),
                                  vec.X*Math.sin(Math.toRadians(vec.A)),
                                    heading   );
      }
      public PositionError(Position pos) {
         errorXYR = pos;
         errorXR = new Vector(Math.sqrt(Math.pow(pos.X,2)+Math.pow(pos.Y,2)), Math.toDegrees(Math.atan2(pos.X, pos.Y)));
         //=DEGREES(ATAN2(A6,B6))
      }
   }

   public class Vector {
      public double X, A;
      public Vector(double X, double A){
         this.X = X;
         this.A = A;
      }
      public Vector(){
         X = 0;
         A = 0;
      }
   }

   public enum Status {
      IDLE,
      DRIVING,
      SUCCESS,
      TIMEOUT,
      CANCELED,
      POSLOST
   }

}