package org.firstinspires.ftc.teamcode.RobotParts.Common;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.Functions;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr.Category;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ImuMgr implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   public boolean disableIMUupdate = false;
   Orientation angles;
   double imuHeadingRaw;
   double imuFieldOffset = 0;
   public double imuRobotHeading;
   YawPitchRollAngles orientation;
   AngularVelocity angularVelocity;

   /* Constructor */
   public ImuMgr(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
      // Read IMU - once per cycle!
      if (!disableIMUupdate) {
         updateImuHeading();
//         TelemetryMgr.message(Category.IMU, "Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
         TelemetryMgr.message(Category.IMU_EXT, "robotHeading", "%.2f Deg. (Heading)", imuRobotHeading);
//         TelemetryMgr.message(Category.IMU_EXT,"Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
//         TelemetryMgr.message(Category.IMU_EXT,"Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
//         TelemetryMgr.message(Category.IMU_EXT,"Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
//         TelemetryMgr.message(Category.IMU_EXT,"Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
//         TelemetryMgr.message(Category.IMU_EXT,"Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
      }
   }

   public void stop() {
   }

   private void updateImuHeading() {
      imuHeadingRaw = imuHeading(true);
      updateImuRobotHeading();
   }

   private void updateImuRobotHeading() {
      imuRobotHeading = Functions.normalizeAngle(imuHeadingRaw + imuFieldOffset);
   }

   private double imuHeading() {
      return imuHeading(false);
   }
   private double imuHeading(boolean readme) {

      // Old IMU code
      //      if (readme) angles = parts.robot.sensorIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      //      return angles.firstAngle;

      if (readme) angles = parts.robot.sensorIMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      return angles.firstAngle;

//      // Retrieve Rotational Angles and Velocities
//      if (readme) orientation = parts.robot.sensorIMU.getRobotYawPitchRollAngles();
////      angularVelocity = parts.robot.sensorIMU.getRobotAngularVelocity(AngleUnit.DEGREES);
//      return orientation.getYaw(AngleUnit.DEGREES);
   }

   public double returnImuHeadingRaw() {
      return imuHeadingRaw;
   }
   public double returnImuHeadingRaw(boolean forceRead) {
      if (forceRead) updateImuHeading();
      return imuHeadingRaw;
   }

   public double returnImuRobotHeading() {
      return imuRobotHeading;
   }
   public double returnImuRobotHeading(boolean forceRead) {
      if (forceRead) updateImuHeading();
      return imuRobotHeading;
   }

   public void setupFieldOffset(Position fieldPosition) {
      setupFieldOffset(fieldPosition.R);
   }
   public void setupFieldOffset(double fieldHeading) {
      if (disableIMUupdate) updateImuHeading();  // if it's not being updated every loop, force an update
      imuFieldOffset = fieldHeading - imuHeadingRaw;
      updateImuRobotHeading();
   }
}