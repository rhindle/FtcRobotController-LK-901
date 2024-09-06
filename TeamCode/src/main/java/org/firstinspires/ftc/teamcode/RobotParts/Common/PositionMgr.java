package org.firstinspires.ftc.teamcode.RobotParts.Common;

import org.firstinspires.ftc.teamcode.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr.Category;

public class PositionMgr implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;
   public Position robotPosition;
   public Position odoPosition;
   public Position slamraPosition;
   public Position tagPosition;
   public Position imuHeading;
   public Position headingOnly;
   public Position imusHeadingOnly;
   public Position slamraHeading;

   public PosSource[] priorityList = {PosSource.ODO, PosSource.SLAMRA, PosSource.TAG};
   public PosSource[] headingOnlyPriority = {PosSource.IMU, PosSource.SLAMRA_R, PosSource.ODO, PosSource.TAG};
   public PosSource[] imusOnlyPriority = {PosSource.IMU, PosSource.SLAMRA_R};
   public Boolean prioritizeSlamraRforODO = false;      // use Slamra R instead of IMU for ODO
   public Boolean prioritizeIMUforSLAMRA = false;       // use IMU instead of Slarma R for SLAMRA
   public PosSource posSource;

   /* Constructor */
   public PositionMgr(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
//      if (fieldStartPosition!=null) {
//         if (parts.useODO) parts.odometry.odoFieldStart=fieldStartPosition;
//         if (parts.useSlamra) parts.slamra.slamraFieldStart=fieldStartPosition;
//      }
   }

   public void preInit() {
   }

   public void initLoop() {
      runLoop();
   }

   public void preRun() {
   }

   public void runLoop() {
      if (parts.useODO) {
         odoPosition = parts.odometry.isOdoPositionGood() ? parts.odometry.odoRobotPosition : null;
      }
      if (parts.useSlamra) {
         slamraPosition = parts.slamra.isSlamraPositionGood() ? parts.slamra.slamraRobotPosition : null;
         slamraHeading = parts.slamra.slamraRobotPosition;
      }
      if (parts.useAprilTag) {
         tagPosition = parts.dsApriltag.tagRobotPosition;
      }
      if (parts.useIMU) {
         imuHeading = parts.imuMgr.returnImuRobotHeadingAsPosition();
      }
      TelemetryMgr.message(Category.POSITION, "odo", (odoPosition==null) ? "(null)" : odoPosition.toString(2));
      TelemetryMgr.message(Category.POSITION, "slm", (slamraPosition==null) ? "(null)" : slamraPosition.toString(2));
      TelemetryMgr.message(Category.POSITION, "tag", (tagPosition==null) ? "(null)" : tagPosition.toString(2));
      TelemetryMgr.message(Category.POSITION, "imu", (imuHeading==null) ? "(null)" : imuHeading.toString(2));
      imusHeadingOnly = headingOnlyUpdate(imusOnlyPriority);
      headingOnly = headingOnlyUpdate(headingOnlyPriority);
      robotPosition = normalUpdate();
      TelemetryMgr.message(Category.POSITION, "fnl", (robotPosition==null) ? "(null)" : robotPosition.toString(2));
      TelemetryMgr.message(Category.POSITION, "ims", (imusHeadingOnly==null) ? "(null)" : imusHeadingOnly.toString(2));
      TelemetryMgr.message(Category.POSITION, "hed", (headingOnly==null) ? "(null)" : headingOnly.toString(2));
   }

   public void stop() {
   }

   public Boolean hasPosition () {
      return (robotPosition!=null);
   }
   public Boolean noPosition () {
      return (robotPosition==null);
   }
   public Boolean hasHeading() {
      return (headingOnly!=null);
   }
   public Boolean hasImusHeading() {
      return (imusHeadingOnly!=null);
   }

   Position normalUpdate() {
      posSource = returnPrioritySource(priorityList);
      switch (posSource) {
         case ODO:
            if (prioritizeSlamraRforODO && slamraPosition!=null) {
               return odoPosition.withR(slamraPosition.R);
            }
            return odoPosition;
         case SLAMRA:
            //todo: finish/check this
            if (prioritizeIMUforSLAMRA && imuHeading!=null) {
               return slamraPosition.withR(imuHeading.R);
            }
            return slamraPosition;
         case TAG:
            return tagPosition;
         case NONE:
         default:
            return null;
      }
   }

   Position headingOnlyUpdate(PosSource[] list) {
      posSource = returnPrioritySource(list);
      switch (posSource) {
         case IMU:
            return new Position().withR(imuHeading.R);
         case ODO:
            return new Position().withR(odoPosition.R);
         case SLAMRA:
         case SLAMRA_R:
            return new Position().withR(slamraHeading.R);
         case TAG:
            return new Position().withR(tagPosition.R);
         case NONE:
         default:
            return null;
      }
   }

//   Position imusOnlyUpdate(PosSource[] list) {
//      posSource = returnPrioritySource(list);
//      switch (posSource) {
//         case IMU:
//            return new Position().withR(imuHeading.R);
//         case SLAMRA_R:
//            return new Position().withR(slamraHeading.R);
//         case NONE:
//         default:
//            return null;
//      }
//   }

   PosSource returnPrioritySource(PosSource[] list) {
      for (PosSource source : list) {
         switch (source) {
            case ODO:
               if (odoPosition!=null) return PosSource.ODO;
               break;
            case SLAMRA:
               if (slamraPosition!=null) return PosSource.SLAMRA;
               break;
            case TAG:
               if (tagPosition!=null) return PosSource.TAG;
               break;
            case IMU:
               if (imuHeading!=null) return PosSource.IMU;   // only valid for heading only
               break;
            case SLAMRA_R:
               if (slamraHeading!=null) return PosSource.SLAMRA_R;   // only valid for heading only
               break;
            default:
         }
      }
      return PosSource.NONE;
   }

   public enum PosSource {
      NONE,
      ODO,
      SLAMRA,
      TAG,
      ENCODER,
      IMU,
      SLAMRA_R
   }

}