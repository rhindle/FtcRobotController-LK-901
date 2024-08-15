package org.firstinspires.ftc.teamcode.robot.Common;

import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.Position;

// 20240601 - Just started working on this
public class PositionMgr implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;
   public Position robotPosition;
   public Position odoPosition;
   public Position slamraPosition;
   public Position tagPosition;
   //public Position encoderPosition;

   public Position fieldStartPosition;
//   public Position odoRobotOffset;
//   public Position slamraRobotOffset;

//   public int odoPriority = 1;
//   public int slarmaPriority = 2;
//   public int tagPriority = 3;
   public PosSource[] priorityList = {PosSource.ODO, PosSource.SLAMRA, PosSource.TAG};
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
      }
      if (parts.useAprilTag) tagPosition=parts.apriltag.tagRobotPosition;
      TelemetryMgr.Message(6, "pmgr:odo", (odoPosition==null) ? "(null)" : odoPosition.toString(2));
      TelemetryMgr.Message(6, "pmgr:slm", (slamraPosition==null) ? "(null)" : slamraPosition.toString(2));
      TelemetryMgr.Message(6, "pmgr:tag", (tagPosition==null) ? "(null)" : tagPosition.toString(2));
      robotPosition = normalUpdate();
      TelemetryMgr.Message(6, "pmgr:fnl", (robotPosition==null) ? "(null)" : robotPosition.toString(2));
   }

   public void stop() {
   }

   public Boolean hasPosition () {
      return (robotPosition!=null);
   }

   Position normalUpdate() {
      posSource = returnPrioritySource();
      switch (posSource) {
         case NONE:
            return null;
         case ODO:
            if (prioritizeSlamraRforODO && slamraPosition!=null) {
               return odoPosition.withR(slamraPosition.R);
            }
            return odoPosition;
         case SLAMRA:
            //if (prioritizeIMUforSLAMRA) return slamraPosition.withR() //todo: finish this
            return slamraPosition;
         case TAG:
            return tagPosition;
         default:
            return null;
      }
   }

//   PosSource returnPrioritySource() {
//      // This logic should really be cleaned up...
//      // First deal with all nulls/
//      if (odoPosition == null && slamraPosition == null && tagPosition == null) return PosSource.NONE;
//      // Then deal with only one usable position
//      if (odoPosition == null && tagPosition == null) return PosSource.SLAMRA;
//      if (slamraPosition == null && tagPosition == null) return PosSource.ODO;
//      if (odoPosition == null && slamraPosition == null) return PosSource.TAG;
//      // Then deal with two usable positions
//      if (odoPosition == null) { if (slarmaPriority < tagPriority) return PosSource.SLAMRA; else return PosSource.TAG; }
//      if (slamraPosition == null) { if (odoPriority < tagPriority) return PosSource.ODO; else return PosSource.TAG; }
//      if (tagPosition == null) { if (odoPriority < slarmaPriority) return PosSource.ODO; else return PosSource.SLAMRA; }
//      // All three are valid, so one final test
//      if (odoPriority < slarmaPriority && odoPriority < tagPriority) return PosSource.ODO;
//      if (slarmaPriority < tagPriority) return PosSource.SLAMRA;
//      return PosSource.TAG;
//   }

   PosSource returnPrioritySource() {
      for (PosSource source : priorityList) {
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
      ENCODER
   }

}