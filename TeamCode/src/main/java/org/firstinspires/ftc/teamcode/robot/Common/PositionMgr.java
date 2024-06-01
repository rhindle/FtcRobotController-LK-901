package org.firstinspires.ftc.teamcode.robot.Common;

import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.Position;

// 20240601 - Just started working on this
public class PositionMgr implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;
   public Position robotPosition;
   public Position odoPosition;
   public Position slamraPosition;
   public Position tagPosition;

   public Position fieldStartPosition;
   public Position odoRobotOffset;
   public Position slamraRobotOffset;

   public int odoPriority = 1;
   public int slarmaPriority = 2;
   public int tagPriority = 3;
   public Boolean prioritizeSlamraR = false;

   /* Constructor */
   public PositionMgr(Parts parts){
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
      runLoop();
   }

   public void preRun() {
   }

   public void runLoop() {
      if (parts.useODO) odoPosition=parts.odometry.odoRobotPosition;
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

   Position normalUpdate() {
      // This logic should really be cleaned up...
      // First deal with all nulls/
      if (odoPosition == null && slamraPosition == null && tagPosition == null) return null;
      // Then deal with only one usable position
      if (odoPosition == null && tagPosition == null) return slamraPosition;
      if (slamraPosition == null && tagPosition == null) return odoPosition;
      if (odoPosition == null && slamraPosition == null) return tagPosition;
      // Then deal with two usable positions
      if (odoPosition == null) { if (slarmaPriority < tagPriority) return slamraPosition; else return tagPosition; }
      if (slamraPosition == null) { if (odoPriority < tagPriority) return odoPosition; else return tagPosition; }
      if (tagPosition == null) { if (odoPriority < slarmaPriority) return odoPosition; else return slamraPosition; }
      // All three are valid, so one final test
      if (odoPriority < slarmaPriority && odoPriority < tagPriority) return odoPosition;
      if (slarmaPriority < tagPriority) return slamraPosition;
      return tagPosition;
   }
}