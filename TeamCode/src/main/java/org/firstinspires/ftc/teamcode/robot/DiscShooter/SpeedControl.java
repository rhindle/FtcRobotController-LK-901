package org.firstinspires.ftc.teamcode.robot.DiscShooter;

import org.firstinspires.ftc.teamcode.robot.Common.Parts;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.Functions;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;

public class SpeedControl implements PartsInterface {

   /* The purpose of this class is to limit the speed of the robot as it gets near the perimeter of the field (to minimize slamming)
      This is accomplished by setting two square "fences" in the interior of the field. When the robot is outside the "fence",
      it is limited to a slower speed unless its direction is back toward the "fence".
    */

   /* Public OpMode members. */
   public Parts parts;

   public Fence fenceInner = new Fence(0.5, new Position(-24,23), new Position(-70,-23));
   public Fence fenceOuter = new Fence(0.25, new Position(-12,35), new Position(-82,-35));

   /* Constructor */
   public SpeedControl(Parts parts){
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
   }

   public void stop() {
   }

   public double checkFences (Position currentPosition, double direction) {
      // assumes that the controls (and thus direction) are robot centric (vs. field centric)
      // which means the direction has to be manipulated to a field centric direction
      direction += currentPosition.R;
      double outerSpeed = fencedSpeed(currentPosition, direction, fenceOuter);
      double innerSpeed = fencedSpeed(currentPosition, direction, fenceInner);
      if (outerSpeed != 1) return outerSpeed;
      if (innerSpeed != 1) return innerSpeed;
      return 1;
   }

   public double checkFences (Position currentPosition, double direction, boolean fieldCentric) {
      if (fieldCentric) {
         // if the controls are field centric, direction is based on the field (i.e., robot angle is 0)
         return checkFences(currentPosition.withR(0), direction);
      }
      else {
         return checkFences(currentPosition, direction);
      }
   }

   public double fencedSpeed (Position currentPosition, double direction, Fence fence) {
      boolean outside=false;
      direction = Functions.normalizeAngle(direction);
      //top, bottom, left, right
      if (currentPosition.X > fence.xMax && (direction > -95 && direction < 95)) outside=true;
      if (currentPosition.X < fence.xMin && (direction < -85 || direction > 85)) outside=true;
      if (currentPosition.Y > fence.yMax && (direction > -5 || direction < -175)) outside=true;
      if (currentPosition.Y < fence.yMin && (direction <  5 || direction >  175)) outside=true;

      return outside ? fence.outsideSpeed : fence.insideSpeed;
   }

   public class Fence {

      public double insideSpeed, outsideSpeed;
      public double xMax, xMin, yMax, yMin;

      public Fence(double speed, double xMax, double xMin, double yMax, double yMin) {
         this.outsideSpeed = speed;
         this.insideSpeed = 1;
         this.xMax = xMax;
         this.xMin = xMin;
         this.yMax = yMax;
         this.yMin = yMin;
      }

      public Fence(double speed, Position topLeft, Position bottomRight) {
         this.outsideSpeed = speed;
         this.insideSpeed = 1;
         this.xMax = topLeft.X;
         this.xMin = bottomRight.X;
         this.yMax = topLeft.Y;
         this.yMin = bottomRight.Y;
      }

      public Fence(double outsideSpeed, double insideSpeed, Position topLeft, Position bottomRight) {
         this.outsideSpeed = outsideSpeed;
         this.insideSpeed = insideSpeed;
         this.xMax = topLeft.X;
         this.xMin = bottomRight.X;
         this.yMax = topLeft.Y;
         this.yMin = bottomRight.Y;
      }

//      public Fence() {
//         speed = 1;
//         xMax = 0;
//         xMin = 0;
//         yMax = 0;
//         yMin = 0;
//      }
   }
}