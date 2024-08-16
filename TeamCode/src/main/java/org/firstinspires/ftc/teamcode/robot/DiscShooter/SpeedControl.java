package org.firstinspires.ftc.teamcode.robot.DiscShooter;

import org.firstinspires.ftc.teamcode.robot.Common.Parts;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.Functions;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;

public class SpeedControl implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   Fence fenceInner = new Fence(0.5, new Position(-24,23), new Position(-70,-23));
   Fence fenceOuter = new Fence(0.25, new Position(-12,35), new Position(-82,-35));
// Array will be [0] speed multiplier, [1] top(maxX), [2] bottom(minX), [3] left(maxY), [4] right(minY)
//   double[] fence1 = { 0.5,  -24.0, -70.0, 23.0, -23.0};
//   double[] fence2 = { 0.25, -12.0, -82.0, 35.0, -35.0};

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
      double outerSpeed = fencedSpeed(currentPosition, direction, fenceOuter);
      double innerSpeed = fencedSpeed(currentPosition, direction, fenceInner);
      if (outerSpeed != 1) return outerSpeed;
      if (innerSpeed != 1) return innerSpeed;
      return 1;
   }

   public double fencedSpeed (Position currentPosition, double direction, Fence fence) {
      boolean outside=false;
      direction = Functions.normalizeAngle(direction);
      //top. bottom, left, right
      if (currentPosition.X > fence.xMax && (direction > -90 && direction < 90)) outside=true;
      if (currentPosition.X < fence.xMin && (direction < -90 || direction > 90)) outside=true;
      if (currentPosition.Y > fence.yMax && direction > 0) outside=true;
      if (currentPosition.Y < fence.yMin && direction < 0) outside=true;

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