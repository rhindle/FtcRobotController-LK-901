package org.firstinspires.ftc.teamcode.robot.DiscShooter;

import org.firstinspires.ftc.teamcode.robot.Common.Parts;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.Functions;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;

public class SpeedControl implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   // Array will be [0] speed multiplier, [1] top(maxX), [2] bottom(minX), [3] left(maxY), [4] right(minY)
   double[] fence1 = { 0.5,  -24.0, -70.0, 23.0, -23.0};
   double[] fence2 = { 0.25, -12.0, -82.0, 35.0, -35.0};

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
      //check outer fence (fence2) first
      double outer = fencedSpeed(currentPosition, direction, fence2);
      //then check inner fence (fence1)
      double inner = fencedSpeed(currentPosition, direction, fence1);

      if (outer != 1) return outer;
      if (inner != 1) return inner;
      return 1;
   }

   public double fencedSpeed (Position currentPosition, double direction, double[] fence) {
      boolean limited=false;
      direction = Functions.normalizeAngle(direction);
      //top
      if (currentPosition.X > fence[1] && (direction > -90 && direction < 90)) limited=true;
      //bottom
      if (currentPosition.X < fence[2] && (direction < -90 || direction > 90)) limited=true;
      //left
      if (currentPosition.Y > fence[3] && direction > 0) limited=true;
      //right
      if (currentPosition.Y < fence[4] && direction < 0) limited=true;

      return limited ? fence[0] : 1;
   }

}