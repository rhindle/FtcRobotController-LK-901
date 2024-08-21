package org.firstinspires.ftc.teamcode.robot.DiscShooter;

import org.firstinspires.ftc.teamcode.robot.Common.Parts;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.NavigationTarget;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.PositionTolerance;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;

public class DSMisc implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   PositionTolerance toleranceImpossible = new PositionTolerance (0.5, 0.5, 250);
   PositionTolerance toleranceHigh = new PositionTolerance (1.0, 1.0, 250);
   PositionTolerance toleranceMedium = new PositionTolerance (2.0, 2.0, 125);
   PositionTolerance toleranceLow = new PositionTolerance(2.0,6.0,5.0,50);
   PositionTolerance toleranceTransition = new PositionTolerance(4.0,180.0,0);

   NavigationTarget test = new NavigationTarget(new Position(-24,0,0), toleranceTransition,1.0,5000,true);

   /* Constructor */
   public DSMisc(Parts parts){
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

}