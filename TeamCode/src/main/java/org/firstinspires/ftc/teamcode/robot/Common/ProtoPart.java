package org.firstinspires.ftc.teamcode.robot.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.Position;
import org.firstinspires.ftc.teamcode.robot.DiscShooter.AprilTag;

public class ProtoPart implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;

   /* Constructor */
   public ProtoPart(Parts parts){
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