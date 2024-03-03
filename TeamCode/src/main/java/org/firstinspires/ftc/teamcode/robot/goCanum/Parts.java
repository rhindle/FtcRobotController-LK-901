package org.firstinspires.ftc.teamcode.robot.goCanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Universal.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Universal.Tools.Position;

public class Parts {

   /* Public OpMode members. */
   public boolean useODO = false; //true;
   public boolean reverseDrive = false;
   public boolean useDistanceSensors = true;
   public boolean useDriveEncoders = true;
   public Position robotPosition;

   public LinearOpMode opMode;
   public Robot robot;
   public ButtonMgr buttonMgr;
   public Sensors sensors;
   public Controls controls;
   public Drivetrain drivetrain;
   public Navigator navigator;
   public LocalizerOdo localizer;

   /* Constructor */
   public Parts(LinearOpMode opMode, robotType rType){
      construct(opMode, rType);
   }

   void construct(LinearOpMode opMode, robotType rType){
      this.opMode = opMode;

      robot = new Robot(this);
      buttonMgr = new ButtonMgr(opMode);
      sensors = new Sensors(this);
//      controls = new Controls_2(this);
      controls = new Controls(this);
      drivetrain = new Drivetrain(this);
      localizer = new LocalizerOdo(this);
      navigator = new Navigator(this);

      switch (rType) {
         case GOCANUM:
            break;
         case ANDYMARK:
            break;
         case GENERIC:
            break;
         default:
      }
   }

   public void preInit() {
      robot.init();
      sensors.init();
   }

   public void preRun() {
      drivetrain.init();
      localizer.init();
      navigator.init();

      localizer.loop();  // get some things squared away before the real program runs
      navigator.loop();
   }

   public void loop() {
      robot.loop();
      sensors.loop();
      buttonMgr.loop();
      localizer.loop();
      controls.loop();
      navigator.loop();
   }

   public enum robotType {
      GOCANUM,
      ANDYMARK,
      GENERIC
   }

}