package org.firstinspires.ftc.teamcode.robot.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.robot.DiscShooter.DSAprilTag;
import org.firstinspires.ftc.teamcode.robot.DiscShooter.DSLed;
import org.firstinspires.ftc.teamcode.robot.DiscShooter.DSMisc;
import org.firstinspires.ftc.teamcode.robot.DiscShooter.DSShooter;
import org.firstinspires.ftc.teamcode.robot.DiscShooter.DSSpeedControl;

public class Parts implements PartsInterface {

   /* Public OpMode members. */
   public boolean useODO = false; //true;
   public boolean reverseDrive = false;
   public boolean useDistanceSensors = true;
   public boolean useDriveEncoders = true;
   public boolean useSlamra = false;
   public boolean useAprilTag = false;
   public boolean useNeoMatrix = false;
   public Position robotPosition = new Position();
   public Position slamraPosition;
   public Position fieldStartPosition; // = new Position();
   public Position odoRobotOffset;
   public Position slamraRobotOffset;

   public LinearOpMode opMode;
   public Robot robot;
   public ButtonMgr buttonMgr;
   public Sensors sensors;
   public Controls controls;
   public Drivetrain drivetrain;
   public Navigator navigator;  // needs to be replaced
   public AutoDrive autoDrive;
   public UserDrive userDrive;
   public Odometry odometry;
   public Slamra slamra;
   public TelemetryMgr telemetryHandler;
   public NeoMatrix neo;
   public PositionMgr positionMgr;

   //---DiscShooter Unique
   public DSAprilTag dsApriltag;
   public DSLed dsLed;
   public DSShooter dsShooter;
   public DSSpeedControl dsSpeedControl;
   public DSMisc dsMisc;
   //---

   public boolean isSetup = false;
   public robotType rType;

   /* Constructor */
   public Parts(LinearOpMode opMode, robotType rType){
      construct(opMode, rType);
   }

   void construct(LinearOpMode opMode, robotType rType){
      this.opMode = opMode;
      this.rType = rType;
      telemetryHandler = new TelemetryMgr(opMode);
   }

   public void setup(){
//      // We do this after the construct because we may want to adjust some settings before creating all the sub-parts
//      if (isSetup) {
//         //throw new RuntimeException("Parts can only be setup once");
//         return;
//      }
//      isSetup = true;
//      robot = new Robot(this);
//      buttonMgr = new ButtonMgr(opMode);
//      sensors = new Sensors(this);
////      controls = new Controls_2(this);
//      controls = new Controls(this);
//      drivetrain = new Drivetrain(this);
//
//      odometry = new Odometry_(this);
//      odometry.odoFieldStart = fieldStartPosition;//.clone();
//      odometry.odoRobotOffset = odoRobotOffset;//.clone();
//
//      navigator = new Navigator(this);
//
//      if (useSlamra) {
//         slamra = new Slamra(this);
//         slamra.slamraFieldStart = fieldStartPosition;//.clone();
//         slamra.slamraRobotOffset = slamraRobotOffset;//.clone();
//      }
//
//      switch (rType) {
//         case GOCANUM:
//            break;
//         case ANDYMARK:
//            break;
//         case GENERIC:
//            break;
//         default:
//      }
   }

   public void initialize() {
   }

   public void preInit() {
//      robot.init();
//      sensors.init();
//      if (useSlamra) slamra.init();
   }

   public void initLoop() {
   }

   public void preRun() {
//      drivetrain.init();
//      odometry.init();
//      navigator.init();
//
//      odometry.loop();  // get some things squared away before the real program runs
//      navigator.loop();
//      if (useSlamra) slamra.onStart();
   }

   public void runLoop() {
//      robot.loop();
//      sensors.loop();
//      buttonMgr.loop();
//      odometry.loop();
//      if (useSlamra) slamra.loop();
//      controls.loop();
//      navigator.loop();
   }

   public void stop() {
//      if (useSlamra) slamra.onStop();
   }

   public enum robotType {
      GOCANUM,
      ANDYMARK,
      GENERIC
   }

}