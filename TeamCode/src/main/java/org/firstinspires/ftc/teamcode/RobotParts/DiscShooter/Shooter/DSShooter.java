package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.NavigationTarget;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;

public class DSShooter implements PartsInterface {

   /* Settings */
   static final double pusherRetracted                 = 0;
   static final double pusherExtended                  = 0.8;
   static final int pusherSweepTime                    = 200;
   static final int pusherAutoCycles                   = 5;

   static final double gateOpen                        = 1;
   static final double gateClosed                      = 0;
   static final int gateSweepTime                      = 325;

   static final double spinnerRPM                      = 3600;
   static final double spinnerTolerance                = 150;
   public static PIDFCoefficients spinnerPID    = new PIDFCoefficients(100,0,0,12.4);

   final double ingesterPower                   = 1;

   static Position autoLaunchPos                       = new Position(-24, 0, 0);  // must be updated!!

   /* Internal use */
   private static DcMotorEx motorSpinner;
   private DcMotorEx motorIngester;
   private static Servo servoPusher;
   private static Servo servoGate;
   private static double spinRPMset;
   private static long gateTimer = System.currentTimeMillis();
   private static long pusherTimer = System.currentTimeMillis();
   static double spinMultiplier;
//   int stateShoot1Step = 0;
//   int stateShoot3Step = 0;
//   int statePushStep = 0;
//   int stateFullAuto = 0;
//   int cycleCount = 0;
   public static long cancelTimer = System.currentTimeMillis();

   /* Public OpMode members. */
   public static Parts parts;

   /* Constructor */
   public DSShooter(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
      initMotors();
      spinMultiplier = 60.0 / 28.0 * 1.0;  // ticksPerRev * gearRatio;
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
      TelemetryMgr.message(TelemetryMgr.Category.BASIC, "SpinnerRPM", getSpinnerRPM());
      Push.stateMachineAutoPush();
      Shoot1.stateMachineShoot1();
      Shoot3.stateMachineShoot3();
      FullAuto.stateMachineFullAuto();
   }

   public void stop() {
      stopMotors();
   }

   public void initMotors () {
      motorSpinner = parts.robot.motor0B;
      motorIngester = parts.robot.motor1B;
      servoGate = parts.robot.servo4B;
      servoPusher = parts.robot.servo0B;

      stopMotors();

      motorIngester.setDirection(DcMotorEx.Direction.REVERSE);
      motorIngester.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
      motorIngester.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

      motorSpinner.setDirection(DcMotorEx.Direction.REVERSE);
      motorSpinner.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, spinnerPID);
      motorSpinner.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
      motorSpinner.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
      motorSpinner.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

      servoPusher.setPosition(pusherRetracted);
      servoGate.setPosition(gateClosed);
   }

   public void startPush () {
      Push.statePushStep = 1;
   }
   public void startShoot1 () {
      Shoot1.stateShoot1Step = 1;
   }
   public void startShoot3 () {
      Shoot3.stateShoot3Step = 1;
   }
   public void startFullAuto () {
      FullAuto.stateFullAuto = 1;
   }

   public void stopMotors() {
      motorIngester.setPower(0);
      motorSpinner.setPower(0);
   }

   static boolean isSpinnerInTolerance() {
      //!!!!!!
//      return true;
      return isSpinnerInTolerance(spinRPMset, spinnerTolerance);
   }
   static boolean isSpinnerInTolerance(double targetRPM, double tolerance) {
      return Math.abs(getSpinnerRPM() - targetRPM) <= tolerance;
   }

   static double getSpinnerRPM() {
      return motorSpinner.getVelocity() * spinMultiplier;
   }

   public static void spinnerOn(double rpm) {
      spinRPMset = rpm;
      motorSpinner.setVelocity(rpm / spinMultiplier);
   }
   public static void spinnerOn() {
      spinnerOn(spinnerRPM);
   }
   public static void spinnerOff() {
      motorSpinner.setPower(0);
   }
   public void intakeOn() {
      motorIngester.setPower(ingesterPower);
   }
   public void intakeReverse() {
      motorIngester.setPower(-ingesterPower);
   }
   public void intakeOff() {
      motorIngester.setPower(0);
   }

   public void armShooter() {
      retractPusher();
      openGate();
      spinnerOn();
   }
   public void disarmShooter() {
      retractPusher();
      closeGate();
      spinnerOff();
   }

   public static void openGate() {
      setGateServo(gateOpen);
   }
   public static void closeGate() {
      setGateServo(gateClosed);
   }
   public static void extendPusher() {
      setPusherServo(pusherExtended);
   }
   public static void retractPusher() {
      setPusherServo(pusherRetracted);
   }

   public static void setGateServo(double newPosition) {
      if (isServoAtPosition(servoGate, newPosition)) return;  // has already been set (but not necessarily done moving)
      servoGate.setPosition(newPosition);
      gateTimer = System.currentTimeMillis() + gateSweepTime;
   }

   public static void setPusherServo(double newPosition) {
      if (isServoAtPosition(servoPusher, newPosition)) return;  // has already been set (but not necessarily done moving)
      servoPusher.setPosition(newPosition);
      pusherTimer = System.currentTimeMillis() + pusherSweepTime;
   }

   public static boolean isGateOpen() {
      return isServoAtPosition(servoGate,gateOpen,gateTimer);
   }
   public boolean isGateClosed() {
      return isServoAtPosition(servoGate,gateClosed,gateTimer);
   }
   public boolean isGateDoneMoving () {
      return System.currentTimeMillis() >= gateTimer;
   }

   public static boolean isPusherExtended() {
      return isServoAtPosition(servoPusher,pusherExtended,pusherTimer);
   }
   public static boolean isPusherRetracted() {
      return isServoAtPosition(servoPusher,pusherRetracted,pusherTimer);
   }
   public boolean isPusherDoneMoving () {
      return System.currentTimeMillis() >= pusherTimer;
   }

   public static boolean isServoAtPosition(Servo servo, double comparePosition, long servoTimer) {
      return isServoAtPosition(servo.getPosition(), comparePosition) && System.currentTimeMillis() >= servoTimer;
   }
   public static boolean isServoAtPosition(Servo servo, double comparePosition) {
      return isServoAtPosition(servo.getPosition(), comparePosition);
   }
   public static boolean isServoAtPosition(double servoPosition, double comparePosition) {
      return(Math.round(servoPosition*100.0) == Math.round(comparePosition*100.0));
   }

//   public void stateMachineAutoPush() {
//      if (statePushStep == -9) {
//         retractPusher();
//         statePushStep = -1;
//      }
//      if (statePushStep < 1 || statePushStep > 999) return;  // not running
//
//      if (statePushStep == 1) {                 // extend pusher
//         extendPusher();
//         statePushStep++;
//      }
//      if (statePushStep == 2) {                 // wait for complete, retract
//         if (isPusherExtended()) {
//            retractPusher();
//            statePushStep++;
//         }
//      }
//      if (statePushStep == 3) {                // wait for complete
//         if (isPusherRetracted()) {
//            statePushStep=1000;
//         }
//      }
//   }

//   public void stateMachineShoot1() {
//      if (stateShoot1Step == -9) {
//         spinnerOff();
//         retractPusher();
//         // do we want to close the gate?  Ring might be in there...
//         stateShoot1Step = -1;
//      }
//      if (stateShoot1Step < 1 || stateShoot1Step > 999) return;  // not running
//
//      if (stateShoot1Step == 1) {                         // cancel other state machines if needed
//         if (stateShoot3Step > 0 && stateShoot3Step <999) {
//            cancelTimer = System.currentTimeMillis() + 1000;
//            stateShoot3Step = -9;
//         }
//         if (System.currentTimeMillis() >= cancelTimer) stateShoot1Step++;
//      }
//      if (stateShoot1Step == 2) {                 // open gate, start spinner
//         statePushStep = -9;   // cancel any ongoing pusher movement
//         openGate();
//         spinnerOn();
//         retractPusher();
//         stateShoot1Step++;
//      }
//      if (stateShoot1Step == 3) {                 // wait for gate up, spinner at rpm
//         if (isGateOpen() && isPusherRetracted() && isSpinnerInTolerance()) {
//            stateShoot1Step++;
//            statePushStep = 1;    // start the pusher state machine
//         }
//      }
//      if (stateShoot1Step == 4) {                 // wait for pusher machine to complete
//         if (statePushStep <= 0) stateShoot1Step = -9;   //cancel if problem
//         if (statePushStep == 1000) stateShoot1Step = 1000;
//      }
//   }

//   public void stateMachineFullAuto() {
//
//      if (stateFullAuto == -9) {
//         parts.autoDrive.setAutoDrive(false);
//      }
//      if (stateFullAuto < 1 || stateFullAuto > 999) return;  // not running
//
//      if (!parts.positionMgr.hasPosition()) stateFullAuto = -9;    // cancel running if no navigation
//
//      if (stateFullAuto == 1) {
//         if (Shoot1.stateShoot1Step > 0 && Shoot1.stateShoot1Step <999) {
//            cancelTimer = System.currentTimeMillis() + 1000;
//            Shoot1.stateShoot1Step = -9;
//         }
//         if (stateShoot3Step > 0 && stateShoot3Step <999) {
//            cancelTimer = System.currentTimeMillis() + 1000;
//            stateShoot3Step = -9;
//         }
//         if (System.currentTimeMillis() >= cancelTimer) stateFullAuto=2;
//      }
//
//      if (stateFullAuto == 2) {                                    // navigate to launch position
//         parts.autoDrive.setNavTarget(new NavigationTarget(autoLaunchPos, parts.dsMisc.toleranceHigh));
//         stateFullAuto++;
//      }
//      if (stateFullAuto == 3) {                                   // wait until reach position then start shooting
//         if (parts.autoDrive.onTargetByAccuracy) {
//            stateShoot3Step = 1;
//            stateFullAuto++;
//         }
//      }
//      if (stateFullAuto == 4) {                                 // start blasting
//         stateShoot3Step = 1;
//         stateFullAuto++;
//      }
//      if (stateFullAuto == 5) {
//         if (stateShoot3Step == 1000) {
//            parts.autoDrive.setAutoDrive(false);
//            closeGate();
//            spinnerOff();
//            retractPusher();
//            stateFullAuto=1000;
//         }
//      }
//
//   }

   public void cancelStateMachines() {
      Shoot1.stateShoot1Step = -9;
      Shoot3.stateShoot3Step = -9;
      Push.statePushStep = -9;
      FullAuto.stateFullAuto = -9;
   }

//   public void stateMachineShoot3() {
//      if (stateShoot3Step == -9) {
//         spinnerOff();
//         retractPusher();
//         // do we want to close the gate?  Ring might be in there...
//         stateShoot3Step = -1;
//      }
//      if (stateShoot3Step < 1 || stateShoot3Step > 999) return;  // not running
//
//      if (stateShoot3Step == 1) {                                       // cancel other state machines if needed
//         if (Shoot1.stateShoot1Step > 0 && Shoot1.stateShoot1Step <999) {
//            cancelTimer = System.currentTimeMillis() + 1000;
//            Shoot1.stateShoot1Step = -9;
//         }
//         if (System.currentTimeMillis() >= cancelTimer) stateShoot3Step++;
//      }
//      if (stateShoot3Step == 2) {                 // open gate, start spinner
//         Push.statePushStep = -9;   // cancel any ongoing pusher movement
//         cycleCount = 0;
//         openGate();
//         spinnerOn();
//         retractPusher();
//         stateShoot3Step++;
//      }
//      if (stateShoot3Step == 3) {                 // wait for gate up, spinner at rpm
//         if (isGateOpen() && isPusherRetracted() && isSpinnerInTolerance()) {
//            stateShoot3Step++;
//            Push.statePushStep = 1;    // start the pusher state machine
//         }
//      }
//      if (stateShoot3Step == 4) {                 // wait for pusher machine to complete
//         if (Push.statePushStep <= 0) stateShoot3Step = -9;   //cancel if problem
//         if (Push.statePushStep == 1000) {
//            cycleCount++;
//            Push.statePushStep = 1;      //restart pusher
//         }
//         if (cycleCount == pusherAutoCycles) stateShoot3Step = 1000;
//      }
//   }

}