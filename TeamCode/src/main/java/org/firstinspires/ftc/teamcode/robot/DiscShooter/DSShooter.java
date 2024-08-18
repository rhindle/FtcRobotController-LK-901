package org.firstinspires.ftc.teamcode.robot.DiscShooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Common.Parts;
import org.firstinspires.ftc.teamcode.robot.Common.TelemetryMgr;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.Position;

public class DSShooter implements PartsInterface {

   /* Settings */
   final double pusherRetracted                 = 0;
   final double pusherExtended                  = 0.8;
   final int pusherSweepTime                    = 200;
   final int pusherAutoCycles                   = 5;

   final double gateOpen                        = 1;
   final double gateClosed                      = 0;
   final int gateSweepTime                      = 325;

   final double spinnerRPM                      = 3600;
   final double spinnerTolerance                = 150;
   public static PIDFCoefficients spinnerPID    = new PIDFCoefficients(100,0,0,12.4);

   final double ingesterPower                   = 1;

   Position autoLaunchPos                       = new Position(-24, 0, 0);  // must be updated!!

   /* Internal use */
   private DcMotorEx motorSpinner, motorIngester;
   private Servo servoPusher, servoGate;
   private double spinRPMset;
   private long gateTimer = System.currentTimeMillis();
   private long pusherTimer = System.currentTimeMillis();
   double spinMultiplier;
   int stateShoot1Step = 0;
   int stateShoot3Step = 0;
   int statePushStep = 0;
   int stateFullAuto = 0;
   int cycleCount = 0;
   private long cancelTimer = System.currentTimeMillis();

   /* Public OpMode members. */
   public Parts parts;

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
      TelemetryMgr.Message(4, "SpinnerRPM", getSpinnerRPM());
      stateMachineAutoPush();
      stateMachineShoot1();
      stateMachineShoot3();
      stateMachineFullAuto();
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

   public void stopMotors() {
      motorIngester.setPower(0);
      motorSpinner.setPower(0);
   }

   boolean isSpinnerInTolerance() {
      //!!!!!!
//      return true;
      return isSpinnerInTolerance(spinRPMset, spinnerTolerance);
   }
   boolean isSpinnerInTolerance(double targetRPM, double tolerance) {
      return Math.abs(getSpinnerRPM() - targetRPM) <= tolerance;
   }

   double getSpinnerRPM() {
      return motorSpinner.getVelocity() * spinMultiplier;
   }

   public void spinnerOn(double rpm) {
      spinRPMset = rpm;
      motorSpinner.setVelocity(rpm / spinMultiplier);
   }
   public void spinnerOn() {
      spinnerOn(spinnerRPM);
   }
   public void spinnerOff() {
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

   public void openGate() {
      setGateServo(gateOpen);
   }
   public void closeGate () {
      setGateServo(gateClosed);
   }
   public void extendPusher () {
      setPusherServo(pusherExtended);
   }
   public void retractPusher () {
      setPusherServo(pusherRetracted);
   }

   public void setGateServo (double newPosition) {
      if (isServoAtPosition(servoGate, newPosition)) return;
      servoGate.setPosition(newPosition);
      gateTimer = System.currentTimeMillis() + gateSweepTime;
   }

   public void setPusherServo (double newPosition) {
      if (isServoAtPosition(servoPusher, newPosition)) return;
      servoPusher.setPosition(newPosition);
      pusherTimer = System.currentTimeMillis() + pusherSweepTime;
   }

   public boolean isGateOpen() {
      return isServoAtPosition(servoGate,gateOpen,gateTimer);
      //return isServoAtPosition(servoGate, gateOpen) && isGateDoneMoving();
   }

   public boolean isPusherExtended() {
      return isServoAtPosition(servoPusher,pusherExtended,pusherTimer);
   }

   public boolean isPusherRetracted() {
      return isServoAtPosition(servoPusher,pusherRetracted,pusherTimer);
   }

   public boolean isGateDoneMoving () {
      return System.currentTimeMillis() >= gateTimer;
   }

   public boolean isPusherDoneMoving () {
      return System.currentTimeMillis() >= pusherTimer;
   }

   public boolean isServoAtPosition(Servo servo, double comparePosition, long servoTimer) {
      return isServoAtPosition(servo.getPosition(), comparePosition) && System.currentTimeMillis() >= servoTimer;
   }
   public boolean isServoAtPosition(Servo servo, double comparePosition) {
      return isServoAtPosition(servo.getPosition(), comparePosition);
   }
   public boolean isServoAtPosition(double servoPosition, double comparePosition) {
      return(Math.round(servoPosition*100.0) == Math.round(comparePosition*100.0));
   }

   public void stateMachineAutoPush() {
      if (statePushStep == -9) {
         retractPusher();
         statePushStep = -1;
      }
      if (statePushStep < 1 || statePushStep > 999) return;  // not running

      if (statePushStep == 1) {                 // extend pusher
         extendPusher();
         statePushStep++;
      }
      if (statePushStep == 2) {                 // wait for complete, retract
         if (isPusherExtended()) {
            retractPusher();
            statePushStep++;
         }
      }
      if (statePushStep == 3) {                // wait for complete
         if (isPusherRetracted()) {
            statePushStep=1000;
         }
      }
   }

   public void stateMachineShoot1() {
      if (stateShoot1Step == -9) {
         spinnerOff();
         retractPusher();
         // do we want to close the gate?  Ring might be in there...
         stateShoot1Step = -1;
      }
      if (stateShoot1Step < 1 || stateShoot1Step > 999) return;  // not running

      if (stateShoot1Step == 1) {                         // cancel other state machines if needed
         if (stateShoot3Step > 0 && stateShoot3Step <999) {
            cancelTimer = System.currentTimeMillis() + 1000;
            stateShoot3Step = -9;
         }
         if (System.currentTimeMillis() >= cancelTimer) stateShoot1Step++;
      }
      if (stateShoot1Step == 2) {                 // open gate, start spinner
         statePushStep = -9;   // cancel any ongoing pusher movement
         openGate();
         spinnerOn();
         retractPusher();
         stateShoot1Step++;
      }
      if (stateShoot1Step == 3) {                 // wait for gate up, spinner at rpm
         if (isGateOpen() && isPusherRetracted() && isSpinnerInTolerance()) {
            stateShoot1Step++;
            statePushStep = 1;    // start the pusher state machine
         }
      }
      if (stateShoot1Step == 4) {                 // wait for pusher machine to complete
         if (statePushStep <= 0) stateShoot1Step = -9;   //cancel if problem
         if (statePushStep == 1000) stateShoot1Step = 1000;
      }
   }

   public void stateMachineFullAuto() {

      if (stateFullAuto == -9) {
         parts.navigator.setAutoDrive(false);
      }
      if (stateFullAuto < 1 || stateFullAuto > 999) return;  // not running

      if (!parts.positionMgr.hasPosition()) stateFullAuto = -9;    // cancel running if no navigation

      if (stateFullAuto == 1) {
         if (stateShoot1Step > 0 && stateShoot1Step <999) {
            cancelTimer = System.currentTimeMillis() + 1000;
            stateShoot1Step = -9;
         }
         if (stateShoot3Step > 0 && stateShoot3Step <999) {
            cancelTimer = System.currentTimeMillis() + 1000;
            stateShoot3Step = -9;
         }
         if (System.currentTimeMillis() >= cancelTimer) stateFullAuto=2;
//         if (!parts.positionMgr.hasPosition()) stateFullAuto = -9;    // cancel if no navigation
      }

      if (stateFullAuto == 2) {                                    // navigate to launch position
         parts.navigator.setTargetAbsolute(autoLaunchPos);
         parts.navigator.setAccuracy(1);
         parts.navigator.setAutoDrive(true);
         stateFullAuto++;
      }
      if (stateFullAuto == 3) {                                   // wait until reach position then start shooting
         if (parts.navigator.isOnTargetByAccuracy()) {
            stateShoot3Step = 1;
            stateFullAuto++;
         }
      }
      if (stateFullAuto == 4) {                                 // start blasting
         stateShoot3Step = 1;
         stateFullAuto++;
      }
      if (stateFullAuto == 5) {
         if (stateShoot3Step == 1000) {
            parts.navigator.setAutoDrive(false);
            closeGate();
            spinnerOff();
            retractPusher();
            stateFullAuto=1000;
         }
      }

   }

   public void cancelStateMachines() {
     stateShoot1Step = -9;
     stateShoot3Step = -9;
     statePushStep = -9;
     stateFullAuto = -9;
   }

   public void stateMachineShoot3() {
      if (stateShoot3Step == -9) {
         spinnerOff();
         retractPusher();
         // do we want to close the gate?  Ring might be in there...
         stateShoot3Step = -1;
      }
      if (stateShoot3Step < 1 || stateShoot3Step > 999) return;  // not running

      if (stateShoot3Step == 1) {                                       // cancel other state machines if needed
         if (stateShoot1Step > 0 && stateShoot1Step <999) {
            cancelTimer = System.currentTimeMillis() + 1000;
            stateShoot1Step = -9;
         }
         if (System.currentTimeMillis() >= cancelTimer) stateShoot3Step++;
      }
      if (stateShoot3Step == 2) {                 // open gate, start spinner
         statePushStep = -9;   // cancel any ongoing pusher movement
         cycleCount = 0;
         openGate();
         spinnerOn();
         retractPusher();
         stateShoot3Step++;
      }
      if (stateShoot3Step == 3) {                 // wait for gate up, spinner at rpm
         if (isGateOpen() && isPusherRetracted() && isSpinnerInTolerance()) {
            stateShoot3Step++;
            statePushStep = 1;    // start the pusher state machine
         }
      }
      if (stateShoot3Step == 4) {                 // wait for pusher machine to complete
         if (statePushStep <= 0) stateShoot3Step = -9;   //cancel if problem
         if (statePushStep == 1000) {
            cycleCount++;
            statePushStep = 1;      //restart pusher
         }
         if (cycleCount == pusherAutoCycles) stateShoot3Step = 1000;
      }
   }

}