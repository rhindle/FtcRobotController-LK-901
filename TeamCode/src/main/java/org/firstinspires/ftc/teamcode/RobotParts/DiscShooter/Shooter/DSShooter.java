package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.Common.TelemetryMgr;
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
   public static PIDFCoefficients spinnerPID           = new PIDFCoefficients(100,0,0,12.4);

   static final double ingesterPower                   = 1;

   static Position autoLaunchPos                       = new Position(-24, 0, 0);  // must be updated!!

   /* Internal use */
   private static DcMotorEx motorSpinner;
   private static DcMotorEx motorIngester;
   private static Servo servoPusher;
   private static Servo servoGate;
   private static double spinRPMset;
   private static long gateTimer = System.currentTimeMillis();
   private static long pusherTimer = System.currentTimeMillis();
   private static double spinMultiplier = 60.0 / 28.0 * 1.0;  // ticksPerRev * gearRatio;;
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
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   @SuppressLint("DefaultLocale")
   public void runLoop() {
      TelemetryMgr.message(TelemetryMgr.Category.DISCSHOOTER, "SpinnerRPM", getSpinnerRPM());
      Pusher.stateMachine();
      Shoot1.stateMachine();
      Shoot3.stateMachine();
      FullAuto.stateMachine();
      TelemetryMgr.message(TelemetryMgr.Category.DISCSHOOTER,
              "States: " +
                      "PU: " + String.format("%02d", Pusher.getState()) +
                      ", S1: " + String.format("%02d", Shoot1.getState()) +
                      ", S3: " + String.format("%02d", Shoot3.getState()) +
                      ", FA: " + String.format("%02d", FullAuto.getState()) );
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
      Pusher.start();
   }
   public void startShoot1 () {
      Shoot1.start();
   }
   public void startShoot3 () {
      Shoot3.start();
   }
   public void startFullAuto () {
      FullAuto.start();
   }

   public void cancelStateMachines() {
      Shoot1.stop();
      Shoot3.stop();
      Pusher.stop();
      FullAuto.stop();
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
}