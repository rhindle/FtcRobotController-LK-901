package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.robot.Universal.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Universal.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Universal.Tools.Functions;

public class Controls {

   Gamepad gamepad1;
   Gamepad gamepad2;
   Robot robot;
   ButtonMgr buttonMgr;
   Navigator3 navigator;
   public double DriveSpeed, DriveAngle, Rotate;

   public final double tileSize = 23.5;  //in inches

   /* Constructor */
   public Controls(Robot robot){//ButtonMgr buttonMgr, Navigator navigator){
      construct(robot);//buttonMgr, navigator);
   }

   void construct(Robot robot){//ButtonMgr buttonMgr, Navigator navigator){
      this.robot = robot;
      this.gamepad1 = robot.opMode.gamepad1;
      this.gamepad2 = robot.opMode.gamepad2;
      this.buttonMgr = robot.buttonMgr;
      this.navigator = robot.navigator;
   }

   void init() {

   }

   public void loop() {
      readAndAct();
      navigator.setUserDriveSettings(DriveSpeed, DriveAngle, Rotate);
   }

   public void readAndAct() {

      // TeleOp / normal drive

      // Get speed and direction from left stick
      DriveSpeed = JavaUtil.minOfList(JavaUtil.createListWith(1, Functions.mathHypotenuse(gamepad1.left_stick_x, gamepad1.left_stick_y)));
      DriveAngle = Math.atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y) / Math.PI * 180;
      // Get rotation from right stick
      Rotate = Math.pow(gamepad1.right_stick_x, 1);
      navigator.handleRotate(Rotate);

      // Toggle FCD
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.start))
         navigator.toggleFieldCentricDrive();

      // Toggle HeadingHold
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.back))
         navigator.toggleHeadingHold();

      // Store heading correction
      if (buttonMgr.wasReleased(1, ButtonMgr.Buttons.right_stick_button))
         navigator.setDeltaHeading();

      if (buttonMgr.wasReleased(1, ButtonMgr.Buttons.left_stick_button))
         navigator.toggleSnapToAngle();

      // This blob is for manually entering destinations by adjusting X, Y, Rot
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpad_up))
         navigator.setTargetByDeltaRelative(2,0,0);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpad_down))
         navigator.setTargetByDeltaRelative(-2,0,0);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpad_left))
         navigator.setTargetByDeltaRelative(0, 2,0);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpad_right))
         navigator.setTargetByDeltaRelative(0, -2,0);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.x)) {
//         navigator.setTargetByDeltaRelative(0, 0, 45);
         navigator.setTargetRotBySnapRelative (45);
         navigator.headingDelay = System.currentTimeMillis() + 500;  // workaround
      }
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.y)) {
//         navigator.setTargetByDeltaRelative(0, 0, -45);
         navigator.setTargetRotBySnapRelative (-45);
         navigator.headingDelay = System.currentTimeMillis() + 500;  // workaround
      }
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.a)) {
//         navigator.setTargetToCurrentPosition();
//         navigator.beginAutoDrive();
//         navigator.togglePositionHold();
         navigator.useAutoDistanceActivation = !navigator.useAutoDistanceActivation;
      }
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.b))
         //navigator.cancelAutoNavigation();
         navigator.togglePositionHold();
      if (!buttonMgr.isPressed(1, ButtonMgr.Buttons.right_bumper))
         navigator.setMaxSpeed(1);
      else
         navigator.setMaxSpeed(0.25);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.left_bumper))
         robot.sensors.readDistSensors();

      //20230911 - Added special chorded controls for PID tuning

      if (buttonMgr.isHeld(2, ButtonMgr.Buttons.right_bumper)) {
         if (!buttonMgr.isPressed(2, ButtonMgr.Buttons.back)) {
            if (buttonMgr.isPressed(2, ButtonMgr.Buttons.x)) navigator.PIDmovement.p -= 0.001;
            if (buttonMgr.isPressed(2, ButtonMgr.Buttons.y)) navigator.PIDmovement.i -= 0.00025;
            if (buttonMgr.isPressed(2, ButtonMgr.Buttons.b)) navigator.PIDmovement.d -= 0.00025;
         }
         else {
            if (buttonMgr.isPressed(2, ButtonMgr.Buttons.x)) navigator.PIDrotate.p -= 0.001;
            if (buttonMgr.isPressed(2, ButtonMgr.Buttons.y)) navigator.PIDrotate.i -= 0.00025;
            if (buttonMgr.isPressed(2, ButtonMgr.Buttons.b)) navigator.PIDrotate.d -= 0.00025;
         }
      }
      else if (buttonMgr.isHeld(2, ButtonMgr.Buttons.left_bumper)) {
         if (!buttonMgr.isPressed(2, ButtonMgr.Buttons.back)) {
            if (buttonMgr.isPressed(2, ButtonMgr.Buttons.x)) navigator.PIDmovement.p += 0.001;
            if (buttonMgr.isPressed(2, ButtonMgr.Buttons.y)) navigator.PIDmovement.i += 0.00025;
            if (buttonMgr.isPressed(2, ButtonMgr.Buttons.b)) navigator.PIDmovement.d += 0.00025;
         }
         else {
            if (buttonMgr.isPressed(2, ButtonMgr.Buttons.x)) navigator.PIDrotate.p += 0.001;
            if (buttonMgr.isPressed(2, ButtonMgr.Buttons.y)) navigator.PIDrotate.i += 0.0005;
            if (buttonMgr.isPressed(2, ButtonMgr.Buttons.b)) navigator.PIDrotate.d += 0.001;
         }
      }
      else {

         // AutoDrive Testing

         // Start auto navigation
         if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.start))
            navigator.beginAutoDrive();

         // Set to home position
         if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.right_bumper))
            navigator.setTargetToZeroPosition();

         // Begin scripted navigation
         if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.left_bumper))
            //navigator.beginScriptedNav();

         // Cancels auto navigation
         if (buttonMgr.isPressed(2, ButtonMgr.Buttons.b))
            navigator.cancelAutoNavigation();

         // Reset target navigation to present position
         if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.a))
            navigator.setTargetToCurrentPosition();

         // This blob is for manually entering destinations by adjusting X, Y, Rot
//      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpad_up))
//         navigator.setTargetByDelta((gamepad2.back ? 1 : tileSize/2.0),0,0);
//      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpad_down))
//         navigator.setTargetByDelta(-(gamepad2.back ? 1 : tileSize/2.0),0,0);
//      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpad_left))
//         navigator.setTargetByDelta(0, (gamepad2.back ? 1 : tileSize/2.0),0);
//      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpad_right))
//         navigator.setTargetByDelta(0, -(gamepad2.back ? 1 : tileSize/2.0),0);
         if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpad_left))
            navigator.setTargetByDelta((gamepad2.back ? 1 : tileSize / 2.0), 0, 0);
         if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpad_right))
            navigator.setTargetByDelta(-(gamepad2.back ? 1 : tileSize / 2.0), 0, 0);
         if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpad_up))
            navigator.setTargetByDelta(0, -(gamepad2.back ? 1 : tileSize / 2.0), 0);  //signs on these temporarily reversed until method fixed
         if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpad_down))
            navigator.setTargetByDelta(0, (gamepad2.back ? 1 : tileSize / 2.0), 0);

         if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.x))
            navigator.setTargetByDelta(0, 0, (gamepad2.back ? 2 : 45));
         if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.y))
            navigator.setTargetByDelta(0, 0, -(gamepad2.back ? 2 : 45));
      }
   }
}
