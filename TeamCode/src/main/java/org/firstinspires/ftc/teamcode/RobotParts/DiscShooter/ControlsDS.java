package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter;

import android.graphics.Color;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr.State;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Controls;
import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.RobotParts.DiscShooter.Shooter.DSShooter;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.DriveData;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.NavigationTarget;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;

public class ControlsDS extends Controls {

   boolean isStopped = false;
   boolean guestOK, teamOK;

   public ControlsDS(Parts parts) {
      super(parts);
   }

   @Override
   public void runLoop() {
      driveData = new DriveData();
      userInput();
//      parts.userDrive.setUserDriveSettings(driveSpeed, driveAngle, rotate);
      parts.userDrive.setUserDriveSettings(driveData);
   }

   @Override
   public void userInput() {

      //      // Get speed and direction from left stick
      ////      driveSpeed = JavaUtil.minOfList(JavaUtil.createListWith(1, Functions.mathHypotenuse(gamepad1.left_stick_x, gamepad1.left_stick_y)));
      //      driveSpeed = Functions.mathHypotenuse(gamepad1.left_stick_x, gamepad1.left_stick_y);
      //      driveAngle = Math.toDegrees(Math.atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y));  //gamepad.x = left/right = robot.y
      //      // Get rotation from right stick
      ////      rotate = Math.pow(gamepad1.right_stick_x, 1);
      //      rotate = gamepad1.right_stick_x;
      ////      parts.navigator.handleRotate(rotate);

      //forza - can't work with dead man switch, so simulate
      //DriveData driveDataTeam = new DriveData(gamepad1.left_stick_y, 0, gamepad1.left_stick_x, gamepad1.right_stick_x);
      DriveData driveDataTeam = new DriveData(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
      DriveData driveDataGuest = new DriveData(gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.right_stick_x);

      /* Controls will be in the style of dead man switches */
      guestOK = buttonMgr.isPressed(1,Buttons.left_trigger);
      teamOK = buttonMgr.isPressed(1,Buttons.right_trigger);

      /* If neither dead man is pressed, stop everything (if needed) and proceed no further */
      if (!guestOK && !teamOK) {
         stopEverything();
         parts.dsLed.updateGraphic('3', Color.rgb(1,0,0));
         return;
      }

      /* If we made it here, things aren't necessarily stopped any more (this affect the e-stop method */
      isStopped = false;
      if (guestOK && teamOK) {
         parts.dsLed.updateGraphic('3', Color.rgb(0,0,2));
      }
      else if (guestOK) {
         parts.dsLed.updateGraphic('3', Color.rgb(0,2,0));
      }
      else {
         parts.dsLed.updateGraphic('3', Color.rgb(1,0,1));
      }

      /* If guest is allowed, start with their drive input */
      if (guestOK) {
         driveData = driveDataGuest.clone();
      }

      /* If team is allowed, override with their drive input if not 0 */
      if (teamOK) {
         if (driveDataTeam.driveSpeed > 0) {
            driveData.driveSpeed = driveDataTeam.driveSpeed;
            driveData.driveAngle = driveDataTeam.driveAngle;
         }
         if (driveDataTeam.rotate != 0) {
            driveData.rotate = driveDataTeam.rotate;
         }
      }

      /* With the most dangerous things out of the way, we can move on to the other controls */

      if (eitherGuestOrTeam(Buttons.dpad_left, State.wasSingleTapped))
         DSShooter.disarmShooter();

      if (eitherGuestOrTeam(Buttons.dpad_right, State.wasSingleTapped))
         DSShooter.armShooter();

      if (eitherGuestOrTeam(Buttons.dpad_left, State.wasDoubleTapped))
         DSShooter.extendPusher();

      if (eitherGuestOrTeam(Buttons.dpad_right, State.wasDoubleTapped))
         DSShooter.retractPusher();

      if (eitherGuestOrTeam(Buttons.dpad_up, State.wasSingleTapped))
         DSShooter.openGate();

      if (eitherGuestOrTeam(Buttons.dpad_down, State.wasSingleTapped))
         DSShooter.closeGate();

      if (eitherGuestOrTeam(Buttons.left_bumper, State.wasSingleTapped))
         parts.dsShooter.intakeReverse();

      if (eitherGuestOrTeam(Buttons.right_bumper, State.wasSingleTapped))
         parts.dsShooter.intakeOn();

      if (eitherGuestOrTeam(Buttons.back, State.isHeld))
         parts.dsShooter.cancelStateMachines();

      if (eitherGuestOrTeam(Buttons.a, State.wasTapped))
         parts.dsShooter.startPush();

      if (eitherGuestOrTeam(Buttons.b, State.wasSingleTapped))
         parts.dsShooter.startShoot1();

      if (eitherGuestOrTeam(Buttons.x, State.wasSingleTapped))
         parts.dsShooter.startShoot3();

      if (eitherGuestOrTeam(Buttons.y, State.wasSingleTapped))
         parts.dsShooter.startFullAuto();

      if (eitherGuestOrTeam(Buttons.dpad_up, State.wasHeld))
         // todo: reference this as a variable from somewhere
         parts.autoDrive.setNavTarget(new NavigationTarget(new Position(-20,0,0), parts.dsMisc.toleranceHigh));

      // Toggle FCD
      //todo: Field centric is broken right now; problem with the angle
      if (eitherGuestOrTeam(Buttons.start, State.wasDoubleTapped)) {
         parts.dsLed.displayMessage('F', parts.userDrive.toggleFieldCentricDrive());
      }

      // Toggle HeadingHold
      if (eitherGuestOrTeam(Buttons.back, State.wasDoubleTapped)) {
         parts.dsLed.displayMessage('H', parts.userDrive.toggleHeadingHold());
      }

      // Store heading correction
      if (eitherGuestOrTeam(Buttons.right_stick_button, State.wasReleased)) {
         parts.userDrive.setDeltaHeading();
         parts.dsLed.displayMessage('D', 1);
      }

      if (eitherGuestOrTeam(Buttons.left_stick_button, State.wasReleased))  {
         parts.dsLed.displayMessage('P', parts.userDrive.togglePositionHold());
      }
   }

   public boolean eitherGuestOrTeam(Buttons button, State state) {
      // if either was enabled and activated the control, return true
      boolean team = teamOK && buttonMgr.getState(1, button, state);
      boolean guest = guestOK && buttonMgr.getState(2, button, state);
      return team || guest;
   }

   public void stopEverything() {
      if (!isStopped) {
         // note: drivedata is already zeroed in the runloop
         parts.drivetrain.stopDriveMotors(true);
         parts.dsShooter.eStop();
         parts.autoDrive.cancelNavigation();
         isStopped = true;
      }
   }
}
