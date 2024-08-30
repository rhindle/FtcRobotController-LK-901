package org.firstinspires.ftc.teamcode.robot.DiscShooter;

import org.firstinspires.ftc.teamcode.robot.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Common.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.robot.Common.Controls;
import org.firstinspires.ftc.teamcode.robot.Common.Parts;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.NavigationTarget;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.Functions;

public class ControlsDS extends Controls {

   public ControlsDS(Parts parts) {
      super(parts);
   }

   @Override
   public void runLoop() {
      userInput();
      parts.userDrive.setUserDriveSettings(driveSpeed, driveAngle, rotate);
   }

   @Override
   public void userInput() {

      // TeleOp / normal drive

      // Get speed and direction from left stick
//      driveSpeed = JavaUtil.minOfList(JavaUtil.createListWith(1, Functions.mathHypotenuse(gamepad1.left_stick_x, gamepad1.left_stick_y)));
      driveSpeed = Functions.mathHypotenuse(gamepad1.left_stick_x, gamepad1.left_stick_y);
      driveAngle = Math.toDegrees(Math.atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y));  //gamepad.x = left/right = robot.y
      // Get rotation from right stick
//      rotate = Math.pow(gamepad1.right_stick_x, 1);
      rotate = gamepad1.right_stick_x;
//      parts.navigator.handleRotate(rotate);

      // Toggle FCD
      //todo: Field centric is broken right now; problem with the angle
      if (buttonMgr.wasTapped(1, Buttons.start)) {
         parts.dsLed.displayMessage('F', parts.userDrive.toggleFieldCentricDrive());
      }

      // Toggle HeadingHold
      if (buttonMgr.wasTapped(1, Buttons.back)) {
         parts.dsLed.displayMessage('H', parts.userDrive.toggleHeadingHold());
      }

      // Store heading correction
      if (buttonMgr.wasReleased(1, Buttons.right_stick_button)) {
         parts.userDrive.setDeltaHeading();
         parts.dsLed.displayMessage('D', 1);
      }

      if (buttonMgr.wasReleased(1, Buttons.left_stick_button)) {
         parts.dsLed.displayMessage('P', parts.userDrive.togglePositionHold());
      }
//         navigator.toggleSnapToAngle();

      // This blob is for manually entering destinations by adjusting X, Y, Rot
      if (buttonMgr.wasTapped(1, Buttons.dpad_up))
           parts.autoDrive.setNavTarget(new NavigationTarget(new Position(-20,0,0), parts.dsMisc.toleranceHigh));
//         navigator.setTargetByDeltaRelative(2,0,0);
      if (buttonMgr.wasTapped(1, Buttons.dpad_down));
//         navigator.setTargetByDeltaRelative(-2,0,0);
      if (buttonMgr.wasTapped(1, Buttons.dpad_left));
//         navigator.setTargetByDeltaRelative(0, 2,0);
      if (buttonMgr.wasTapped(1, Buttons.dpad_right));
//         navigator.setTargetByDeltaRelative(0, -2,0);
      if (buttonMgr.wasTapped(1, Buttons.x)) {
//         navigator.setTargetRotBySnapRelative (45);
//         navigator.headingDelay = System.currentTimeMillis() + 500;  // workaround
      }
      if (buttonMgr.wasTapped(1, Buttons.y)) {
//         navigator.setTargetRotBySnapRelative (-45);
//         navigator.headingDelay = System.currentTimeMillis() + 500;  // workaround
      }

      if (!buttonMgr.isPressed(1, Buttons.right_bumper));
//         navigator.setMaxSpeed(1);
      else;
//         navigator.setMaxSpeed(0.25);

      // AutoDrive Testing

//      liftSpeed = -gamepad2.left_stick_y;

      if (buttonMgr.wasTapped(2, Buttons.dpad_left))
         parts.dsShooter.extendPusher();

      if (buttonMgr.wasTapped(2, Buttons.dpad_down))
         parts.dsShooter.closeGate();

      if (buttonMgr.wasTapped(2, Buttons.dpad_right))
         parts.dsShooter.retractPusher();

      if (buttonMgr.wasTapped(2, Buttons.dpad_up))
         parts.dsShooter.openGate();

      if (buttonMgr.wasTapped(2, Buttons.back)) {
         parts.dsShooter.cancelStateMachines();
      }

      if (buttonMgr.wasPressed(2, Buttons.start))
         parts.dsShooter.armShooter();
      if (buttonMgr.wasReleased(2, Buttons.start))
         parts.dsShooter.disarmShooter();

      if (buttonMgr.wasTapped(2, Buttons.a))
         parts.dsShooter.statePushStep = 1;

      if (buttonMgr.wasTapped(2, Buttons.b))
         parts.dsShooter.stateShoot1Step = 1;

      if (buttonMgr.wasTapped(2, Buttons.x))
         parts.dsShooter.stateShoot3Step = 1;

      if (buttonMgr.wasTapped(2, Buttons.y))
         parts.dsShooter.stateFullAuto = 1;

      if (buttonMgr.wasTapped(2, Buttons.left_bumper))
         parts.dsShooter.intakeReverse();

      if (buttonMgr.wasTapped(2, Buttons.right_bumper))
         parts.dsShooter.intakeOn();


//      if (buttonMgr.isHeld(2, Buttons.left_bumper) &&
//              buttonMgr.wasTapped(2, Buttons.right_bumper));
////         lifter.action(LiftbotLifter.LiftActions.AUTOMATE_HOME);
//
//      if (buttonMgr.wasTapped(2, Buttons.right_bumper)) {
//         if (buttonMgr.isHeld(2, Buttons.left_bumper)) {
////            lifter.action(LiftbotLifter.LiftActions.AUTOMATE_HOME);
//         } else {
//            // reserved for future use
//         }
//      }
//
//      if (buttonMgr.wasTapped(2, Buttons.left_bumper));
////         lifter.action(LiftbotLifter.LiftActions.LIFT_DOWNSTACK);


   }
}
