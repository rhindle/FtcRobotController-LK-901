package org.firstinspires.ftc.teamcode.robot.DiscShooter;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.robot.Common.ButtonMgr;
import org.firstinspires.ftc.teamcode.robot.Common.Controls;
import org.firstinspires.ftc.teamcode.robot.Common.Parts;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.Functions;

public class ControlsDS extends Controls {

   public ControlsDS(Parts parts) {
      super(parts);
   }

   public int someNewVariable;

   @Override
   public void runLoop() {
      userInput();
      parts.navigator.setUserDriveSettings(DriveSpeed, DriveAngle, Rotate);
   }

   @Override
   public void userInput() {

      // TeleOp / normal drive

      // Get speed and direction from left stick
      DriveSpeed = JavaUtil.minOfList(JavaUtil.createListWith(1, Functions.mathHypotenuse(gamepad1.left_stick_x, gamepad1.left_stick_y)));
      DriveAngle = Math.atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y) / Math.PI * 180;
      // Get rotation from right stick
      Rotate = Math.pow(gamepad1.right_stick_x, 1);
      parts.navigator.handleRotate(Rotate);

      // Toggle FCD
      //todo: Field centric is broken right now; problem with the angle
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.start)) {
         parts.misc.displayMessage('F', parts.navigator.toggleFieldCentricDrive());
      }

      // Toggle HeadingHold
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.back)) {
         parts.misc.displayMessage('H', parts.navigator.toggleHeadingHold());
      }

      // Store heading correction
      if (buttonMgr.wasReleased(1, ButtonMgr.Buttons.right_stick_button)) {
         parts.navigator.setDeltaHeading();
         parts.misc.displayMessage('D', 1);
      }

      if (buttonMgr.wasReleased(1, ButtonMgr.Buttons.left_stick_button)) {
         parts.misc.displayMessage('P', parts.navigator.togglePositionHold());
      }
//         navigator.toggleSnapToAngle();

      // This blob is for manually entering destinations by adjusting X, Y, Rot
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpad_up));
//         navigator.setTargetByDeltaRelative(2,0,0);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpad_down));
//         navigator.setTargetByDeltaRelative(-2,0,0);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpad_left));
//         navigator.setTargetByDeltaRelative(0, 2,0);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.dpad_right));
//         navigator.setTargetByDeltaRelative(0, -2,0);
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.x)) {
//         navigator.setTargetRotBySnapRelative (45);
//         navigator.headingDelay = System.currentTimeMillis() + 500;  // workaround
      }
      if (buttonMgr.wasTapped(1, ButtonMgr.Buttons.y)) {
//         navigator.setTargetRotBySnapRelative (-45);
//         navigator.headingDelay = System.currentTimeMillis() + 500;  // workaround
      }

      if (!buttonMgr.isPressed(1, ButtonMgr.Buttons.right_bumper));
//         navigator.setMaxSpeed(1);
      else;
//         navigator.setMaxSpeed(0.25);

      // AutoDrive Testing

//      liftSpeed = -gamepad2.left_stick_y;

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpad_left))
         parts.shooter.extendPusher();

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpad_down))
         parts.shooter.closeGate();

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpad_right))
         parts.shooter.retractPusher();

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.dpad_up))
         parts.shooter.openGate();

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.back)) {
         parts.shooter.cancelStateMachines();
      }

      if (buttonMgr.wasPressed(2, ButtonMgr.Buttons.start))
         parts.shooter.armShooter();
      if (buttonMgr.wasReleased(2, ButtonMgr.Buttons.start))
         parts.shooter.disarmShooter();

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.a))
         parts.shooter.statePushStep = 1;

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.b))
         parts.shooter.stateShoot1Step = 1;

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.x))
         parts.shooter.stateShoot3Step = 1;

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.y))
         parts.shooter.stateFullAuto = 1;

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.left_bumper))
         parts.shooter.intakeReverse();

      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.right_bumper))
         parts.shooter.intakeOn();


//      if (buttonMgr.isHeld(2, ButtonMgr.Buttons.left_bumper) &&
//              buttonMgr.wasTapped(2, ButtonMgr.Buttons.right_bumper));
////         lifter.action(LiftbotLifter.LiftActions.AUTOMATE_HOME);
//
//      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.right_bumper)) {
//         if (buttonMgr.isHeld(2, ButtonMgr.Buttons.left_bumper)) {
////            lifter.action(LiftbotLifter.LiftActions.AUTOMATE_HOME);
//         } else {
//            // reserved for future use
//         }
//      }
//
//      if (buttonMgr.wasTapped(2, ButtonMgr.Buttons.left_bumper));
////         lifter.action(LiftbotLifter.LiftActions.LIFT_DOWNSTACK);


   }
}
