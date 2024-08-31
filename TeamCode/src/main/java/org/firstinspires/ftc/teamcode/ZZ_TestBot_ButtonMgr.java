package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParts.Common.ButtonMgr;

@TeleOp (name="ZZ_TestBot_ButtonMgr", group="Test")
//@Disabled
public class ZZ_TestBot_ButtonMgr extends LinearOpMode {

   ButtonMgr buttonMgr;
   int wasTappedCount;
   int wasHeldCount;
   int wasPressedCount;
   int wasReleasedCount;
   int wasSingleTappedCount;
   int wasDoubleTappedCount;

   @Override
   public void runOpMode() {
      buttonMgr = new ButtonMgr(this);

      resetCounters();

      while (!isStarted()) {
         telemetry.addData(">", "Opmode Ready.");
         telemetry.update();
         sleep(100);
      }

      while (opModeIsActive()) {

         buttonMgr.updateAll();

         ButtonMgr.cButton button1 = new ButtonMgr.cButton(1, ButtonMgr.Buttons.x);
         ButtonMgr.cButton button2 = new ButtonMgr.cButton(1, ButtonMgr.Buttons.y);
         ButtonMgr.cButton buttonReset = new ButtonMgr.cButton(1, ButtonMgr.Buttons.b);

         if(buttonMgr.wasPressed(buttonReset)) resetCounters();

         if(buttonMgr.wasTapped(button1)) {
            wasTappedCount++;
         }
         if(buttonMgr.wasSingleTapped(button1)) {
            wasSingleTappedCount++;
         }
         if(buttonMgr.wasDoubleTapped(button1)) {
            wasDoubleTappedCount++;
         }
         if(buttonMgr.wasPressed(button1)) {
            wasPressedCount++;
         }
         if(buttonMgr.wasReleased(button1)) {
            wasReleasedCount++;
         }
         if(buttonMgr.wasHeld(button1)) {
            wasHeldCount++;
         }

         telemetry.addLine("X is primary button for events below");
         telemetry.addLine("Y is secondary button (Pressed2)");
         telemetry.addLine("B resets the counts");
         telemetry.addLine();
         telemetry.addData("Pressed Count_",wasPressedCount);
         telemetry.addData("Released Count",wasReleasedCount);
         telemetry.addData("Tapped Count_",wasTappedCount);
         telemetry.addData("1x Tap Count_",wasSingleTappedCount);
         telemetry.addData("2x Tap Count_",wasDoubleTappedCount);

         telemetry.addData("Held Count___",wasHeldCount);

         telemetry.addData("Pressed______", buttonMgr.isPressed(button1));
         telemetry.addData("Held_________", buttonMgr.isHeld(button1));
         telemetry.addData("Tap-Held_____", buttonMgr.isSingleTapHeld(button1));

         telemetry.addData("Pressed2_____", buttonMgr.isPressed(button2));

         telemetry.update();
      }
   }

   void resetCounters() {
      wasTappedCount = 0;
      wasHeldCount = 0;
      wasPressedCount = 0;
      wasReleasedCount = 0;
      wasSingleTappedCount = 0;
      wasDoubleTappedCount = 0;
   }
}