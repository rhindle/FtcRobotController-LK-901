package org.firstinspires.ftc.teamcode.robot.DiscShooter;

import android.graphics.Color;

import org.firstinspires.ftc.teamcode.robot.Common.Parts;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;

public class DSLed implements PartsInterface {

   /* Public OpMode members. */
   public Parts parts;


   /* Constructor */
   public DSLed(Parts parts){
      construct(parts);
   }

   void construct(Parts parts){
      this.parts = parts;
   }

   public void initialize(){
      parts.neo.initialize();
      parts.neo.setUpdateLimit(0);
      parts.neo.setPreventTearing(true);
      parts.neo.setDimmingValue(192);
      parts.neo.drawRectangle(0, 7, 0, 7, Color.rgb(1, 1, 0));
      textMatrix = parts.neo.buildPixelMapFromString("abcd", marquis, Color.rgb(1,1,0), Color.rgb(0,0,0));
   }

   public void preInit() {
   }

   public void initLoop() {
      parts.neo.applyPixelMapToBuffer(textMatrix,0,7, 0, true);
      parts.neo.applyPixelMapToBuffer(parts.neo.reversePixelMap(textMatrix),8,15, 0, true);
      textMatrix = parts.neo.shiftPixelMap(textMatrix,-8,0,true);
      clearMessage();
      parts.neo.runLoop();
   }

   public void preRun() {
      parts.neo.clearMatrix();
      parts.neo.drawRectangle(0,7,0,7, Color.rgb(1,1,1));
      parts.neo.setUpdateLimit(1);
   }

   public void runLoop() {
      clearMessage();
      parts.neo.runLoop();
   }

   public void stop() {
   }

   long clearMessageTimer = 0;
   int messageDisplayTime = 1000;

   public void displayMessage (char msgChar, boolean boo) {
      if (boo) displayMessage(msgChar, 2);
      else displayMessage(msgChar, 3);
   }
   public void displayMessage (char msgChar, int color) {
      if (!parts.useNeoMatrix) return;
      clearMessageTimer = System.currentTimeMillis() + messageDisplayTime;
      parts.neo.clearCols(8,15);
      int msgColor;
      int[][] textMatrix;
      switch (color) {
         case 2:
            msgColor = Color.rgb(0,4,0);
            break;
         case 3:
            msgColor = Color.rgb(4,0,0);
            break;
         case 1:
         default:
            msgColor = Color.rgb(2,2,2);
      }
      textMatrix = parts.neo.buildPixelMapFromString(String.valueOf(msgChar), parts.neo.bigLetters, msgColor);
      parts.neo.applyPixelMapToBuffer(textMatrix,10,15,0, true);
   }

   public void clearMessage () {
      if (clearMessageTimer == 0) return;
      if (System.currentTimeMillis() >= clearMessageTimer) {
         clearMessageTimer = 0;
         parts.neo.clearCols(8,15);
      }
   }

   int[][] textMatrix;

   public final char[][] marquis = {
           {'a', 17, 128, 0, 0, 1, 128, 0, 34},
           {'b', 34, 0, 128, 1, 0, 0, 128, 17},
           {'c', 68, 0, 1, 128, 0, 0, 1, 136},
           {'d', 136, 1, 0, 0, 128, 1, 0, 68},
           {'e', 238, 1, 129, 129, 128, 1, 129, 221},
           {'f', 221, 129, 1, 128, 129, 129, 1, 238},
           {'g', 187, 129, 128, 1, 129, 129, 128, 119},
           {'h', 119, 128, 129, 129, 1, 128, 129, 187} };

}