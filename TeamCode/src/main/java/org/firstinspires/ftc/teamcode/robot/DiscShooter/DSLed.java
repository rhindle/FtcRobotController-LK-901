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
   }

   public void preInit() {
   }

   public void initLoop() {
   }

   public void preRun() {
   }

   public void runLoop() {
   }

   public void stop() {
   }

   long clearMessageTimer = 0; //System.currentTimeMillis();
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

}