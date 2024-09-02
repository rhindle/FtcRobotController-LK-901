package org.firstinspires.ftc.teamcode.RobotParts.DiscShooter;

import android.graphics.Color;

import org.firstinspires.ftc.teamcode.RobotParts.Common.Parts;
import org.firstinspires.ftc.teamcode.Tools.PartsInterface;

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
      normalMatrix = parts.neo.buildPixelMapFromString("abcd", marquis, Color.rgb(1,1,0), Color.rgb(0,0,0));
   }

   public void preInit() {
   }

   public void initLoop() {
      parts.neo.applyPixelMapToBuffer(normalMatrix,0,7, 0, true);
      parts.neo.applyPixelMapToBuffer(parts.neo.reversePixelMap(normalMatrix),8,15, 0, true);
      normalMatrix = parts.neo.shiftPixelMap(normalMatrix,-8,0,true);
      clearMessage();
      parts.neo.runLoop();
   }

   public void preRun() {
      parts.neo.clearMatrix();
      normalMatrix = parts.neo.newPixelMapSameSize(normalMatrix);
      updateGraphic('4', Color.rgb(1,1,1));
      parts.neo.applyPixelMapToBuffer(finalMatrix,0,15,0, true);
      parts.neo.forceUpdateMatrix();
//      parts.neo.drawRectangle(0,7,0,7, Color.rgb(1,1,1));
      parts.neo.setUpdateLimit(1);
   }

   public void runLoop() {
      clearMessage();
      parts.neo.applyPixelMapToBuffer(finalMatrix,0,15,0, true);
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
//      parts.neo.clearCols(8,15);
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
      messageMatrix = parts.neo.newPixelMapSameSize(messageMatrix);
      messageMatrix = parts.neo.overlayPixelMap(textMatrix, messageMatrix, 2);
      messageMatrix = parts.neo.overlayPixelMap(textMatrix, messageMatrix, 10);
      finalMatrix = parts.neo.cloneArray(messageMatrix);
//      parts.neo.applyPixelMapToBuffer(messageMatrix,10,15,0, true);
//      parts.neo.applyPixelMapToBuffer(messageMatrix,2,7,0, true);
   }

   public void updateGraphic (char msgChar, int color) {
      int[][] textMatrix;
      textMatrix = parts.neo.buildPixelMapFromString(String.valueOf(msgChar), circles, color);
      normalMatrix = parts.neo.overlayPixelMap(textMatrix, normalMatrix,0);
      normalMatrix = parts.neo.overlayPixelMap(textMatrix, normalMatrix,8);
   }

   public void clearMessage () {
      if (clearMessageTimer == 0) {
         finalMatrix = parts.neo.cloneArray(normalMatrix);
         return;
      }
      if (System.currentTimeMillis() >= clearMessageTimer) {
         clearMessageTimer = 0;
//         parts.neo.clearCols(8,15);
      }
   }

//   int[][] textMatrix;
   int[][] messageMatrix = new int[16][8];
   int[][] normalMatrix = new int[16][8];
   int[][] finalMatrix = new int[16][8];

   public final char[][] marquis = {
           {'a', 17, 128, 0, 0, 1, 128, 0, 34},
           {'b', 34, 0, 128, 1, 0, 0, 128, 17},
           {'c', 68, 0, 1, 128, 0, 0, 1, 136},
           {'d', 136, 1, 0, 0, 128, 1, 0, 68},
           {'e', 238, 1, 129, 129, 128, 1, 129, 221},
           {'f', 221, 129, 1, 128, 129, 129, 1, 238},
           {'g', 187, 129, 128, 1, 129, 129, 128, 119},
           {'h', 119, 128, 129, 129, 1, 128, 129, 187} };

   public final char[][] circles = {
           {'1', 0, 0, 0, 24, 24, 0, 0, 0},
           {'2', 0, 0, 60, 36, 36, 60, 0, 0},
           {'3', 0, 60, 66, 66, 66, 66, 60, 0},
           {'4', 60, 66, 129, 129, 129, 129, 66, 60} };
}