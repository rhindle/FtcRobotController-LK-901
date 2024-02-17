package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Tools.AdafruitNeoDriver;

public class NeoMatrix {

   HardwareMap hardwareMap;
   Telemetry telemetry;
   Robot robot;

   public AdafruitNeoDriver ledMatrix = null;

   private boolean updatePanel = true;
   private boolean pendingUpdate = false;
   private int updateLimit = 1;
   private int updatePosition = 0;
   private int dimMax = 255;   // for initial debugging, let's not changethe max

   final int ledRows = 8;
   final int ledCols = 32;
   final int ledQty = ledCols * ledRows;
   final boolean flipVert = false;
   final boolean flipHoriz = false;
   final int updateSize = 8;   // i2c driver limit is 24 bytes, or 8 pixels * 3 RGB bytes

   int[][] matrixBuffer = new int[ledCols][ledRows];
   //int[][] currentBuffer = new int[ledCols][ledRows];
   //int[][] dimmedBuffer = new int[ledCols][ledRows];
   int[]   stringBuffer  = new int[ledQty];
   int[]   stringActual  = new int[ledQty];
   int[]   stringUpdate  = new int[updateSize];

   /* Constructor */
   public NeoMatrix(Robot robot){
      construct(robot);
   }

   void construct(Robot robot){
      this.robot = robot;
      this.telemetry = robot.telemetry;
      this.hardwareMap = robot.hardwareMap;
   }

   public void init() {
      ledMatrix = hardwareMap.get(AdafruitNeoDriver.class, "neo");
      ledMatrix.setNumberOfPixels(ledQty);
   }

   public void loop(){
      if (!updatePanel) return;
      convertMatrix();
      updateMatrix();
   }

   void convertMatrix() {
      // The purpose of this function is to convert the rows and columns of the buffer into the topology of the matrix.
      // Here, I am assuming a short zig zag layout.
      // Also, apply dimming.
      boolean flipFlop = flipVert;
      int zigCount = 0;
      int stringPos;
      for (int c = 0; c < ledCols; c++) {
         for (int r = 0; r < ledRows; r++) {
            stringPos = c * ledRows + r;
            if (flipHoriz) stringPos = ledQty-1 - stringPos;
            stringBuffer[stringPos] = !flipFlop ? dimColor(matrixBuffer[c][r], dimMax) : dimColor(matrixBuffer[c][ledRows-1-r], dimMax);

            zigCount++;
            if (zigCount == ledCols) {
               zigCount = 0;
               flipFlop = !flipFlop;
            }
         }
      }
   }

//   void dimBuffer(){
//
//      for (int c = 0; c < ledCols; c++) {
//         for (int r = 0; r < ledRows; r++) {
//            int intColor = matrixBuffer[c][r];
//            int red = (int)(Color.red(intColor) * dimMax/255.0);
//            int green = (int)(Color.green(intColor) * dimMax/255.0);
//            int blue = (int)(Color.blue(intColor) * dimMax/255.0);
//            dimmedBuffer[c][r] = Color.rgb(red,green,blue);
//         }
//      }
//
//
//   }

   private int dimColor(int color0, int dmax) {
      // Simple linear dimming. Perhaps should upgrade to perceived dimming?
      return Color.rgb( (int)(Color.red(color0) * dmax/255.0),
                        (int)(Color.green(color0) * dmax/255.0),
                        (int)(Color.blue(color0) * dmax/255.0) );
   }

   void updateBlock(int start) {
      // first make sure we won't be longer than the string
      if (start + updateSize - 1 >= ledQty) start = ledQty - updateSize;
      // build the update array and update the stored "actual"
      for (int i=0; i<updateSize; i++) {
         stringActual[start+i] = stringBuffer[start+i];
         stringUpdate[i] = stringBuffer[start+i];
      }
      // send the update
      ledMatrix.transmitPixelUpdateA(stringUpdate, start);
      pendingUpdate = true;
   }

   void showMatrix() {
      ledMatrix.show();
      pendingUpdate = false;
   }

   void updateMatrixNoLimit() {
      // Since there's no limit imposed, update the entire matrix.
      // Nevertheless, optimize by only updating what has changed.
      for (int x=0; x < ledQty; x++) {
         if (stringBuffer[x] != stringActual[x]) {
            updateBlock(x);
            x += updateSize; // no need to check the next 8 pixels
         }
      }
      updatePosition=0;
      showMatrix();
   }

   void updateMatrix() {
      // we're allowed updateLimit transactions. 0 means no limit!
      if (updateLimit == 0) {
         updateMatrixNoLimit();
         return;
      }
      int updateCount = 0;
      boolean looped = false;
      // reminder to myself:  "break" exits the loop, "continue" iterates
      while (updateCount < updateLimit) {
         if (updatePosition >= ledQty) {
            updatePosition = 0;
            if (pendingUpdate) {
               showMatrix();
               updateCount++;
            }
            looped = true;
            continue;
         }
         if (stringBuffer[updatePosition] != stringActual[updatePosition]) {
            updateBlock(updatePosition);
            updateCount++;
            updatePosition += updateSize;
         }
         else updatePosition++;
         if (updatePosition >= ledQty && !pendingUpdate && looped) break; // full loop around without update so stop
      }
   }

   public void clearMatrix() {
      // This will clear the buffer, and then the normal update process will write it to the LEDs
      for (int c = 0; c < ledCols; c++) {
         for (int r = 0; r < ledRows; r++) {
            matrixBuffer[c][r]=0;
         }
      }
   }

   public void clearCols(int startC, int numC) {
      if (startC < 0 || startC >= ledCols) return;
      if (numC < 1) return;
      int endC = startC + numC - 1;
      if (endC >= ledCols) endC = ledCols - 1;
      for (int c = startC; c <= endC; c++) {
         for (int r = 0; r < ledRows; r++) {
            matrixBuffer[c][r]=0;
         }
      }
   }

   public void clearRows(int startR, int numR) {
      if (startR < 0 || startR >= ledRows) return;
      if (numR < 1) return;
      int endR = startR + numR - 1;
      if (endR >= ledRows) endR = ledRows - 1;
      for (int c = 0; c < ledCols; c++) {
         for (int r = startR; r <= endR; r++) {
            matrixBuffer[c][r]=0;
         }
      }
   }

   public void forceClearMatrix() {
      // This will make for a long loop time
      ledMatrix.fill(1280);
      showMatrix();
   }

   public void setUpdateLimit(int limit) {
      if (limit >= 0 && limit <= ledCols) {
         updateLimit = limit;
      }
   }

   public void setUpdatePanel(boolean boo) {
       updatePanel = boo;
   }
}
