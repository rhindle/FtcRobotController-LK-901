package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Om.Vector3;
import org.firstinspires.ftc.teamcode.robot.Tools.AdafruitNeoDriver;

import java.util.Arrays;

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

   public void drawLine(int startX, int startY, int endX, int endY, int lineColor) {
      // To assure the line is unbroken, take the lazy approach of drawing it both x and y directions
      int xStep = (startX <= endX) ? 1 : -1;
      int yStep = (startY <= endY) ? 1 : -1;
      for (int x = startX; x != endX; x += xStep) {
         matrixBuffer[x][interpolate(startX,startY,endX,endY,x)] = lineColor;
      }
      for (int y = startY; y != endY; y += yStep) {
         matrixBuffer[interpolate(startY,startX,endY,endX,y)][y] = lineColor;
      }
      // the last point is missed because both loops stop before reaching the end
      matrixBuffer[endX][endY] = lineColor;
   }

   int interpolate (int X1, int Y1, int X2, int Y2, int XX) {
      return (int)(0.5 + Y1 + ((XX - X1) / (X2 - X1)) * (Y2 - Y1));
   }
   float interpolate (float X1, float Y1, float X2, float Y2, float XX) {
      //y = y1 + ((x - x1) / (x2 - x1)) * (y2 - y1)
      return Y1 + ((XX - X1) / (X2 - X1)) * (Y2 - Y1);
   }

   public void drawRectangle(int startColumn, int endColumn, int startRow, int endRow, int lineColor) {
      drawRectangle(startColumn, endColumn, startRow, endRow, lineColor, false,0);
   }
   public void drawRectangle(int startColumn, int endColumn, int startRow, int endRow, int lineColor, boolean fill, int fillColor) {
      // Part 1: The outer rectangle
      if (startColumn < 0 || startColumn >= ledCols) return;
      if (endColumn < startColumn || endColumn >= ledCols) return;
      if (startRow < 0 || startRow >= ledRows) return;
      if (endRow < startRow || endRow >= ledRows) return;
      // a. make the lines across
      for (int c = startColumn; c <= endColumn; c++) {
         matrixBuffer[c][startRow]=lineColor;
         matrixBuffer[c][endRow]=lineColor;
      }
      // b. make the lines down
      startRow++;
      endRow--;
      for (int r = startRow; r <= endRow; r++) {
         matrixBuffer[startColumn][r] = lineColor;
         matrixBuffer[endColumn][r] = lineColor;
      }
      if (!fill) return;
      // Part 2: The fill
      startColumn++;
      endColumn--;
      //if (startColumn >= ledCols || endColumn < startColumn) return;
      //if (startRow >= ledRows || endRow < startRow) return;
      for (int c = startColumn; c <= endColumn; c++) {
         for (int r = startRow; r <= endRow; r++) {
            matrixBuffer[c][r] = fillColor;
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

   public void scrollRegion (int startColumn, int endColumn, int startRow, int endRow, int xDir, int yDir, boolean rotate) {

   }

//   public int[][] shiftPixelMapVert (int[][] pMap, boolean rotate, boolean up) {
//      int height = pMap[0].length - 1;  //should be 8 always, minis 1 for the index
//      for (int x=0; x < pMap.length; x++) {
//         int save = pMap[x][height];
//         for (int y=0; y < height; y++) {
//            pMap[x][y+1] = pMap[x][y];
//         }
//         if (rotate) pMap[x][0] = save; else pMap[x][0] = 0;
//      }
//      return pMap;
//   }

   public int[][] shiftPixelMapDown (int[][] pMap, boolean rotate) {
      int height = pMap[0].length - 1;  //should be 8 always, minis 1 for the index
      for (int x=0; x < pMap.length; x++) {
         int save = pMap[x][height];
         for (int y=0; y < height; y++) {
            pMap[x][y+1] = pMap[x][y];
         }
         if (rotate) pMap[x][0] = save; else pMap[x][0] = 0;
      }
      return pMap;
   }

   public int[][] shiftPixelMapUp (int[][] pMap, boolean rotate) {
      int height = pMap[0].length - 1;  //should be 8 always, minis 1 for the index
      for (int x=0; x < pMap.length; x++) {
         int save = pMap[x][0];
         for (int y=0; y < height; y++) {
            pMap[x][y] = pMap[x][y+1];
         }
         if (rotate) pMap[x][height] = save; else pMap[x][height] = 0;
      }
      return pMap;
   }

   public int[][] shiftPixelMapLeft (int[][] pMap, boolean rotate) {
      int width = pMap.length - 1;
      int[] save = pMap[0];
      for (int x=0; x < width; x++) {
         pMap[x]=pMap[x+1];
      }
      if (rotate) pMap[width] = save; else pMap[width] = new int[save.length];
      return pMap;
   }

   public int[][] shiftPixelMapRight (int[][] pMap, boolean rotate) {
      int width = pMap.length - 1;
      int[] save = pMap[width];
      for (int x=0; x < width; x++) {
         pMap[x+1]=pMap[x];
      }
      if (rotate) pMap[0] = save; else pMap[0] = new int[save.length];
      return pMap;
   }

   public int[][] convertBitMap (char[] bitMap, int foreColor, int backColor) {
      // this will create an array [x][y] with y being 8.
      // If this doesn't match the matrix height, will need to do additional manipulation somewhere.
      if (bitMap.length == 0) return null;
      int[][] pixelMap = new int[bitMap.length][8];
      for (int i=0; i < bitMap.length; i++) {
         for (int j=0; j<8; j++) {
            // bitMap is a binary representation of 8 pixels high, simply on or off.
            // Here, we bitwise shift the bitmap, bitwise AND it with 1 to clear the other bits,
            // and compare it with "1" to see if it's set to determine if the pixelMap should be
            // foreground color or backgroudn color.
            pixelMap[i][j] = ((bitMap[i] >> j) & 1) == 1 ? foreColor : backColor;
         }
      }
      return pixelMap;
   }

   public char[] getBitMap (char[][] charSet, char letter) {
      // note: all the bitmaps are based on a byte, therefore represent 8 pixels high.
      // This function simply returns the 8 bit bitmap for the character wanted from a "charSet" array of bitmaps
      for (int i=0; i < charSet.length; i++) {
         if (letter == charSet[i][0]) {
            return Arrays.copyOfRange(charSet[i], 1, charSet[i].length);
         }
      }
      return new char[] {0};
   }

   final char[][] blockNumerals = {
                              {'0', 127, 65, 65, 65, 127, 0},
                              {'1', 0, 0, 127, 0, 0, 0},
                              {'2', 121, 73, 73, 73, 79, 0},
                              {'3', 65, 73, 73, 73, 127, 0},
                              {'4', 15, 8, 8, 8, 127, 0},
                              {'5', 79, 73, 73, 73, 121, 0},
                              {'6', 127, 73, 73, 73, 121, 0},
                              {'7', 1, 1, 1, 1, 127, 0},
                              {'8', 127, 73, 73, 73, 127, 0},
                              {'9', 79, 73, 73, 73, 127,0} };

   final char[][] specialChar = {
                              {'a', 6, 15, 15, 6},
                              {'b', 96, 240, 240, 96},
                              {'c', 0, 14, 31, 31, 14},
                              {'d', 32, 112, 32, 0, 0},
                              {'e', 192, 240, 236, 211, 169, 149, 147, 137, 253, 5, 3, 1},
                              {'f', 64, 96, 80, 79, 77, 85, 85, 85, 117, 21, 23, 24},
                              {'g', 128, 192, 160, 159, 153, 169, 169, 165, 229, 37, 39, 56} };

   final char[][] bigLetters =  {
                              {'0', 62, 81, 73, 69, 62, 0},
                              {'1', 0, 66, 127, 64, 0, 0},
                              {'2', 114, 73, 73, 73, 70, 0},
                              {'3', 34, 65, 73, 73, 54, 0},
                              {'4', 14, 8, 8, 8, 127, 0},
                              {'5', 79, 73, 73, 73, 49, 0},
                              {'6', 62, 73, 73, 73, 50, 0},
                              {'7', 1, 1, 113, 9, 7, 0},
                              {'8', 54, 73, 73, 73, 54, 0},
                              {'9', 38, 73, 73, 73, 62, 0},
                              {'A', 126, 9, 9, 9, 126, 0},
                              {'B', 127, 73, 73, 73, 54, 0},
                              {'C', 62, 65, 65, 65, 65, 0},
                              {'D', 127, 65, 65, 65, 62, 0},
                              {'E', 127, 73, 73, 73, 65, 0},
                              {'F', 127, 9, 9, 9, 1, 0},
                              {'G', 62, 65, 65, 73, 121, 0},
                              {'H', 127, 8, 8, 8, 127, 0},
                              {'I', 0, 65, 127, 65, 0, 0},
                              {'J', 48, 64, 65, 65, 63, 0},
                              {'K', 127, 8, 20, 34, 65, 0},
                              {'L', 127, 64, 64, 64, 64, 0},
                              {'M', 127, 1, 127, 1, 126, 0},
                              {'N', 127, 4, 8, 16, 127, 0},
                              {'O', 62, 65, 65, 65, 62, 0},
                              {'P', 127, 9, 9, 9, 6, 0},
                              {'Q', 62, 65, 81, 33, 94, 0},
                              {'R', 127, 9, 25, 41, 70, 0},
                              {'S', 70, 73, 73, 73, 49, 0},
                              {'T', 1, 1, 127, 1, 1, 0},
                              {'U', 63, 64, 64, 64, 63, 0},
                              {'V', 7, 24, 96, 24, 7, 0},
                              {'W', 127, 64, 127, 64, 63, 0},
                              {'X', 99, 20, 8, 20, 99, 0},
                              {'Y', 3, 4, 120, 4, 3, 0},
                              {'Z', 97, 81, 73, 69, 67, 0},
                              {'*', 34, 20, 127, 20, 34, 0},
                              {'(', 0, 0, 62, 65, 0, 0},
                              {')', 0, 65, 62, 0, 0, 0},
                              {'#', 20, 127, 20, 127, 20, 0},
                              {'-', 0, 8, 8, 8, 0, 0},
                              {'+', 0, 8, 28, 8, 0, 0},
                              {'"', 0, 7, 0, 7, 0, 0},
                              {'\'', 0, 0, 7, 0, 0, 0},
                              {'!', 0, 0, 95, 0, 0, 0},
                              {'&', 48, 74, 85, 42, 80, 0},
                              {'=', 0, 20, 20, 20, 0, 0},
                              {'$', 4, 42, 127, 42, 16, 0},
                              {':', 0, 0, 20, 0, 0, 0},
                              {';', 0, 32, 20, 0, 0, 0},
                              {'>', 65, 34, 20, 8, 0, 0},
                              {'<', 8, 20, 34, 65, 0, 0},
                              {'?', 2, 1, 81, 9, 6, 0},
                              {'/', 96, 16, 8, 4, 3, 0},
                              {'%', 99, 19, 8, 100, 99, 0},
                              {'^', 4, 2, 1, 2, 4, 0},
                              {'@', 62, 65, 93, 81, 14, 0},
                              {'.', 0, 96, 96, 0, 0, 0},
                              {',', 0, 128, 96, 0, 0, 0},
                              {'~', 4, 2, 4, 4, 2, 0} };

   final char[][] littleLetters = {
                              {'0', 31, 17, 31, 0},
                              {'1', 0, 31, 0, 0},
                              {'2', 29, 21, 23, 0},
                              {'3', 21, 21, 31, 0},
                              {'4', 7, 4, 31, 0},
                              {'5', 23, 21, 29, 0},
                              {'6', 31, 21, 29, 0},
                              {'7', 1, 1, 31, 0},
                              {'8', 31, 21, 31, 0},
                              {'9', 23, 21, 31, 0},
                              {'A', 31, 5, 31, 0},
                              {'B', 31, 21, 10, 0},
                              {'C', 14, 17, 17, 0},
                              {'D', 31, 17, 14, 0},
                              {'E', 31, 21, 17, 0},
                              {'F', 31, 5, 1, 0},
                              {'G', 30, 17, 25, 0},
                              {'H', 31, 4, 31, 0},
                              {'I', 17, 31, 17, 0},
                              {'J', 17, 17, 15, 0},
                              {'K', 31, 4, 27, 0},
                              {'L', 31, 16, 16, 0},
                              {'M', 31, 31, 30, 0},
                              {'N', 31, 6, 31, 0},
                              {'O', 14, 17, 14, 0},
                              {'P', 31, 5, 2, 0},
                              {'Q', 14, 25, 30, 0},
                              {'R', 31, 5, 26, 0},
                              {'S', 18, 21, 9, 0},
                              {'T', 1, 31, 1, 0},
                              {'U', 31, 16, 31, 0},
                              {'V', 15, 24, 15, 0},
                              {'W', 31, 31, 15, 0},
                              {'X', 27, 4, 27, 0},
                              {'Y', 3, 28, 3, 0},
                              {'Z', 25, 21, 19, 0},
                              {'*', 21, 14, 21, 0},
                              {'(', 14, 17, 0, 0},
                              {')', 17, 14, 0, 0},
                              {'#', 10, 31, 10, 0},
                              {'-', 4, 4, 4, 0},
                              {'+', 4, 14, 4, 0},
                              {'"', 3, 0, 3, 0},
                              {'\'', 3, 0, 0, 0},
                              {'!', 23, 0, 0, 0},
                              {'&', 14, 31, 10, 0},
                              {'=', 10, 10, 10, 0},
                              {'$', 23, 31, 29, 0},
                              {':', 10, 0, 0, 0},
                              {';', 16, 10, 0, 0},
                              {'>', 17, 10, 4, 0},
                              {'<', 4, 10, 17, 0},
                              {'?', 1, 21, 3, 0},
                              {'/', 24, 4, 3, 0},
                              {'%', 25, 4, 19, 0},
                              {'^', 2, 0, 2, 0},
                              {'@', 15, 17, 23, 0},
                              {'.', 16, 0, 0, 0},
                              {',', 16, 8, 0, 0},
                              {'~', 2, 4, 2, 0} };
}


