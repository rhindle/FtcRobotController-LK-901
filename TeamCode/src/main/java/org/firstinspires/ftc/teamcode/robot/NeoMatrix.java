package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.robot.Tools.AdafruitNeoDriver;
import org.opencv.core.Mat;

import java.util.Arrays;

public class NeoMatrix {

   HardwareMap hardwareMap;
   Telemetry telemetry;
   Robot robot;

   public AdafruitNeoDriver ledMatrix = null;

   private boolean updatePanel = true;
   private boolean pendingUpdate = false;
   private boolean preventTearing = false;
//   private boolean clearToUpdate = true;

   private int updateLimit = 1;
   private int updatePosition = 0;
   private int dimMax = 255;     // for initial debugging, let's not change the max

   final int ledRows = 8;
   final int ledCols = 32; //32
   final int ledQty = ledCols * ledRows;
   final int hwLedQty = Math.min(1536, ledQty); // the board doesn't work at more than 255 pixels? A problem with documentation?
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
//      ledMatrix.setNumberOfPixels(ledQty);
      ledMatrix.setNumberOfPixels(hwLedQty);
      ledMatrix.fill(0);
      ledMatrix.show();
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
      if (preventTearing && pendingUpdate) return;
      boolean flipFlop = flipVert;
//      int zigCount = 0;
      int stringPos;
      for (int c = 0; c < ledCols; c++) {
         for (int r = 0; r < ledRows; r++) {
            stringPos = c * ledRows + r;
            if (flipHoriz) stringPos = ledQty-1 - stringPos;
            stringBuffer[stringPos] = !flipFlop ? dimColor(matrixBuffer[c][r], dimMax) : dimColor(matrixBuffer[c][ledRows-1-r], dimMax);

//            zigCount++;
//            if (zigCount == ledCols) {
//               zigCount = 0;
//               flipFlop = !flipFlop;
//            }
         }
         flipFlop = !flipFlop;
      }
   }

   private int dimColor(int color0, int dmax) {
      //return color0;  //temp disable during debug
      // Simple linear dimming. Perhaps should upgrade to perceived dimming?
      return Color.rgb( (int)(0.5 + Color.red(color0) * dmax/255.0),
                        (int)(0.5 + Color.green(color0) * dmax/255.0),
                        (int)(0.5 + Color.blue(color0) * dmax/255.0) );
   }

   void updateBlock(int start) {
      // first make sure we won't be longer than the string
      if (start + updateSize - 1 >= ledQty) start = ledQty - updateSize;
//^^temp      if (start + updateSize - 1 >= hwLedQty) start = hwLedQty - updateSize;
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

   public void forceUpdateMatrix() {
      pendingUpdate = false;
      convertMatrix();
      updateMatrixNoLimit();
   }

   void updateMatrixNoLimit() {
      // Since there's no limit imposed, update the entire matrix.
      // Nevertheless, optimize by only updating what has changed.
      for (int x=0; x < ledQty; x++) {
         if (stringBuffer[x] != stringActual[x]) {
            updateBlock(x);
            x += updateSize - 1; // no need to check the next 8 pixels
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
      return (int)(0.5 + Y1 + (1.0* (XX - X1) / (X2 - X1)) * (Y2 - Y1));
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

   public void setPreventTearing(boolean boo) {
      preventTearing = boo;
   }

   public void setDimmingValue(int dmax) {
      if (dmax < 0 || dmax > 255) return;
      dimMax = dmax;
   }

   public void setUpdatePanel(boolean boo) {
       updatePanel = boo;
   }

   public void scrollRegion (int startColumn, int endColumn, int startRow, int endRow, int xDir, int yDir, boolean rotate) {
      if (startColumn < 0 || startColumn >= ledCols) return;
      if (endColumn < startColumn || endColumn >= ledCols) return;
      if (startRow < 0 || startRow >= ledRows) return;
      if (endRow < startRow || endRow >= ledRows) return;
      if (endColumn==startColumn) xDir = 0;
      if (endRow==startRow) yDir = 0;
      boolean right = (xDir > 0);
      boolean up = (yDir < 0);
      // Do the x (columns) first
//      if (xDir != 0) {
      for (int i = 0; i < Math.abs(xDir); i++) {
         for (int y = startRow; y <= endRow; y++) {
            int save = right ? matrixBuffer[endColumn][y] : matrixBuffer[startColumn][y];
            for (int x = startColumn; x < endColumn; x++) {
//               if (right) matrixBuffer[x+1][y] = matrixBuffer[x][y];   //wrong?
//               if (right) pMap[width-x] = pMap[width-x-1];
               if (right) matrixBuffer[endColumn-x][y] = matrixBuffer[endColumn-x-1][y];
               else matrixBuffer[x][y] = matrixBuffer[x+1][y];
            }
            if (right) matrixBuffer[startColumn][y] = rotate ? save : 0;
            else matrixBuffer[endColumn][y] = rotate ? save : 0;
         }
      }
      // Then do the y (rows)
//      if (yDir != 0) {
      for (int i = 0; i < Math.abs(yDir); i++) {
         for (int x = startColumn; x <= endColumn; x++) {
            int save = !up ? matrixBuffer[x][endRow] : matrixBuffer[x][startRow];
            for (int y = startRow; y < endRow; y++) {
//               if (!up) matrixBuffer[x][y+1] = matrixBuffer[x][y];    //wrong?
//               if (!up) pMap[x][height-y] = pMap[x][height-y-1];
               if (!up) matrixBuffer[x][endRow-y] = matrixBuffer[x][endRow-y-1];
               else matrixBuffer[x][y] = matrixBuffer[x][y+1];
            }
            if (!up) matrixBuffer[x][startRow] = rotate ? save : 0;
            else matrixBuffer[x][endRow] = rotate ? save : 0;
         }
      }
   }

   public void applyPixelMapToBuffer (int[][] pMap, int colStart, int colEnd, int mapStart, boolean opaque) {
      // assuming that the height of the pixel map is the same as the panel because lazy
      if (colStart < 0) colStart = 0;
      if (colEnd == 0) colEnd = ledCols;
      if (colEnd-colStart+1 > pMap.length) colEnd = colStart+pMap.length-1;
      int pMapX = mapStart;
      for (int c = colStart; c <= colEnd; c++) {
         for (int r = 0; r < ledRows; r++) {
            int px = pMap[pMapX][r];
            if (opaque || px != 0) matrixBuffer[c][r] = px;
         }
         pMapX++;
      }
   }

   public int[][] buildPixelMapFromString (String text, char[][] charSet, int foreColor, int backColor ) {
      char[] charArray = text.toCharArray();
      int[][] pMap = new int[0][];
      for (char c : charArray) {
         pMap = appendPixelMap(pMap, convertBitMap(getBitMap(charSet, c), foreColor, backColor));
      }
      return pMap;
   }

   public int[][] appendPixelMap (int[][] pixMap1, int[][] pixMap2) {
      int[][] pMap1 = cloneArray(pixMap1);
      int[][] pMap2 = cloneArray(pixMap2);
      if (pMap1 == null && pMap2 == null) return null;
      if (pMap1 == null || pMap1.length == 0) return pMap2;
      if (pMap2 == null || pMap2.length == 0) return pMap1;
      // assume the 2nd dimension (rows) is the same across (should be 8)
      int[][] newMap = new int[pMap1.length + pMap2.length][pMap1[0].length];
      // Note: the following copies references rather than making a new array, which is why I cloned the arrays first
      System.arraycopy(pMap1, 0, newMap, 0, pMap1.length);
      System.arraycopy(pMap2, 0, newMap, pMap1.length, pMap2.length);
      return newMap;
   }

//   public int[][] cloneArray(int[][] src) {
//      int length = src.length;
//      int[][] target = new int[length][src[0].length];
//      for (int i = 0; i < length; i++) {
//         System.arraycopy(src[i], 0, target[i], 0, src[i].length);
//      }
//      return target;
//   }

   public int[][] cloneArray(final int[][] array) {
      // This works for 2D int arrays, which is what we have.
      if (array != null) {
         final int[][] copy = new int[array.length][];
         for (int i = 0; i < array.length; i++) {
            final int[] row = array[i];
            copy[i] = new int[row.length];
            System.arraycopy(row, 0, copy[i], 0, row.length);
         }
         return copy;
      }
      return null;
   }

   public int[][] shiftPixelMap (int[][] pixMap, int xDir, int yDir, boolean rotate) {
      // Should add checking for magnitude of xDir and yDir
      int[][] pMap = cloneArray(pixMap);
      int width = pMap.length - 1;
      int height = pMap[0].length - 1;
      boolean right = (xDir > 0);
      boolean up = (yDir < 0);
      //if (xDir != 0) {
         for (int i = 0; i < Math.abs(xDir); i++) {
            int[] save = right ? pMap[width] : pMap[0];
            for (int x = 0; x < width; x++) {
//            if (right) pMap[x + 1] = pMap[x];   // wrong
               if (right) pMap[width-x] = pMap[width-x-1];
               else pMap[x] = pMap[x + 1];
            }
            if (right) pMap[0] = rotate ? save : new int[save.length];
            else pMap[width] = rotate ? save : new int[save.length];
         }
      //}
      //if (yDir != 0) {
         for (int i = 0; i < Math.abs(yDir); i++) {
            for (int x = 0; x < pMap.length; x++) {
               int save = !up ? pMap[x][height] : pMap[x][0];
               for (int y = 0; y < height; y++) {
//               if (!up) pMap[x][y + 1] = pMap[x][y];   // wrong
                  if (!up) pMap[x][height-y] = pMap[x][height-y-1];
                  else pMap[x][y] = pMap[x][y + 1];
               }
               if (!up) pMap[x][0] = rotate ? save : 0;
               else pMap[x][height] = rotate ? save : 0;
            }
         }
      //}
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
            // foreground color or background color.
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

   public final char[][] blockNumerals = {
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

   public final char[][] specialChar = {
                              {' ', 0, 0, 0},
                              {'a', 6, 15, 15, 6},
                              {'b', 96, 240, 240, 96},
                              {'c', 0, 14, 31, 31, 14},
                              {'d', 32, 112, 32, 0, 0},
                              {'e', 192, 240, 236, 211, 169, 149, 147, 137, 253, 5, 3, 1},
                              {'f', 64, 96, 80, 79, 77, 85, 85, 85, 117, 21, 23, 24, 16},
                              {'g', 128, 192, 160, 159, 153, 169, 169, 165, 229, 37, 39, 56} };

   public final char[][] bigLetters =  {
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

   public final char[][] littleLetters = {
                              {' ', 0, 0, 0},
                              {'0', 31, 17, 31, 0},
//                              {'1', 0, 31, 0, 0},
                              {'1', 31, 0},
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

   public final char[][] littleLettersSq = {
                              {' ', 0, 0, 0},
                              {'0', 31, 17, 31, 0},
                              {'1', 18, 31, 16, 0},
                              {'2', 29, 21, 23, 0},
                              {'3', 21, 21, 31, 0},
                              {'4', 7, 4, 31, 0},
                              {'5', 23, 21, 29, 0},
                              {'6', 31, 21, 29, 0},
                              {'7', 1, 1, 31, 0},
                              {'8', 31, 21, 31, 0},
                              {'9', 23, 21, 31, 0},
                              {'A', 31, 5, 31, 0},
                              {'B', 31, 21, 27, 0},
                              {'C', 31, 17, 17, 0},
                              {'D', 31, 17, 14, 0},
                              {'E', 31, 21, 17, 0},
                              {'F', 31, 5, 1, 0},
                              {'G', 31, 17, 29, 0},
                              {'H', 31, 4, 31, 0},
                              {'I', 17, 31, 17, 0},
                              {'J', 17, 17, 15, 0},
                              {'K', 31, 4, 27, 0},
                              {'L', 31, 16, 16, 0},
                              {'M', 31, 1, 31, 1, 30, 0},
                              {'N', 31, 2, 4, 31, 0},
                              {'O', 31, 17, 31, 0},
                              {'P', 31, 5, 7, 0},
                              {'Q', 31, 25, 31, 0},
                              {'R', 31, 5, 27, 0},
                              {'S', 23, 21, 29, 0},
                              {'T', 1, 31, 1, 0},
                              {'U', 31, 16, 31, 0},
                              {'V', 15, 16, 15, 0},
                              {'W', 31, 16, 31, 16, 15, 0},
                              {'X', 27, 4, 27, 0},
                              {'Y', 7, 28, 7, 0},
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

   public final char[][] littleNumbers = {
                              {' ', 0, 0, 0, 0},
                              {'0', 31, 17, 31, 0},
                              {'1', 0, 31, 0, 0},
                              {'2', 29, 21, 23, 0},
                              {'3', 21, 21, 31, 0},
                              {'4', 7, 4, 31, 0},
                              {'5', 23, 21, 29, 0},
                              {'6', 31, 21, 29, 0},
                              {'7', 1, 1, 31, 0},
                              {'8', 31, 21, 31, 0},
                              {'9', 23, 21, 31, 0} };

   public final char[][] bigNumbers7H = {
                              {' ', 0, 0, 0, 0, 0, 0},
                              {'0', 62, 81, 73, 69, 62, 0},
                              {'1', 0, 66, 127, 64, 0, 0},
                              {'2', 114, 73, 73, 73, 70, 0},
                              {'3', 34, 65, 73, 73, 54, 0},
                              {'4', 14, 8, 8, 8, 127, 0},
                              {'5', 79, 73, 73, 73, 49, 0},
                              {'6', 62, 73, 73, 73, 50, 0},
                              {'7', 1, 1, 113, 9, 7, 0},
                              {'8', 54, 73, 73, 73, 54, 0},
                              {'9', 38, 73, 73, 73, 62, 0} };

   public final char[][] bigNumbers7HT = {
                              {' ', 0, 0, 0, 0, 0, 0},
                              {'0', 62, 127, 65, 127, 62, 0},
                              {'1', 0, 68, 126, 127, 64, 0},
                              {'2', 114, 123, 73, 79, 70, 0},
                              {'3', 34, 99, 73, 127, 54, 0},
                              {'4', 15, 15, 8, 127, 127, 0},
                              {'5', 79, 79, 73, 121, 49, 0},
                              {'6', 62, 127, 73, 123, 50, 0},
                              {'7', 1, 113, 121, 15, 7, 0},
                              {'8', 54, 127, 73, 127, 54, 0},
                              {'9', 38, 111, 73, 127, 62, 0} };

   public final char[][] bigNumbers8H = {
                              {' ', 0, 0, 0, 0, 0, 0},
                              {'0', 126, 129, 153, 129, 126, 0},
                              {'1', 0, 130, 255, 128, 0, 0},
                              {'2', 194, 161, 145, 137, 134, 0},
                              {'3', 66, 129, 137, 137, 118, 0},
                              {'4', 30, 16, 16, 16, 255, 0},
                              {'5', 143, 137, 137, 137, 113, 0},
                              {'6', 126, 137, 137, 137, 114, 0},
                              {'7', 1, 193, 49, 13, 3, 0},
                              {'8', 118, 137, 137, 137, 118, 0},
                              {'9', 78, 145, 145, 145, 126, 0} };

   public final char[][] faces = {
                              {'1', 16, 38, 66, 64, 70, 34, 16},
                              {'2', 16, 34, 70, 64, 66, 38, 16},
                              {'3', 16, 36, 70, 64, 68, 38, 16},
                              {'4', 16, 38, 68, 64, 70, 36, 16},
                              {'5', 32, 70, 130, 144, 134, 66, 32},
                              {'6', 32, 66, 134, 144, 130, 70, 32},
                              {'7', 32, 68, 134, 144, 132, 70, 32},
                              {'8', 32, 70, 132, 144, 134, 68, 32},
                              {'a', 60, 66, 149, 161, 161, 149, 66, 60},
                              {'b', 60, 66, 165, 145, 145, 165, 66, 60},
                              {'c', 60, 66, 129, 129, 129, 129, 66, 60},
                              {'d', 0, 0, 20, 32, 32, 20, 0, 0},
                              {'e', 0, 0, 36, 16, 16, 36, 0, 0},
                              {'A', 96, 134, 134, 128, 134, 134, 96},
                              {'B', 32, 70, 134, 128, 134, 70, 32} };
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


//   public int[][] shiftPixelMapVert (int[][] pMap, boolean rotate, boolean up) {
//      int height = pMap[0].length - 1;  //should be 8 always, minis 1 for the index
//      for (int x=0; x < pMap.length; x++) {
//         int save = !up ? pMap[x][height] : pMap[x][0];
//         for (int y=0; y < height; y++) {
//            if (!up) pMap[x][y+1] = pMap[x][y]; else pMap[x][y] = pMap[x][y+1];
//         }
//         if (!up) {
//            if (rotate) pMap[x][0] = save; else pMap[x][0] = 0;
//         }
//         else {
//            if (rotate) pMap[x][height] = save; else pMap[x][height] = 0;
//         }
//      }
//      return pMap;
//   }

//   public int[][] shiftPixelMapDown (int[][] pMap, boolean rotate) {
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
//
//   public int[][] shiftPixelMapUp (int[][] pMap, boolean rotate) {
//      int height = pMap[0].length - 1;  //should be 8 always, minis 1 for the index
//      for (int x=0; x < pMap.length; x++) {
//         int save = pMap[x][0];
//         for (int y=0; y < height; y++) {
//            pMap[x][y] = pMap[x][y+1];
//         }
//         if (rotate) pMap[x][height] = save; else pMap[x][height] = 0;
//      }
//      return pMap;
//   }

//   public int[][] shiftPixelMapHoriz (int[][] pMap, boolean rotate, boolean right) {
//      int width = pMap.length - 1;
//      int[] save = right ? pMap[width] : pMap[0];
//      for (int x=0; x < width; x++) {
//         if (right) pMap[x+1]=pMap[x]; else pMap[x]=pMap[x+1];
//      }
//      if (right) {
//         if (rotate) pMap[0] = save; else pMap[0] = new int[save.length];
//      }
//      else {
//         if (rotate) pMap[width] = save; else pMap[width] = new int[save.length];
//      }
//      return pMap;
//   }

//   public int[][] shiftPixelMapLeft (int[][] pMap, boolean rotate) {
//      int width = pMap.length - 1;
//      int[] save = pMap[0];
//      for (int x=0; x < width; x++) {
//         pMap[x]=pMap[x+1];
//      }
//      if (rotate) pMap[width] = save; else pMap[width] = new int[save.length];
//      return pMap;
//   }
//
//   public int[][] shiftPixelMapRight (int[][] pMap, boolean rotate) {
//      int width = pMap.length - 1;
//      int[] save = pMap[width];
//      for (int x=0; x < width; x++) {
//         pMap[x+1]=pMap[x];
//      }
//      if (rotate) pMap[0] = save; else pMap[0] = new int[save.length];
//      return pMap;
//   }


//moved from AdafruitNeoDriver

/* LK Future work:

Maintain a list of the colors of each pixel at the NeoDriver (actual display)

//Maintain a list of changes requested (to Prioritize)?
//    -What if the list gets too long and never catches up?
//
//or Maintain a list of colors to be sent (pixel order not priority)?
//    -What if the changes keep happening early in the array so only that side gets updated regularly?
//    -Better idea might be to step through the list a little further every loop

Maintain a list of the colors of each pixel that we want to be displayed (local, future display)

Current concept:
0. Start at index 0

1. Look for the first LED needing a change
    a. Send over that LED plus the next x pixels (for a total of MAX_TX_BYTES bytes)
    b. Update the local list of the displayed LEDs
    b. Advance the counter by x pixels - exit to next loop

2. If reached the end, Execute show
    Assuming 320 pixels, and 10 per update, this would be 32 sends.  At 20ms per loop, that's a 640ms update.
    Another option would be to "show" periodically, every so many writes, for a higher refresh but with tearing

In this scenario, only one I2C transaction happens per loop.
However, if the robot is idle (no user input or sensitive state machines), you could update more than one transaction.

*/