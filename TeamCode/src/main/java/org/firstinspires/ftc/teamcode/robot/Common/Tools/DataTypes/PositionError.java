package org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes;

public class PositionError {
    Position errorXYR;
    Vector2D errorDA;
    boolean xyPreferred;

    public PositionError() {
        errorXYR = new Position();
        errorDA = new Vector2D();
        xyPreferred = false;
    }

    public double errorR() {
        return errorXYR.R;
    }

    public PositionError(Position pos, Vector2D vec) {
        xyPreferred = true;
        errorXYR = pos.clone();
        errorXYR.abs();
        errorDA = vec.clone();
    }

    public PositionError(Vector2D vec, double heading) {
        xyPreferred = false;
        errorDA = vec.clone();
//        errorXYR = new Position(vec.distance * Math.cos(Math.toRadians(vec.angle)),
//                vec.distance * Math.sin(Math.toRadians(vec.angle)),
//                heading);
        errorXYR = new Position (vec.X(), vec.Y(), heading);
    }

    public PositionError(Position pos) {
        xyPreferred = true;
        errorXYR = pos.clone();
        //errorDA = new Vector2D(Math.sqrt(Math.pow(pos.X, 2) + Math.pow(pos.Y, 2)), Math.toDegrees(Math.atan2(pos.X, pos.Y))); // arguments backwards?
        errorDA = new Vector2D().fromXY(pos.X, pos.Y);
        //=DEGREES(ATAN2(A6,B6))
    }

    //todo: What was I thinking here? Is this a tolerance? Or an error? Why is everything based on a single position and/or vector?

    public PositionError(Position target, Position current) {
        errorXYR = target.clone();
        errorXYR.subtract(current);
        errorXYR.normalize();
        errorDA = new Vector2D().fromXY(errorXYR.X, errorXYR.Y);
    }

    public boolean inTolerance (Position target, Position current) {
        Position posError = target.clone();
        posError.subtract(current);
        posError.normalize();
        Vector2D posErrorV = new Vector2D().fromXY(posError.X, posError.Y);
//        posError.abs();
//        if (xyPreferred) {
//            //
//            double diffX = Math.abs(posError.X) - Math.abs(errorXYR.X);
//            double diffY = Math.abs(posError.Y) - Math.abs(errorXYR.Y);
//            double diffR = Math.abs(posError.R - Math.
//            if (diffX <= 0 && diffY <= 0 && diffR <= 0) {
//
//            }
//        }
        return false;
    }
}
