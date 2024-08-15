package org.firstinspires.ftc.teamcode.robot.Common.Tools.DataTypes;

public class PositionError {
    Position errorXYR;
    Vector errorXR;

    public PositionError() {
        errorXYR = new Position();
        errorXR = new Vector();
    }

    public PositionError(Position pos, Vector vec) {
        errorXYR = pos;
        errorXR = vec;
    }

    public PositionError(Vector vec, double heading) {
        errorXR = vec;
        errorXYR = new Position(vec.X * Math.cos(Math.toRadians(vec.A)),
                vec.X * Math.sin(Math.toRadians(vec.A)),
                heading);
    }

    public PositionError(Position pos) {
        errorXYR = pos;
        errorXR = new Vector(Math.sqrt(Math.pow(pos.X, 2) + Math.pow(pos.Y, 2)), Math.toDegrees(Math.atan2(pos.X, pos.Y)));
        //=DEGREES(ATAN2(A6,B6))
    }
}
