package org.firstinspires.ftc.teamcode.robot.Common;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.spartronics4915.lib.T265Camera;
import com.spartronics4915.lib.T265Helper;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.Position;

public class Slamra  {

	volatile T265Camera slamra;
	Parts parts;
//	Telemetry telemetry;

	public Position slamraFieldStart = null;								// set when start pushed (? final ?)
//	public Position slamraRobotOffset = new Position(-6.5,0,-90);  // position transform to account for mounting position vs center of robot
//	public Position slamraRobotOffset = new Position(-8,-1,0);  // position transform to account for mounting position vs center of robot
    public Position slamraRobotOffset = new Position();                     // position transform to account for mounting position vs center of robot
	Position slamraRawPose = new Position();								// original position from slamra device
	Position slamraRobotPose = new Position();								// slamra transformed by robot position
	Position slamraFinalPose = new Position();                              // slamra transformed to field
	Position slamraFieldOffset = new Position();							// transform from initial position reported by slamra (may not be zero!)

	Position lastPos = new Position();
	int timesStuck = 0;

	public Slamra(Parts parts){
		construct(parts);
	}

	void construct(Parts parts){
		this.parts = parts;
//		this.telemetry = parts.opMode.telemetry;
	}

	public void init() {
		// We'll set up slamra without an initial position because that has historically been very broken
		if (slamra == null) {
			slamra = T265Helper.getCamera(
					new T265Camera.OdometryInfo(new Pose2d(0,0,0),0.1),
					parts.opMode.hardwareMap.appContext);
		}
		if (!slamra.isStarted()) slamra.start();
	}

	public void onStart() {
		//slamraFieldStart = parts.robotPosition;
		//setupFieldOffset(slamraFieldStart);
	}

	public void onStop() {
		slamra.stop();
	}

	public void initLoop () {
		setupFieldOffset();
		slamraFinalPose = getSlamraFinalPose();
		parts.slamraPosition = slamraFinalPose;//.clone();
		isSlamraChanging();
		addTeleOpTelemetry();
	}

	public void loop() {
		updateSlamraPosition();
		//telemetry.addData("slam final", slamraFinalPose.toString(2));
//		TelemetryHandler.Message(5,"slam final", slamraFinalPose.toString(2));
//		//telemetry.addData("last pos", lastPos.toString(2));
//		TelemetryHandler.Message(5,"last pos", lastPos.toString(2));
//		if(!slamraFinalPose.equals(lastPos)) {
//			parts.slamraPosition = slamraFinalPose.clone();
//			timesStuck = 0;
//			lastPos = slamraFinalPose;
//		} else {
//			timesStuck ++;
//		}

		isSlamraChanging();

//		//telemetry.addData("slamra stuck", timesStuck);
//		TelemetryHandler.Message(2, "slamra stuck", timesStuck);
		addTeleOpTelemetry();
	}

	public boolean isSlamraDead(){return timesStuck > 4;}

	public boolean isSlamraChanging() {
//		if(!slamraRawPose.equals(lastPos)) {
//		if(tempDiffTest(slamraRawPose, lastPos)) {
		if(!slamraRawPose.isEqualTo(lastPos)) {
//			parts.slamraPosition = slamraFinalPose.clone();
			timesStuck = 0;
			lastPos = slamraRawPose.clone();  // add .clone?????
			return true;
		} else {
			timesStuck ++;
			return false;
		}
	}

//	public boolean tempDiffTest (Position Pos1, Position Pos2) {
//		if (Pos1.X != Pos2.X) return true;
//		if (Pos1.Y != Pos2.Y) return true;
//		if (Pos1.R != Pos2.R) return true;
//		return false;
//	}

	public void setupFieldOffset(Position fieldPosition) {
		slamraFieldOffset = new Position (0, 0, 0);    // clear any existing offset
		updateSlamraPosition();
		slamraFieldOffset = getSlamraFieldOffset(slamraRobotPose, fieldPosition);
	}
	public void setupFieldOffset() {
		slamraFieldOffset = new Position (0, 0, 0);    // clear any existing offset
		updateSlamraPosition();
		slamraFieldOffset = getSlamraFieldOffset(slamraRobotPose, slamraFieldStart);
//		slamraFinalPose = getSlamraFinalPose();
	}

	public void updateSlamraPosition() {
		T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
		Pose2d update = up.pose;
		slamraRawPose = new Position(update.getX(), update.getY(), Math.toDegrees(update.getHeading()));
		slamraRobotPose = getSlamraRobotPose();
		slamraFinalPose = getSlamraFinalPose();
		parts.slamraPosition = slamraFinalPose;//.clone();
	}

	Position getSlamraRobotPose() {
		//pos1 = slamraRawPose, pos2 = slamraRobotOffset
		return transformPosition(slamraRawPose, slamraRobotOffset);
	}

	Position getSlamraFinalPose() {
		//pos1 = slamraFieldOffset, pos2 = slamraRobotPose
		Position slamraFinal;
		slamraFinal = transformPosition(slamraFieldOffset, slamraRobotPose);
		slamraFinal.normalize();
		return slamraFinal;
	}

	Position getSlamraFieldOffset(Position robotPose, Position fieldPose) {
//		Position sFS = slamraFieldStart;
//		Position sRP = slamraRobotPose;
		double offsetR = fieldPose.R - robotPose.R;
//		slamraFieldOffset = new Position (
		return new Position (
				fieldPose.X - (robotPose.X*Math.cos(Math.toRadians(offsetR)) - robotPose.Y*Math.sin(Math.toRadians(offsetR))),
				fieldPose.Y - (robotPose.X*Math.sin(Math.toRadians(offsetR)) + robotPose.Y*Math.cos(Math.toRadians(offsetR))),
				offsetR *1
		);
	}

	Position transformPosition(Position pos1, Position pos2) {
		return new Position(
				pos1.X + (pos2.X*Math.cos(Math.toRadians(pos1.R)) - pos2.Y*Math.sin(Math.toRadians(pos1.R))),
				pos1.Y + (pos2.X*Math.sin(Math.toRadians(pos1.R)) + pos2.Y*Math.cos(Math.toRadians(pos1.R))),
				pos1.R + pos2.R
		);
	}

	public void addTeleOpTelemetry() {
//		telemetry.addData("s-fldof", slamraFieldOffset.toString(2));
//		telemetry.addData("s-raw__", slamraRawPose.toString(2));
//		telemetry.addData("s-robot", slamraRobotPose.toString(2));
//		telemetry.addData("s-final", slamraFinalPose.toString(2));
		TelemetryHandler.Message(6, "s-fldof", slamraFieldOffset.toString(2));
		TelemetryHandler.Message(6, "s-raw__", slamraRawPose.toString(2));
		TelemetryHandler.Message(6, "s-robot", slamraRobotPose.toString(2));
		TelemetryHandler.Message(6, "s-final", slamraFinalPose.toString(2));
		TelemetryHandler.Message(2, "slamra stuck", timesStuck);
		TelemetryHandler.Message(6,"last pos", lastPos.toString(2));
	}
}