package org.firstinspires.ftc.teamcode.robot.Common;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.spartronics4915.lib.T265Camera;
import com.spartronics4915.lib.T265Helper;

import org.firstinspires.ftc.teamcode.robot.Common.Tools.PartsInterface;
import org.firstinspires.ftc.teamcode.robot.Common.Tools.Position;

public class Slamra implements PartsInterface {

	volatile T265Camera slamra;
	Parts parts;

	public Position slamraFieldStart = null;								// set when start pushed (? final ?)
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
	}

	public void initialize() {
		// Use raw slamra values only (functions in the library are broken)
		if (slamra == null) {
			slamra = T265Helper.getCamera(
					new T265Camera.OdometryInfo(new Pose2d(0,0,0),0.1),
					parts.opMode.hardwareMap.appContext);
		}
		if (!slamra.isStarted()) slamra.start();
	}

	public void preInit() {
	}

	public void initLoop () {
		setupFieldOffset();
		slamraFinalPose = getSlamraFinalPose();
		parts.slamraPosition = slamraFinalPose;//.clone();
		isSlamraChanging();
		addTeleOpTelemetry();
	}

	public void preRun() {
	}

	public void runLoop() {
		updateSlamraPosition();
		isSlamraChanging();
		addTeleOpTelemetry();
	}

	public void stop() {
		slamra.stop();
	}

	public boolean isSlamraDead(){return timesStuck > 4;}

	public boolean isSlamraChanging() {
		if(!slamraRawPose.isEqualTo(lastPos)) {
			timesStuck = 0;
			lastPos = slamraRawPose.clone();  // add .clone?????
			return true;
		} else {
			timesStuck ++;
			return false;
		}
	}

	public void setupFieldOffset(Position fieldPosition) {
		slamraFieldOffset = new Position (0, 0, 0);    // clear any existing offset
		updateSlamraPosition();
		slamraFieldOffset = getSlamraFieldOffset(slamraRobotPose, fieldPosition);
	}
	public void setupFieldOffset() {
		slamraFieldOffset = new Position (0, 0, 0);    // clear any existing offset
		updateSlamraPosition();
		slamraFieldOffset = getSlamraFieldOffset(slamraRobotPose, slamraFieldStart);
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
		double offsetR = fieldPose.R - robotPose.R;
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
		TelemetryMgr.Message(6, "s-fldof", slamraFieldOffset.toString(2));
		TelemetryMgr.Message(6, "s-raw__", slamraRawPose.toString(2));
		TelemetryMgr.Message(6, "s-robot", slamraRobotPose.toString(2));
		TelemetryMgr.Message(6, "s-final", slamraFinalPose.toString(2));
		TelemetryMgr.Message(2, "slamra stuck", timesStuck);
		TelemetryMgr.Message(6,"last pos", lastPos.toString(2));
	}
}