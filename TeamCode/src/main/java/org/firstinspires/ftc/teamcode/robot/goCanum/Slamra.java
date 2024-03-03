package org.firstinspires.ftc.teamcode.robot.goCanum;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.spartronics4915.lib.T265Camera;
import com.spartronics4915.lib.T265Helper;

//import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Universal.Tools.Position;

public class Slamra  {
	//variables
//	private static T265Camera slamra = null;
	volatile T265Camera slamra;
	Parts parts;
	Telemetry telemetry;

//	private final FtcDashboard dashboard = FtcDashboard.getInstance();
//	Transform2d cameraToRobot = new Transform2d(new Translation2d(-2 * 0.0254, 0.0 * 0.254), new Rotation2d());
//	double encoderMeasurementCovariance = 0.1; //0.8;

	public Position slamraFieldStart = null;								// set when start pushed (? final ?)
//	public Position slamraRobotOffset = new Position(-6.5,0,-90);  // position transform to account for mounting position vs center of robot
	public Position slamraRobotOffset = new Position(-8,-1,0);  // position transform to account for mounting position vs center of robot
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
		this.telemetry = parts.opMode.telemetry;
	}

	public void init() {
		// We'll set up slamra without an initial position because that has historically been very broken
		if (slamra == null) {
			slamra = T265Helper.getCamera(
					new T265Camera.OdometryInfo(new Pose2d(0,0,0),0.1),
					parts.opMode.hardwareMap.appContext);
//			slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, parts.opMode.hardwareMap.appContext);
//			slamra = new T265Camera(
//					new T265Camera.OdometryInfo(new Pose2d(0,0,0), 0.1),
//					"",
//					parts.opMode.hardwareMap.appContext);
		}
		if (!slamra.isStarted()) slamra.start();

//		if (slamra == null) {
//			//slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
//			slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, parts.opMode.hardwareMap.appContext);
//			//slamra.setPose(startingPose); // Useful if your robot doesn't start at the field-relative origin
//		}
//		if (!slamra.isStarted()) slamra.start();
	}

	public void onStart() {
//		slamraFieldStart = parent.getCurrentPosition();
		slamraFieldStart = parts.robotPosition;
		setupFieldOffset();
	}

//	@Override
//	public void onSettingsUpdate(SlamraSettings settings) {
//		if(slamra != null) throw new RuntimeException("slamra settings can not be updated after init");
//	}

	public boolean isSlamraDead(){
		return timesStuck > 4;
	}

	public void loop() {
		updateSlamraPosition();
		telemetry.addData("slam final", slamraFinalPose.toString(2));
		telemetry.addData("last pos", lastPos.toString(2));
		if(!slamraFinalPose.equals(lastPos)) {
//			parent.addPositionTicket(Slamra.class, new PositionTicket(slamraFinal));
			parts.slamraPosition = slamraFinalPose.clone();
			timesStuck = 0;
			lastPos = slamraFinalPose;
		}else{
			timesStuck ++;
		}

		telemetry.addData("slamra stuck", timesStuck);
		addTeleOpTelemetry();
	}

	public void updateSlamraPosition() {
		T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
//		if (up == null) return;
		Pose2d update = up.pose;
		slamraRawPose = new Position(update.getX(), update.getY(), Math.toDegrees(update.getHeading()));

		//LK
		updateSlamraRobotPose();
		setSlamraFinalPose();
	}

	public void setupFieldOffset() {
		updateSlamraPosition();
		setSlamraFieldOffset();
	}

		//LK ????????
//	void updateSlamraRobotPose() {
////		double sX, sY, sR, rX, rY, rR;
////		sX = slamraRawPose.X;
////		sY = slamraRawPose.Y;
////		sR = slamraRawPose.R;  // assuming was in radians tjk really degrees
////		//rX = robotOffset.getX();
////		//rY = robotOffset.getY();
////		//rR = robotOffset.getHeading();         // assuming was in degrees
////		rX = slamraRobotOffset.X;
////		rY = slamraRobotOffset.Y;
////		rR = slamraRobotOffset.R;
////		//x_robot*COS(RADIANS($C10))-y_robot*SIN(RADIANS($C10))
////		slamraRobotPose = slamraRobotPose.withX(sX + (rX*Math.cos(Math.toRadians(sR)) - rY*Math.sin(Math.toRadians(sR))));
////		//=x_robot*SIN(RADIANS($C10))+y_robot*COS(RADIANS($C10))
////		slamraRobotPose = slamraRobotPose.withY(sY + (rX*Math.sin(Math.toRadians(sR)) + rY*Math.cos(Math.toRadians(sR))));
////		slamraRobotPose = slamraRobotPose.withZ(sR + rR);
//
//		// Proposed slamra transformation code cleanup phase 1
//		Position pose1 = slamraRawPose;
//		Position pose2 = slamraRobotOffset;
//		slamraRobotPose = new Position(
//				pose1.X + (pose2.X*Math.cos(Math.toRadians(pose1.R)) - pose2.Y*Math.sin(Math.toRadians(pose1.R))),
//				pose1.Y + (pose2.X*Math.sin(Math.toRadians(pose1.R)) + pose2.Y*Math.cos(Math.toRadians(pose1.R))),
//				pose1.R + pose2.R
//		);
//	}
	void updateSlamraRobotPose() {
		//pos1 = slamraRawPose, pos2 = slamraRobotOffset
		slamraRobotPose = transformPosition(slamraRawPose, slamraRobotOffset);
	}

	// run this once at start after getting first robot pose
//	void setSlamraFieldOffset() {
////		double fX, fY, fR, rX, rY, rR, sR;
////		if (slamraFieldStart == null) {
////			fX = slamraRobotPose.X;
////			fY = slamraRobotPose.Y;
////			fR = slamraRobotPose.R;
////		} else {
////			fX = slamraFieldStart.X;
////			fY = slamraFieldStart.Y;
////			fR = slamraFieldStart.R;
////		}
////		rX = slamraRobotPose.X;
////		rY = slamraRobotPose.Y;
////		rR = slamraRobotPose.Z;
////		slamraFieldOffset = slamraFieldOffset.withZ(fR - rR);
////		sR = slamraFieldOffset.Z;
////		//=M4*COS(RADIANS(r_field_slam))-N4*SIN(RADIANS(r_field_slam))  m4=rX, n4=rY
////		slamraFieldOffset = slamraFieldOffset.withX(fX - (rX*Math.cos(Math.toRadians(sR)) - rY*Math.sin(Math.toRadians(sR))));
////		//=M4*SIN(RADIANS(r_field_slam))+N4*COS(RADIANS(r_field_slam))
////		slamraFieldOffset = slamraFieldOffset.withY(fY - (rX*Math.sin(Math.toRadians(sR)) + rY*Math.cos(Math.toRadians(sR))));
//
//		// Proposed slamra transformation code cleanup phase 1
//		Position sFS = slamraFieldStart;
//		Position sRP = slamraRobotPose;
//		double offsetR = sFS.R - sRP.R;
//		slamraFieldOffset = new Position (
//				sFS.X - (sRP.X*Math.cos(Math.toRadians(offsetR)) - sRP.Y*Math.sin(Math.toRadians(offsetR))),
//				sFS.Y - (sRP.X*Math.sin(Math.toRadians(offsetR)) + sRP.Y*Math.cos(Math.toRadians(offsetR))),
//				offsetR
//		);
//	}
	void setSlamraFieldOffset() {
		Position sFS = slamraFieldStart;
		Position sRP = slamraRobotPose;
		double offsetR = sFS.R - sRP.R;
		slamraFieldOffset = new Position (
				sFS.X - (sRP.X*Math.cos(Math.toRadians(offsetR)) - sRP.Y*Math.sin(Math.toRadians(offsetR))),
				sFS.Y - (sRP.X*Math.sin(Math.toRadians(offsetR)) + sRP.Y*Math.cos(Math.toRadians(offsetR))),
				offsetR *1
		);
	}

//	void setSlamraFinalPose() {
////		//rotates slamra position to field coordinates & add offset
////		double oX, oY, oR, rX, rY, rR, sR;
////		rX = slamraRobotPose.X;
////		rY = slamraRobotPose.Y;
////		rR = slamraRobotPose.R;
////		oX = slamraFieldOffset.X;
////		oY = slamraFieldOffset.Y;
////		oR = slamraFieldOffset.R;
////		//=I11*COS(RADIANS(r_field_slam))-J11*SIN(RADIANS(r_field_slam))  i11=rX, j11=rY
////		slamraFinal = slamraFinal.withX((rX*Math.cos(Math.toRadians(oR)) - rY*Math.sin(Math.toRadians(oR))) + oX);
////		//=I11*SIN(RADIANS(r_field_slam))+J11*COS(RADIANS(r_field_slam))
////		slamraFinal = slamraFinal.withY((rX*Math.sin(Math.toRadians(oR)) + rY*Math.cos(Math.toRadians(oR))) + oY);
////		slamraFinal = slamraFinal.withZ(rR + oR);
//
//		// Proposed slamra transformation code cleanup phase 1
//		Position pose1 = slamraFieldOffset;
//		Position pose2 = slamraRobotPose;
//		slamraFinalPose = new Position(
//				pose1.X + (pose2.X*Math.cos(Math.toRadians(pose1.R)) - pose2.Y*Math.sin(Math.toRadians(pose1.R))),
//				pose1.Y + (pose2.X*Math.sin(Math.toRadians(pose1.R)) + pose2.Y*Math.cos(Math.toRadians(pose1.R))),
//				pose1.R + pose2.R
//		);
//	}
	void setSlamraFinalPose() {
		//pos1 = slamraFieldOffset, pos2 = slamraRobotPose
		slamraFinalPose = transformPosition(slamraFieldOffset, slamraRobotPose);
		slamraFinalPose.normalize();
	}
	//End of LK ????????

	public void onStop() {
		slamra.stop();
	}

	Position transformPosition(Position pos1, Position pos2) {
		return new Position(
				pos1.X + (pos2.X*Math.cos(Math.toRadians(pos1.R)) - pos2.Y*Math.sin(Math.toRadians(pos1.R))),
				pos1.Y + (pos2.X*Math.sin(Math.toRadians(pos1.R)) + pos2.Y*Math.cos(Math.toRadians(pos1.R))),
				pos1.R + pos2.R
		);
	}

	public void addTeleOpTelemetry() {
		telemetry.addData("s-fldof", slamraFieldOffset.toString(2));
		telemetry.addData("s-raw__", slamraRawPose.toString(2));
		telemetry.addData("s-robot", slamraRobotPose.toString(2));
		telemetry.addData("s-final", slamraFinalPose.toString(2));
	}
}