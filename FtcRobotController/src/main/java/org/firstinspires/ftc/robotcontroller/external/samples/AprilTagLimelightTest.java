package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(group = "A")

public class AprilTagLimelightTest extends OpMode {

    private Limelight3A limelight;

    private IMU imu;

    private long loopCounter = 0;


  @Override
    public void init() {
      limelight = hardwareMap.get(Limelight3A.class, "limelight");
      limelight.pipelineSwitch(0);
      imu = hardwareMap.get(IMU.class, "imu");
      RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
              RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
      imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
      telemetry.addData("init", 0);
      telemetry.update();


  }

  @Override
    public void start() {
      limelight.start();
      telemetry.addData("start", 1);
      telemetry.update();
  }

  @Override
    public void loop() {
      telemetry.addData("loop", loopCounter);
      loopCounter += 1;
      YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
      limelight.updateRobotOrientation(orientation.getYaw());
      LLResult llResult = limelight.getLatestResult();
      telemetry.addData("yaw ", orientation.getYaw());
      if (llResult !=null && llResult.isValid()) {
          Pose3D botPose = llResult.getBotpose_MT2();
          telemetry.addData("Tx", llResult.getTx());
          telemetry.addData("Ty", llResult.getTy());
          telemetry.addData("Ta", llResult.getTa());

          List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
          for (LLResultTypes.FiducialResult fiducial : fiducials) {
              int id = fiducial.getFiducialId(); // The ID number of the fiducial
              double x = fiducial.getTargetXDegrees(); // Where it is (left-right)
              telemetry.addData(" x ", x);
              double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
              telemetry.addData(" y ", y);
              double distance = fiducial.getRobotPoseTargetSpace().getPosition().y;
              telemetry.addData("Fiducial " + id, "is " + distance + " meters away");
          }
      }
      telemetry.update();

  }

}
