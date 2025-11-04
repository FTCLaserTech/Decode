package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(group = "A")

public class AprilTagLimelightTest extends OpMode {

    private Limelight3A limelight;

    private IMU imu;

    private long loopCounter = 0;


  @Override
    public void init() {
      limelight = hardwareMap.get(Limelight3A.class, "limelight");
      limelight.pipelineSwitch(8);
      imu = hardwareMap.get(IMU.class, "imu");
      RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
              RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
      imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
      telemetry.addData("init", 0);
      telemetry.update();


  }

  @Override
    public void start() {
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
      if (llResult !=null && llResult.isValid()) {
          Pose3D botPose = llResult.getBotpose();
          telemetry.addData("Tx", llResult.getTx());
          telemetry.addData("Ty", llResult.getTy());
          telemetry.addData("Ta", llResult.getTa());

      }
      telemetry.update();

  }

}
