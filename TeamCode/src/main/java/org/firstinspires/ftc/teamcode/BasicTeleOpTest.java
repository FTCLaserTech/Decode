package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled

@TeleOp(group = "A")
public class BasicTeleOpTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        ExtraOpModeFunctionsTest extras = new ExtraOpModeFunctionsTest(hardwareMap, this);
        //TrajectoryBook book = new TrajectoryBook(drive, extras);

        /*
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                drive.PARAMS.logoFacingDirection,
                drive.PARAMS.usbFacingDirection));
        int IMUReset = 0;
        */
        double stickForward;
        double stickSideways;
        double stickForwardRotated;
        double stickSidewaysRotated;
        double adjustedAngle;
        double speedMultiplier;

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
        double stickRotation;

        double maxShooterRPM = 6000.0;  //RPM
        double shooterTicksPerRev = 28.0;
        double maxShooterTPS = shooterTicksPerRev * maxShooterRPM / 60; // 2800


        double shooterVelocity = 0.0;

        double previousOrientation = extras.readAutoStartRotation();

        telemetry.addData("Previous Orientation: ", previousOrientation);
        //telemetry.addData("Odo Orientation: ", drive.odo.getHeading());
        telemetry.update();

        waitForStart();

        while (!isStopRequested())
        {

            if (gamepad1.left_bumper)
                speedMultiplier = 0.85;
            else
                speedMultiplier = 1.0;

            adjustedAngle = previousOrientation;

            if(gamepad1.xWasPressed())
            {
                shooterVelocity = 0.0;
            }
            if(gamepad1.bWasPressed())
            {
                shooterVelocity = 2000.0;
            }
            if(gamepad1.yWasPressed())
            {
                shooterVelocity = shooterVelocity + 50.0;
                if (shooterVelocity > maxShooterTPS)
                {
                    shooterVelocity = maxShooterTPS;
                }
            }
            if(gamepad1.aWasPressed())
            {
                shooterVelocity = shooterVelocity - 50.0;
                if (shooterVelocity < 0.0)
                {
                    shooterVelocity = 0.0;
                }
            }

            extras.shooter1.setVelocity(shooterVelocity);
            extras.shooter2.setVelocity(shooterVelocity);

            telemetry.addData("Shooter velocity set: ", shooterVelocity);
            telemetry.addData("Shooter1 velocity actual: ", extras.shooter1.getVelocity());
            telemetry.addData("Shooter2 velocity actual: ", extras.shooter2.getVelocity());


            if (extras.beamBreak.isPressed())
            {
                telemetry.addData("Touch Sensor", "Is Pressed");
            }
            else
            {
                telemetry.addData("Touch Sensor", "Is Not Pressed");
            }

            telemetry.addData("turretLimitCW: ", extras.turretLimitCW.isPressed());
            telemetry.addData("turretLimitCCW: ", extras.turretLimitCCW.isPressed());

            /*
            stickSideways = gamepad1.left_stick_x * speedMultiplier;
            stickForward = -gamepad1.left_stick_y * speedMultiplier;
            stickSidewaysRotated = (stickSideways * Math.cos(-adjustedAngle)) - (stickForward * Math.sin(-adjustedAngle));
            stickForwardRotated = (stickSideways * Math.sin(-adjustedAngle)) + (stickForward * Math.cos(-adjustedAngle));

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            stickSidewaysRotated,
                            stickForwardRotated
                    ),
                    -gamepad1.right_stick_x
            ));
            drive.updatePoseEstimate();
            */

            /*
            // RESET IMU
            if ((gamepad1.back) && (gamepad1.b))
            {
                IMUReset = 1;
            }
            else if (IMUReset == 1)
            {
                IMUReset = 0;
                telemetry.addLine("IMU Resetting...");
                telemetry.update();

                //drive.imu.initialize(imuParameters);
                //drive.odo.resetPosAndIMU();
                //previousOrientation = drive.odo.getHeading()+ PI/2;
                extras.saveAutoStartRotation(previousOrientation);
                sleep(500);
            }
            */

            telemetry.addData("ODO adjusted angle", adjustedAngle);

            telemetry.addData("Elapsed time: ", getRuntime());

            telemetry.update();
        }
    }
}