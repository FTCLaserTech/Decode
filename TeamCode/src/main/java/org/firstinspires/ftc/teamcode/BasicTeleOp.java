package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

//imports from the Mecanum website


@TeleOp(group = "A")
public class BasicTeleOp extends LinearOpMode
{
    public enum Targeting{MANUAL,AUTO};

    @Override
    public void runOpMode() throws InterruptedException
    {
        ExtraOpModeFunctions.Datalog datalog;
        datalog = new ExtraOpModeFunctions.Datalog("datalog_01");

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);

        Targeting targeting = Targeting.AUTO;


        //TrajectoryBook book = new TrajectoryBook(drive, extras);

        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                drive.PARAMS.logoFacingDirection,
                drive.PARAMS.usbFacingDirection));
        int IMUReset = 0;
        int elevatorBottomThreshold = 200;
        double stickForward;
        double stickSideways;
        double stickForwardRotated;
        double stickSidewaysRotated;
        double adjustedAngle;
        double speedMultiplier;


        boolean manualStickMove = false;

        double slope;

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
        double stickRotation;

        double armCurrent = 0;
        double armMaxCurrent = 0;
        int numDangerArmAmps = 0;
        double elevatorCurrent = 0;
        double elevatorMaxCurrent = 0;
        int numDangerElevatorAmps = 0;
        boolean manualLiftStop = false;

        String slot1Detections = "NNNNNNNNNNNNNNNNNNNNNNNNNNNNNN";
        String slot2Detections = "NNNNNNNNNNNNNNNNNNNNNNNNNNNNNN";
        String slot3Detections = "NNNNNNNNNNNNNNNNNNNNNNNNNNNNNN";
        String slot1Artifact = "N";
        String slot2Artifact = "N";
        String slot3Artifact = "N";

        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //extras.wristMiddle();
        //extras.clawOpen();

        double previousOrientation = extras.readAutoStartRotation();

        boolean initArmAtStart = false;


        boolean targetSearchingMode = false;
        long loopCounter = 0;

        NormalizedRGBA colors1;
        NormalizedRGBA colors2;
        NormalizedRGBA colors3;
        final float[] hsvValues1 = new float[3];
        final float[] hsvValues2 = new float[3];
        final float[] hsvValues3 = new float[3];

        telemetry.addData("Previous Orientation: ", previousOrientation);
        //telemetry.addData("Odo Orientation: ", drive.odo.getHeading());
        telemetry.addData("Init Complete", initArmAtStart);

        telemetry.update();

        waitForStart();

        while (!isStopRequested())
        {
            telemetry.addData("team color", extras.teamColor);

            if (gamepad2.dpadRightWasPressed())
            {
                if (targeting == Targeting.AUTO)
                {
                    targeting = Targeting.MANUAL;
                } else
                {
                    targeting = Targeting.AUTO;
                }
            }

            if (targeting == Targeting.AUTO)
            {
                extras.trackDepot();
            } else // manual targeting
            {
                // buttons for rotate
                extras.turret.setPower(gamepad2.left_stick_x * 0.5);

                // buttons for range
                if (gamepad2.dpad_up)
                {
                    extras.shooterVelocity = extras.shooterVelocity + 50.0;
                    if (extras.shooterVelocity > extras.maxShooterTPS)
                    {
                        extras.shooterVelocity = extras.maxShooterTPS;
                    }
                }
                if (gamepad2.dpad_down)
                {
                    extras.shooterVelocity = extras.shooterVelocity - 50.0;
                    if (extras.shooterVelocity < 0.0)
                    {
                        extras.shooterVelocity = 0.0;
                    }
                }

            }

            extras.shooter1.setVelocity(extras.shooterVelocity);
            extras.shooter2.setVelocity(extras.shooterVelocity);

            telemetry.addData("Shooter velocity set: ", extras.shooterVelocity);
            telemetry.addData("Shooter1 velocity actual: ", extras.shooter1.getVelocity());
            telemetry.addData("Shooter2 velocity actual: ", extras.shooter2.getVelocity());
            // shooter on/off function


            // intake control
            if (gamepad1.right_trigger > 0)
            {
                extras.intake1Forward();
            }
            else
            {
                extras.intake1Off();
            }

            // shoot all
            if (gamepad1.left_trigger > 0)
            {
                extras.intake1Forward();
                extras.intake2Forward();
            }
            else
            {
                extras.intake1Off();
                extras.intake2Off();
            }



            if (gamepad2.left_bumper)
                speedMultiplier = 0.85;
            else
                speedMultiplier = 1.0;

            //adjustedAngle = 0;
            //adjustedAngle = extras.adjustAngleForDriverPosition(drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), ExtraOpModeFunctions.RobotStartPosition.STRAIGHT);
            adjustedAngle = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            //drive.odo.update();
            //adjustedAngle = drive.odo.getHeading() + (Math.PI / 2);
            //adjustedAngle = drive.odo.getHeading() + previousOrientation;
            //adjustedAngle = previousOrientation;

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

            // change team color if needed
            if(gamepad2.xWasPressed())
            {
                if(extras.teamColor == ExtraOpModeFunctions.TeamColor.RED)
                    extras.teamColor = ExtraOpModeFunctions.TeamColor.BLUE;
                else
                    extras.teamColor = ExtraOpModeFunctions.TeamColor.RED;
            }

            if(gamepad2.yWasPressed())
            {
                //extras.s1down();
                telemetry.addData("y Pressed", "s1down");
            }
            if(gamepad2.bWasPressed())
            {
                //extras.s2up();
                telemetry.addData("b Pressed", "s2up");
            }

            if(gamepad2.aWasPressed())
            {
                //extras.turret.setPower(0.0);
                telemetry.addData("a Pressed", "0.0");
            }

            colors1 = extras.vision.colorSensor1.getNormalizedColors();
            colors2 = extras.vision.colorSensor2.getNormalizedColors();
            colors3 = extras.vision.colorSensor3.getNormalizedColors();

            slot1Detections = checkArtifact(colors1.green, colors1.blue) + slot1Detections.substring(0, slot1Detections.length() - 1);
            slot2Detections = checkArtifact(colors2.green, colors2.blue) + slot2Detections.substring(0, slot2Detections.length() - 1);
            slot3Detections = checkArtifact(colors3.green, colors3.blue) + slot3Detections.substring(0, slot3Detections.length() - 1);

            if(slot1Detections.indexOf("G") != -1)
                slot1Artifact = "G";
            else if(slot1Detections.indexOf("P") != -1)
                slot1Artifact = "P";
            else
                slot1Artifact = "N";

            if(slot2Detections.indexOf("G") != -1)
                slot2Artifact = "G";
            else if(slot2Detections.indexOf("P") != -1)
                slot2Artifact = "P";
            else
                slot2Artifact = "N";

            if(slot3Detections.indexOf("G") != -1)
                slot3Artifact = "G";
            else if(slot3Detections.indexOf("P") != -1)
                slot3Artifact = "P";
            else
                slot3Artifact = "N";

            telemetry.addLine(slot1Artifact + " " + slot2Artifact + " " + slot3Artifact);

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

                drive.lazyImu.get().initialize(imuParameters);
                //drive.odo.resetPosAndIMU();
                //previousOrientation = drive.odo.getHeading()+ PI/2;
                extras.saveAutoStartRotation(previousOrientation);
                sleep(500);
            }


            //telemetry.addData("x", drive.pose.position.x);
            //telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.localizer.getPose().heading);

            telemetry.addData("IMU heading: ", Math.toDegrees(adjustedAngle));
            //telemetry.addData("ODO adjusted angle", adjustedAngle);
            //telemetry.addData("IMU angle", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            //Pose2D pos = drive.odo.getPosition();
            //String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            //telemetry.addData("Position", data);

            //telemetry.addLine();

            telemetry.addData("Elapsed time: ", getRuntime());

            drive.updatePoseEstimate();

            datalog.loopCounter.set(loopCounter);
            datalog.cameraPosition.set(extras.turretPower);
            datalog.targetHeading.set(extras.targetHeading);
            datalog.imuHeading.set(Math.toDegrees(adjustedAngle));
            datalog.writeLine();

            telemetry.update();
        }
    }

    String checkArtifact(float green, float blue)
    {
        double threshold = 0.020;
        if((green>threshold) && (blue>threshold))
        {
            if (green > blue)
            {
                //telemetry.addLine("Green");
                return("G");
            }
            else
            {
                //telemetry.addLine("Purple");
                return("P");
            }
        }
        else
        {
            //telemetry.addLine("No Ball");
            return("N");
        }
    }

}

