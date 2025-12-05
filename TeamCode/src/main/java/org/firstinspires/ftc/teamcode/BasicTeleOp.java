package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
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

        boolean shooterOn = false;

        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //extras.wristMiddle();
        //extras.clawOpen();

        double previousOrientation = extras.readAutoStartRotation();
        extras.teamColor = extras.readTeamColor();

        boolean initArmAtStart = false;


        double lightColor = extras.Light_Red;
        boolean targetSearchingMode = false;
        long loopCounter = 0;

        telemetry.addData("Team Color: ", extras.teamColor);
        telemetry.addData("adjustedAngle: ", Math.toDegrees(drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        telemetry.addData("previousOrientation: ", Math.toDegrees(previousOrientation));
        //telemetry.addData("Odo Orientation: ", drive.odo.getHeading());
        telemetry.addData("Init Complete", initArmAtStart);

        telemetry.update();

        waitForStart();

        while (!isStopRequested())
        {
            // change team color if needed
            if(gamepad2.xWasPressed())
            {
                if(extras.teamColor == ExtraOpModeFunctions.TeamColor.RED)
                    extras.teamColor = ExtraOpModeFunctions.TeamColor.BLUE;
                else
                    extras.teamColor = ExtraOpModeFunctions.TeamColor.RED;
            }
            telemetry.addData("Team Color: ", extras.teamColor);

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
            telemetry.addData("Targeting Mode: ", targeting);

            if (targeting == Targeting.AUTO)
            {
                ExtraOpModeFunctions.TrackDepotState trackDepotState = extras.trackDepot();
                switch (trackDepotState)
                {
                    case NOTFOUND:
                        extras.turret.setPower(gamepad2.left_stick_x * 0.5);
                        lightColor = extras.Light_Red;
                        break;
                    case ONTARGET:
                        lightColor = extras.Light_Green;
                        break;
                    case TARGETING:
                        lightColor = extras.Light_Blue;
                        break;
                }
            }
            else // manual targeting
            {
                lightColor = extras.Light_Purple;
                // buttons for rotate
                extras.turretPower = gamepad2.left_stick_x * 0.5;
                if((extras.turretLimitCW.isPressed())&&(extras.turretPower>0))
                {
                    extras.turretPower = 0.0;
                }
                else if((extras.turretLimitCCW.isPressed())&&(extras.turretPower<0))
                {
                    extras.turretPower = 0.0;
                }
                extras.turret.setPower(gamepad2.left_stick_x * 0.5);

                // buttons for range
                if (gamepad2.dpadUpWasPressed())
                {
                    extras.shooterVelocity = extras.shooterVelocity + 50.0;
                    if (extras.shooterVelocity > extras.maxShooterTPS)
                    {
                        extras.shooterVelocity = extras.maxShooterTPS;
                    }
                }
                if (gamepad2.dpadDownWasPressed())
                {
                    extras.shooterVelocity = extras.shooterVelocity - 50.0;
                    if (extras.shooterVelocity < 0.0)
                    {
                        extras.shooterVelocity = 0.0;
                    }
                }
            }

            extras.light.setPosition(lightColor);

            //extras.vision.readColorSensors();

            // shooter on/off function
            if(gamepad1.aWasPressed())
            {
                if(shooterOn == true)
                {
                    shooterOn = false;
                }
                else
                {
                    shooterOn = true;
                }
            }
            if(shooterOn == true)
            {
                extras.setShooter(extras.shooterVelocity);
            }
            else
            {
                extras.setShooter(0.0);
            }
            telemetry.addData("Shooter velocity set: ", extras.shooterVelocity);
            telemetry.addData("Shooter1 velocity actual: ", extras.shooter1.getVelocity());
            telemetry.addData("Shooter2 velocity actual: ", extras.shooter2.getVelocity());

            // intake and shooter control
            if (gamepad1.right_trigger > 0)
            {
                extras.intakeForward();
                extras.ballStopOff();
            }
            else if (gamepad1.left_trigger > 0)
            {
                extras.intakeForward();
            }
            else
            {
                extras.intakeOff();
                extras.ballStopOn();
            }

            // DRIVE CONTROL STARTS HERE
            if (gamepad1.left_bumper)
                speedMultiplier = 0.55;
            else
                speedMultiplier = 0.7;

            //adjustedAngle = 0;
            adjustedAngle = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            if(extras.teamColor == ExtraOpModeFunctions.TeamColor.RED)
            {
                adjustedAngle = adjustedAngle + PI/2 + previousOrientation;
            }
            else  // team color is blue
            {
                adjustedAngle = adjustedAngle - PI/2 + previousOrientation;
            }

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

                //drive.lazyImu.get().initialize(imuParameters);
                drive.lazyImu.get().resetYaw();
                sleep(500);
                previousOrientation = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                //drive.odo.resetPosAndIMU();
                //previousOrientation = drive.odo.getHeading()+ PI/2;
                extras.saveAutoStartRotation(previousOrientation);
            }


            //telemetry.addData("x", drive.pose.position.x);
            //telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.localizer.getPose().heading);

            telemetry.addData("adjustedAngle: ", Math.toDegrees(adjustedAngle));
            telemetry.addData("previousOrientation: ", Math.toDegrees(previousOrientation));
            //telemetry.addData("ODO adjusted angle", adjustedAngle);
            telemetry.addData("IMU angle", Math.toDegrees(drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));

            //Pose2D pos = drive.odo.getPosition();
            //String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            //telemetry.addData("Position", data);

            //telemetry.addLine();

            telemetry.addData("Elapsed time: ", getRuntime());

            drive.updatePoseEstimate();

            /*
            datalog.loopCounter.set(loopCounter);
            datalog.cameraPosition.set(extras.turretPower);
            datalog.targetHeading.set(extras.targetHeading);
            datalog.imuHeading.set(Math.toDegrees(adjustedAngle));
            datalog.writeLine();
            */

            telemetry.update();
        }
    }

}

