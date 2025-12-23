package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
        sleep(300);
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
        double imuHeading = 0.0;
        double adjustedHeading = 0.0;
        double speedMultiplier = 1.0;
        double rotationMultiplier = 1.0;

        double turretPower = 0.0;
        double launcherSpeed = 0.0;

        boolean launcherOn = true;
        boolean depotFound = false;

        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //extras.wristMiddle();
        //extras.clawOpen();

        double lightColor = extras.Light_Red;

        double previousOrientation = extras.readAutoStartRotation();
        extras.teamColor = extras.readTeamColor();
        if (extras.teamColor == ExtraOpModeFunctions.TeamColor.RED)
        {
            lightColor = extras.Light_Red;
        }
        else
        {
            lightColor = extras.Light_Blue;
        }
        extras.light1.setPosition(lightColor);
        extras.light2.setPosition(lightColor);


        boolean targetSearchingMode = false;
        long loopCounter = 0;

        telemetry.addData("Team Color: ", extras.teamColor);

        PinpointLocalizer ppLocalizer = (PinpointLocalizer) drive.localizer;
        double ppYaw = ppLocalizer.driver.getHeading(AngleUnit.RADIANS);
        double imuYaw = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addData("ppYaw r: ", ppYaw);
        telemetry.addData("chYaw r: ", imuYaw);
        telemetry.addData("ppYaw d: ", Math.toDegrees(ppYaw));
        telemetry.addData("chYaw d: ", Math.toDegrees(imuYaw));
        telemetry.addData("savedAngle r: ", previousOrientation);
        telemetry.addData("savedAngle d: ", Math.toDegrees(previousOrientation));

        telemetry.addLine("Init Complete");
        telemetry.update();

        waitForStart();

        extras.vision.limelight.start();

        while (!isStopRequested())
        {
            // change team color if needed
            if (gamepad2.xWasPressed())
            {
                if (extras.teamColor == ExtraOpModeFunctions.TeamColor.RED)
                {
                    extras.teamColor = ExtraOpModeFunctions.TeamColor.BLUE;
                    lightColor = extras.Light_Blue;
                }
                else
                {
                    extras.teamColor = ExtraOpModeFunctions.TeamColor.RED;
                    lightColor = extras.Light_Red;
                }
                extras.light1.setPosition(lightColor);
                extras.light2.setPosition(lightColor);
            }
            telemetry.addData("Team Color: ", extras.teamColor);

            // change between AUTO and MANUAL targeting
            if (gamepad2.dpadRightWasPressed())
            {
                if (targeting == Targeting.AUTO)
                {
                    targeting = Targeting.MANUAL;
                }
                else
                {
                    targeting = Targeting.AUTO;
                }
            }
            telemetry.addData("Targeting Mode: ", targeting);

            // launcher on/off function
            if (gamepad1.aWasPressed())
            {
                if (launcherOn == true)
                {
                    launcherOn = false;
                } else
                {
                    launcherOn = true;
                }
            }
            telemetry.addData("Launcher On: ", launcherOn);

            // Look for the Depot
            depotFound = extras.lookForDeopt();

            if ((targeting == Targeting.AUTO) && (depotFound))
            {
                turretPower = extras.autoAimTurret();
                telemetry.addData("AutoTurret Aim OK? ", extras.isTurretAimGood());

                launcherSpeed = extras.autoLauncherSpeed();
            }
            else // manual targeting
            {
                // manual turret
                // joystick for rotate
                turretPower = gamepad2.left_stick_x * 0.5;

                // manual speed
                // buttons for range
                if (gamepad2.dpadUpWasPressed())
                {
                    launcherSpeed = launcherSpeed + 20.0;
                    //launcherSpeed = launcherSpeed + 0.05;
                }
                if (gamepad2.dpadDownWasPressed())
                {
                    launcherSpeed = launcherSpeed - 20.0;
                    //launcherSpeed = launcherSpeed - 0.05;
                }
            }

            // check if the turret is at a limit and stop it if it is
            telemetry.addData("CW Limit: ", extras.turretLimitCW.isPressed());
            telemetry.addData("CCW Limit: ", extras.turretLimitCCW.isPressed());
            if ((extras.turretLimitCW.isPressed()) && (turretPower > 0))
            {
                turretPower = 0.0;
            }
            else if ((extras.turretLimitCCW.isPressed()) && (turretPower < 0))
            {
                turretPower = 0.0;
            }
            // now set the turret power
            extras.turret.setPower(turretPower * 0.5);
            telemetry.addData("turret power: ", extras.turret.getPower());

            // check if the launcher speed is in range and adjust if needed
            if (launcherSpeed > extras.maxLauncherTPS)
            {
                launcherSpeed = extras.maxLauncherTPS;
            }
            else if (launcherSpeed < 0.0)
            {
                launcherSpeed = 0.0;
            }
            // now set the launcher speed
            if (launcherOn == false)
            {
                launcherSpeed = 0.0;
            }
            extras.setLauncher(launcherSpeed);

            extras.setLights();

            // intake and banana control
            if (gamepad1.right_bumper)
            {
                extras.intakeReverse();
            }
            else if (gamepad1.right_trigger > 0)
            {
                extras.intakeForward();
                extras.ballStopOff();
            }
            else if (gamepad1.left_trigger > 0)
            {
                extras.intakeForward();
            } else
            {
                extras.intakeOff();
                extras.ballStopOn();
            }

            // DRIVE CONTROL STARTS HERE
            if (gamepad1.left_bumper)
            {
                speedMultiplier = 0.55;
                rotationMultiplier = 0.55;
            }
            else
            {
                speedMultiplier = 1.0;
                rotationMultiplier = 1.0;
            }

            imuHeading = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            adjustedHeading = imuHeading - previousOrientation - PI/2;

            stickSideways = gamepad1.left_stick_x * speedMultiplier;
            stickForward = -gamepad1.left_stick_y * speedMultiplier;
            stickSidewaysRotated = (stickSideways * Math.cos(-adjustedHeading)) - (stickForward * Math.sin(-adjustedHeading));
            stickForwardRotated = (stickSideways * Math.sin(-adjustedHeading)) + (stickForward * Math.cos(-adjustedHeading));

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            stickSidewaysRotated,
                            stickForwardRotated
                    ),
                    -(gamepad1.right_stick_x * rotationMultiplier)
            ));

            if(gamepad2.yWasPressed())
            {
                //extras.s1down();
                telemetry.addData("y Pressed", "s1down");
            }
            if(gamepad2.bWasPressed())
            {
                extras.freezeRange = true;
                telemetry.addData("b Pressed", "s2up");
            }

            if(gamepad2.aWasPressed())
            {
                extras.freezeRange = false;
                telemetry.addData("a Pressed", "0.0");
            }
            telemetry.addData("freezeRange", extras.freezeRange);


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
            //telemetry.addData("heading", drive.localizer.getPose().heading);
            telemetry.addData("IMU Heading: ", Math.toDegrees(imuHeading));
            telemetry.addData("adjustedHeading: ", Math.toDegrees(adjustedHeading));
            telemetry.addData("previousOrientation: ", Math.toDegrees(previousOrientation));
            //telemetry.addData("ODO adjusted angle", adjustedHeading);
            //telemetry.addData("IMU angle", Math.toDegrees(drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));

            //Pose2D pos = drive.odo.getPosition();
            //String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            //telemetry.addData("Position", data);

            //telemetry.addLine();

            telemetry.addData("Elapsed time: ", getRuntime());

            drive.updatePoseEstimate();

            telemetry.update();
        }
    }

}

