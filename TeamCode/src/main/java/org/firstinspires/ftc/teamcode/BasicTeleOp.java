package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

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

        double turretPosition = 0.5;
        double MAX_TURRETANGLE = Math.toRadians(40.0);
        double MIN_TURRETANGLE = Math.toRadians(-40.0);
        double MAX_SERVO = 1.0;
        double GOAL_X_RED = 66;
        double GOAL_Y_RED = -66;
        double GOAL_X_BLUE = -66;
        double GOAL_Y_BLUE = -66;

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

        Pose2d startPose = new Pose2d(0,0,Math.toRadians(270));
        drive.localizer.setPose(startPose);

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

            //////////////
            // start of aiming

            // false = odometry
            // treu = limelight
            if(true) // limelight
            {
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
                }

                // check if the turret is at a limit and stop it if it is
                telemetry.addData("CW Limit: ", extras.turretLimitCW.isPressed());
                telemetry.addData("CCW Limit: ", extras.turretLimitCCW.isPressed());
                if ((extras.turretLimitCW.isPressed()) && (turretPower > 0))
                {
                    turretPower = 0.0;
                } else if ((extras.turretLimitCCW.isPressed()) && (turretPower < 0))
                {
                    turretPower = 0.0;
                }
                // now set the turret power
                extras.turretCR.setPower(turretPower * 0.6);
                telemetry.addData("turret power: ", extras.turretCR.getPower());
            }
            else // odometry
            {
                double driveHeading = drive.localizer.getPose().heading.toDouble();
                double launcherHeading = driveHeading - Math.PI;
                double goalHeading = 0;
                double turretAngle = 0;
                if (extras.teamColor == ExtraOpModeFunctions.TeamColor.RED)
                {
                    goalHeading = atan2(GOAL_Y_RED - drive.localizer.getPose().position.y, GOAL_X_RED - drive.localizer.getPose().position.x);
                    turretAngle = goalHeading - launcherHeading;
                }
                else
                {
                    goalHeading = atan2(GOAL_Y_BLUE - drive.localizer.getPose().position.y, GOAL_X_BLUE - drive.localizer.getPose().position.x);
                    turretAngle = Math.toRadians(180) - (launcherHeading - goalHeading);
                }
                if(turretAngle > Math.PI)
                {
                    turretAngle = turretAngle - 2*PI;
                }
                else if(turretAngle < -Math.PI)
                {
                    turretAngle = turretAngle + 2*PI;
                }

                telemetry.addData("drive x", drive.localizer.getPose().position.x);
                telemetry.addData("drive y", drive.localizer.getPose().position.y);
                telemetry.addData("drive heading", driveHeading );
                telemetry.addData("goal heading", goalHeading );
                telemetry.addData("turret angle", Math.toDegrees(turretAngle));

                if (targeting == Targeting.AUTO)
                {
                    ;
                }
                else // manual targeting
                {
                    double turretStick = gamepad2.left_stick_x * 0.05;
                    turretAngle = turretAngle + turretStick;
                }

                if(turretAngle > MAX_TURRETANGLE)
                {
                    turretAngle = MAX_TURRETANGLE;
                }
                else if(turretAngle < MIN_TURRETANGLE)
                {
                    turretAngle = MIN_TURRETANGLE;
                }

                turretPosition = turretAngle * ((MAX_SERVO - 0.5)/MAX_TURRETANGLE) + 0.5;
                //extras.turretS.setPosition(turretPosition);
                telemetry.addData("turret voltage", turretPosition);
            }
            // end of aiming
            //////////////

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
            }
            else
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
            adjustedHeading = imuHeading - previousOrientation + PI/2;

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

            //telemetry.addData("pp x", ppLocalizer.driver.getPosX(DistanceUnit.INCH));
            //telemetry.addData("pp y", ppLocalizer.driver.getPosY(DistanceUnit.INCH));
            //telemetry.addData("pp heading", ppLocalizer.driver.getHeading(AngleUnit.DEGREES));
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

