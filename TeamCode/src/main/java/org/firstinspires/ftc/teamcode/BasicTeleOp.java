package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.sqrt;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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

        Targeting targeting = Targeting.MANUAL;

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

        double turretAngle = 0;
        double turretPosition = 0.5;
        double MAX_TURRETANGLE = Math.toRadians(155.0);
        double MIN_TURRETANGLE = Math.toRadians(-155.0);
        double MAX_SERVO = 1.0;

        double turretMotorEncoder = 384.5;  // PPR at the output shaft per motor data sheet
        double turretBaseTeeth = 200.0;
        double driveTeeth = 50.0;
        double MAX_TURRETENCODER = turretMotorEncoder * (turretBaseTeeth/driveTeeth) * (MAX_TURRETANGLE/Math.toRadians(360));
        //double MAX_TURRETENCODER = 1955.0;

        double GOAL_X_RED = 59; //62
        double GOAL_Y_RED = -56; //-62
        double GOAL_X_BLUE = 59; //62
        double GOAL_Y_BLUE = 56; //62
        double APRIL_TAG_X_RED = 59;
        double APRIL_TAG_Y_RED = -56;
        double APRIL_TAG_X_BLUE = 59;
        double APRIL_TAG_Y_BLUE = 56;

        double turretPower = 0.0;
        double launcherSpeed = 0.0;

        boolean launcherOn = true;
        boolean depotFound = false;

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
            imuHeading = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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
                extras.saveTeamColor(extras.teamColor);

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

            //////////////
            // start of aiming

            // Look for the Depot
            depotFound = extras.lookForDeopt();
            if (depotFound)
            {
                telemetry.addData("limelightTargetRange: ", extras.getAprilTagPose().range);
            }

            // false = odometry
            // true = limelight
            if(false) // limelight
            {
                if ((targeting == Targeting.AUTO) && (depotFound))
                {
                    turretPower = extras.autoAimTurret();
                    telemetry.addData("AutoTurret Aim OK? ", extras.isTurretAimGood());

                    launcherSpeed = extras.limelightLauncherSpeed();
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
                double drivePositionX = drive.localizer.getPose().position.x;
                double drivePositionY = drive.localizer.getPose().position.y;
                double driveHeading = drive.localizer.getPose().heading.toDouble();
                double launcherHeading = driveHeading - Math.PI;
                double goalDistance = 0;
                double goalHeading = 0;
                if (extras.teamColor == ExtraOpModeFunctions.TeamColor.RED)
                {
                    goalDistance = sqrt((APRIL_TAG_X_RED - drivePositionX) * (APRIL_TAG_X_RED - drivePositionX) + ( APRIL_TAG_Y_RED - drivePositionY) * (APRIL_TAG_Y_RED - drivePositionY) + (29.5-14) * (29.5-14))-5.875;
                    goalHeading = atan2(GOAL_Y_RED - drivePositionY, GOAL_X_RED - drivePositionX);
                }
                else
                {
                    goalDistance = sqrt((APRIL_TAG_X_BLUE - drivePositionX) * (APRIL_TAG_X_BLUE - drivePositionX) + ( APRIL_TAG_Y_BLUE - drivePositionY) * (APRIL_TAG_Y_BLUE - drivePositionY) + (29.5-14) * (29.5-14))-5.875;
                    goalHeading = atan2(GOAL_Y_BLUE - drivePositionY, GOAL_X_BLUE - drivePositionX);
                }

                telemetry.addData("drive x", drivePositionX);
                telemetry.addData("drive y", drivePositionY);
                telemetry.addData("drive heading", driveHeading );
                telemetry.addData("goal heading", goalHeading );
                telemetry.addData("goal distance", goalDistance );

                if (targeting == Targeting.AUTO)
                {
                    turretAngle = goalHeading - launcherHeading;
                    if(turretAngle > Math.PI)
                    {
                        turretAngle = turretAngle - 2*PI;
                    }
                    else if(turretAngle < -Math.PI)
                    {
                        turretAngle = turretAngle + 2*PI;
                    }

                    launcherSpeed = extras.odometryLauncherSpeed(goalDistance);
                }
                else // manual targeting
                {
                    // aim
                    double turretStick = gamepad2.left_stick_x * 0.02;
                    turretAngle = turretAngle + turretStick;

                    // range
                    if(gamepad2.dpadUpWasPressed())
                    {
                        launcherSpeed = launcherSpeed + 10;
                    }
                    if(gamepad2.dpadDownWasPressed())
                    {
                        launcherSpeed = launcherSpeed - 10;
                    }
                }

                if(turretAngle > MAX_TURRETANGLE)
                {
                    turretAngle = MAX_TURRETANGLE;
                }
                else if(turretAngle < MIN_TURRETANGLE)
                {
                    turretAngle = MIN_TURRETANGLE;
                }

                //turretPosition = turretAngle * ((MAX_SERVO - 0.5)/MAX_TURRETANGLE) + 0.5;
                //extras.turretS.setPosition(turretPosition);

                turretPosition = turretAngle / MAX_TURRETANGLE * MAX_TURRETENCODER;

                if(false)
                {
                    extras.turretMotor.setTargetPosition((int) turretPosition);
                    //extras.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extras.turretMotor.setPower(1.0);
                    telemetry.addData("turret angle", Math.toDegrees(turretAngle));
                    telemetry.addData("turret position", turretPosition);
                    telemetry.addData("turret encoder", extras.turretMotor.getCurrentPosition());
                }
                else
                {
                    extras.setTurret(turretPosition);
                }

                Pose3D pose3D = extras.vision.getRobotFieldPosition(imuHeading);
                telemetry.addData("RFP x", pose3D.getPosition().x*39.3700787);
                telemetry.addData("RFP y", pose3D.getPosition().y*39.3700787);
                telemetry.addData("RFP z", pose3D.getPosition().z*39.3700787);
                telemetry.addData("RFP roll", pose3D.getOrientation().getRoll(AngleUnit.DEGREES));
                telemetry.addData("RFP pitch", pose3D.getOrientation().getPitch(AngleUnit.DEGREES));
                telemetry.addData("RFP yaw", pose3D.getOrientation().getYaw(AngleUnit.DEGREES));
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
                extras.ballStopOff();
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

            /*
            if(gamepad2.left_bumper)
            {
                extras.turretMotorForward();
            }
            else if(gamepad2.right_bumper)
            {
                extras.turretMotorReverse();
            }
            else
            {
                extras.turretMotorOff();
            }
             */

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

            // reset the turret motor encoder
            if(gamepad2.yWasPressed())
            {
                extras.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extras.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // freeze the launcher motor speed
            if(gamepad2.bWasPressed())
            {
                extras.freezeRange = true;
                telemetry.addData("b Pressed", "s2up");
            }

            // unfreeze the launcher motor speed
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

