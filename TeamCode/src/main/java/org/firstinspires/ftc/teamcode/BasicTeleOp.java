package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//imports from the Mecanum website

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(group = "A")
public class BasicTeleOp extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0), this);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);
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

        boolean gp2_left_trigger_pressed = false;
        boolean gp2_left_bumper_pressed = false;
        boolean gp2_dpad_left_pressed = false;
        boolean gp2_dpad_right_pressed = false;
        boolean gp2_dpad_up_pressed = false;
        boolean gp2_dpad_down_pressed = false;
        boolean gp2_right_bumper_pressed = false;
        boolean gp2_right_trigger_pressed = false;
        boolean gp2_right_stick_y_neg_pressed = false;
        boolean gp2_right_stick_y_pos_pressed = false;
        boolean gp2_a_pressed = false;
        boolean gp2_b_pressed = false;
        boolean gp2_y_pressed = false;
        boolean gp1_y_pressed = false;
        boolean gp1_a_pressed = false;
        boolean gp2_x_pressed = false;
        boolean gp2_by_pressed = false;
        boolean gp1_bx_pressed = false;
        boolean gp2_bx_pressed = false;


        boolean elevatorStopped = true;
        boolean elevatorBottom = true;
        boolean intakeOn = false;
        boolean manualStickMove = false;
        boolean intakeElevatorShouldMove = false;
        boolean elevatorResetOK= false;

        double elevMultMin = 0.5;
        double elevMult = 0;
        double elevHeightMax = 800;
        double slope;
        double elevatorEncoderCounts;
        double rightBackEncoderCounts;;
        double rightFrontEncoderCounts;;
        double leftBackEncoderCounts;;
        double leftFrontEnocoderCounts;;

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

        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //extras.wristMiddle();
        //extras.clawOpen();

        double previousOrientation = extras.readAutoStartRotation();

        boolean initArmAtStart = false;
        if(gamepad2.x)
        {
            initArmAtStart = true;
        }



        telemetry.addData("Previous Orientation: ", previousOrientation);
        telemetry.addData("Odo Orientation: ", drive.odo.getHeading());
        telemetry.addData("Init Complete", initArmAtStart);
        telemetry.update();


        waitForStart();

        while (!isStopRequested())
        {
            if(initArmAtStart == true)
            {
                extras.initArm();
                initArmAtStart = false;
            }

            elevatorCurrent = extras.elevator.getCurrent(CurrentUnit.AMPS);
            if (elevatorCurrent > elevatorMaxCurrent)
            {
                elevatorMaxCurrent = elevatorCurrent;
            }

            if (elevatorCurrent >= 7)
            {
                numDangerElevatorAmps += 1;
                telemetry.addLine("WARNING: HIGH ELEVATOR AMPS");
                if(numDangerElevatorAmps >= 10)
                {
                    // in future, ONLY stop elevator motion
                    stop();
                }
            }
            else
            {
                numDangerElevatorAmps = 0;
            }

            armCurrent = extras.arm.getCurrent(CurrentUnit.AMPS);
            if (armCurrent > armMaxCurrent)
            {
                armMaxCurrent = armCurrent;
            }
            if (armCurrent >= 7)
            {
                numDangerArmAmps += 1;
                telemetry.addLine("WARNING: HIGH AMPS");
                if(numDangerArmAmps >= 10)
                {
                    // in future, ONLY stop arm motion
                    stop();
                }
            }
            else
            {
                numDangerArmAmps = 0;
            }

            if (gamepad1.left_bumper)
                speedMultiplier = 0.85;
            else
                speedMultiplier = 1.0;

            //adjustedAngle = 0;
            //adjustedAngle = extras.adjustAngleForDriverPosition(drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), ExtraOpModeFunctions.RobotStartPosition.STRAIGHT);
            drive.odo.update();
            //adjustedAngle = drive.odo.getHeading() + (Math.PI / 2);
            adjustedAngle = drive.odo.getHeading() + previousOrientation;
            stickSideways = gamepad1.left_stick_x * speedMultiplier;
            stickForward = -gamepad1.left_stick_y * speedMultiplier;
            stickSidewaysRotated = (stickSideways * Math.cos(-adjustedAngle)) - (stickForward * Math.sin(-adjustedAngle));
            stickForwardRotated = (stickSideways * Math.sin(-adjustedAngle)) + (stickForward * Math.cos(-adjustedAngle));

            switch (0) {
                case 0:
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    stickSidewaysRotated,
                                    stickForwardRotated
                            ),
                            -gamepad1.right_stick_x
                    ));
                    break;
                case 1:
                    // Reading joystick imputs
                    stickRotation = gamepad1.right_stick_x;

                    frontLeftPower = stickForwardRotated - stickSidewaysRotated + stickRotation;
                    frontRightPower = stickForwardRotated + stickSidewaysRotated - stickRotation;
                    backLeftPower = stickForwardRotated + stickSidewaysRotated + stickRotation;
                    backRightPower = stickForwardRotated - stickSidewaysRotated - stickRotation;

                    ArrayList<Double> motorList = new ArrayList<>(Arrays.asList(frontLeftPower, frontRightPower, backLeftPower, backRightPower));

                    double highest = 0;

                    // Find the highest value out of the 4 motor values
                    for (double n : motorList)
                    {
                        if (Math.abs(n) > highest) {
                            highest = Math.abs(n);
                        }
                    }

                    if (highest < 1)
                    {
                        highest = 1;
                    }

                    // Normalize each of the values
                    frontLeftPower = (frontLeftPower / highest) * (speedMultiplier / 10);
                    frontRightPower = (frontRightPower / highest) * (speedMultiplier / 10);
                    backLeftPower = (backLeftPower / highest) * (speedMultiplier / 10);
                    backRightPower = (backRightPower / highest) * (speedMultiplier / 10);

                    drive.leftFront.setPower(frontLeftPower);
                    drive.rightFront.setPower(frontRightPower);
                    drive.leftBack.setPower(backLeftPower);
                    drive.rightBack.setPower(backRightPower);
                    break;
            }


            //extend lift
            if (gamepad2.right_trigger > 0 && !gamepad2.start && !gamepad2.back)
            {
                extras.liftRetract();

            }
            //extend lift
            if (gamepad2.right_bumper && !gamepad2.back)
            {
                extras.liftExtend();
                extras.elevatorDown();
                extras.armRetract();
            }

            if(gamepad2.left_bumper) {
                extras.liftRetractManual();
                manualLiftStop = true;
            }
            else if(gamepad2.left_trigger > 0) {
                extras.liftExtendManual();
                manualLiftStop = true;
            }
            else
            {
                if(manualLiftStop == true)
                {
                extras.liftStop();
                manualLiftStop = false;
                }
            }


            /*
            slope = -elevMultMin / elevHeightMax;
            elevatorEncoderCounts = (extras.elevator.getCurrentPosition());
            elevMult = slope * elevatorEncoderCounts + 1;
            leftBackEncoderCounts = (drive.leftBack.getCurrentPosition());
            leftFrontEnocoderCounts = (drive.leftFront.getCurrentPosition());
            rightBackEncoderCounts = (drive.rightBack.getCurrentPosition());
            rightFrontEncoderCounts = (drive.rightFront.getCurrentPosition());


            // MANUAL ELEVATOR CONTROL- gamepad 2
            // stop if the limit switch is pressed

            float elevatorStick = gamepad2.left_stick_y;
            if(extras.elevatorLimit.isPressed())
            {
                extras.elevator.setPower(0);
                elevatorStopped = true;

                // it's OK to move up if the limit switch is pressed
                if(elevatorStick < 0)
                {
                    extras.elevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    extras.elevator.setPower(-elevatorStick);
                    elevatorStopped = false;
                }
                else if (gamepad2.right_stick_y < 0)
                {
                    extras.elevatorHigh();
                }

            }
            // don't go above the max height
            else if((extras.elevator.getCurrentPosition() > elevHeightMax) && (elevatorStick < 0))
            {
                extras.elevator.setPower(0);
                elevatorStopped = true;

                int elevPos1 = extras.elevator.getCurrentPosition();
                extras.elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                extras.elevator.setTargetPosition(elevPos1);
                extras.elevator.setPower(1.0);
            }

            else
            {
                // If stick is not moved, only set power to 0 once
                if((elevatorStick == 0) && !elevatorStopped)
                {
                    extras.elevator.setPower(0);
                    elevatorStopped = true;

                    int elevPos1 = extras.elevator.getCurrentPosition();
                    extras.elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extras.elevator.setTargetPosition(elevPos1);
                    extras.elevator.setPower(1.0);

                }
                else if (elevatorStick != 0)
                {
                    extras.elevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    extras.elevator.setPower(-elevatorStick);
                    elevatorStopped = false;
                }
            }
            */
            // Elevator to top with right stick up
            /*
            if (gamepad2.right_stick_y < 0)
            {
                extras.elevatorHigh();
            }

            // Elevator to bottom with right sitck down
            if ((gamepad2.right_stick_y > 0))
            {
                extras.elevatorGround();
            }
            */
            drive.updatePoseEstimate();

            extras.elevatorMonitor();


            if (gamepad2.dpad_up)
            {
                gp2_dpad_up_pressed = true;
            }
            else if(!gamepad2.dpad_up && gp2_dpad_up_pressed)
            {
                gp2_dpad_up_pressed = false;
                extras.specimenPickupState = ExtraOpModeFunctions.SpecimenPickupStates.PICKUP;
            }
            extras.specimenPickupStateMachine();;

            if (gamepad2.dpad_down)
            {
                extras.armVertical();
                extras.elevatorDown();
                extras.tailUp();
                telemetry.addData("Dpad down", extras.elevator.getTargetPosition());
                telemetry.addData("Dpad down", extras.elevator.getPower());
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

                //drive.imu.initialize(imuParameters);
                drive.odo.resetPosAndIMU();
                previousOrientation = drive.odo.getHeading()+ PI/2;
                extras.saveAutoStartRotation(previousOrientation);
                sleep(500);
            }

            // Init Elevator
            if ((gamepad2.back) && (gamepad2.y))
            {
                gp2_by_pressed = true;
            }
            else if (!gamepad2.y && gp2_by_pressed)
            {
                extras.initElevator();
                gp2_by_pressed = false;
            }

            // Init Arm
            if ((gamepad1.back) && (gamepad1.x))
            {
                gp1_bx_pressed = true;
            }
            else if (!gamepad1.x && gp1_bx_pressed)
            {
                extras.initArm();
                gp1_bx_pressed = false;
            }
            if ((gamepad2.back) && (gamepad2.x))
            {
                gp2_bx_pressed = true;
            }
            else if (!gamepad2.x && gp2_bx_pressed)
            {
                extras.initArm();
                gp2_bx_pressed = false;
            }

            if (gamepad1.left_trigger > 0)
            {
                extras.intakeOut();
                intakeOn = true;
            }
            else if (gamepad1.right_trigger > 0)
            {
                extras.intakeIn();
                intakeOn = true;
            }
            else if (extras.basketDeliveryState == ExtraOpModeFunctions.BasketDelivery.IDLE)
            {
                extras.intakeOff();
            }

            /*
            if (gamepad2.left_bumper)
            {
                gp2_left_bumper_pressed = true;
            }
            else if (!gamepad2.left_bumper && gp2_left_bumper_pressed) {
                gp2_left_bumper_pressed = false;
                extras.tailUp();
            }

            if (gamepad2.left_trigger > 0)
            {
                gp2_left_trigger_pressed = true;
            }
            else if ((gamepad2.left_trigger == 0) && gp2_left_trigger_pressed) {
                gp2_left_trigger_pressed = false;
                extras.tailDown();
            }
            */
            if(gamepad2.a && !gamepad2.back)
            {
                extras.armBack();
            }

            if(gamepad2.b && !gamepad2.back)
            {
                extras.armPark();
            }



            extras.basketDeliveryStateMachine();

            if ((!gamepad2.back) && (gamepad2.y))
            {
                extras.armVertical();
            }

            if ((!gamepad2.back) && (gamepad2.x))
            {
                extras.armHang2();
            }


            extras.collectStateMachine();

            if(gamepad1.a)
            {
                gp1_a_pressed = true;
            }
            else if (!gamepad1.a && gp1_a_pressed) {
                gp1_a_pressed = false;
                if (extras.armPosition == ExtraOpModeFunctions.ArmPosition.HORIZONTAL)
                {
                    extras.armExtend();
                }
                else
                {
                    extras.armHorizontal();
                }
            }

            // Arm to Zero
            if ( gamepad1.y )
            {
                gp1_y_pressed = true;
            }
            else if (!gamepad1.y && gp1_y_pressed)
            {
                gp1_y_pressed = false;
                if (extras.armPosition == ExtraOpModeFunctions.ArmPosition.HORIZONTAL)
                {
                    extras.armBack();
                }
                else
                {
                    extras.armHorizontal();
                }

            }
            //telemetry.addData("x", drive.pose.position.x);
            //telemetry.addData("y", drive.pose.position.y);
            //telemetry.addData("heading", drive.pose.heading.real);

            telemetry.addData("ODO heading", drive.odo.getHeading());
            telemetry.addData("ODO adjusted angle", adjustedAngle);
            //telemetry.addData("IMU angle", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            //Pose2D pos = drive.odo.getPosition();
            //String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            //telemetry.addData("Position", data);

            //telemetry.addLine();

            telemetry.addData("Lift Counts: ", extras.lift.getCurrentPosition());
            telemetry.addData("Lift Current: ", extras.lift.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Arm Encoder Counts: ", extras.arm.getCurrentPosition());
            telemetry.addData("Arm Target Counts: ", extras.arm.getTargetPosition());
            telemetry.addData("Arm Current: ", armCurrent);
            telemetry.addData("Arm Max Current: ", armMaxCurrent);
            telemetry.addData("Elevator Encoder Counts: ", extras.elevator.getCurrentPosition());
            telemetry.addData("Elevator Target Counts: ", extras.elevator.getTargetPosition());
            telemetry.addData("Elevator Current: ", elevatorCurrent);
            telemetry.addData("Elevator Max Current: ", elevatorMaxCurrent);
            telemetry.addData("Elevator limit: ", extras.elevatorLimit.isPressed());
            telemetry.addData("Elevator Resets: ", extras.elevatorResetCounter);

            telemetry.addData("Elapsed time: ", getRuntime());

            telemetry.update();
        }
    }
}