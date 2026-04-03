package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
//@Disabled

@Autonomous(group = "a")

public class Depot18Eat extends LinearOpMode
{
    @Override

    public void runOpMode() throws InterruptedException
    {
        double initialRotation = 270;
        Pose2d initPose = new Pose2d(0,0,Math.toRadians(initialRotation));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        safeWaitSeconds(0.3);
        VisionFunctions vision = new VisionFunctions(hardwareMap, this);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);
        AutoFunctions autoFun = new AutoFunctions(this, extras, vision);

        PinpointLocalizer ppLocalizer = (PinpointLocalizer) drive.localizer;
        double ppYawInitial = 0.0;
        double ppYawFinal = 0.0;
        double chYawInitial = 0.0;
        double chYawFinal = 0.0;
        double savedAngle = 0.0;

        drive.lazyImu.get().resetYaw();

        telemetry.addLine("Initialized");
        //telemetry.addData("x", drive.pose.position.x);
        //telemetry.addData("y", drive.pose.position.y);
        //telemetry.addData("heading", drive.pose.heading);
        telemetry.update();

        VisionFunctions.ObeliskPattern obelisk;

        AprilTagPoseFtc cam;
        AprilTagPoseFtc ll;

        extras.turretHome();
        extras.setTurret(0.0);

        while (!isStopRequested() && !opModeIsActive())
        {
            autoFun.autoInitFunction();
            safeWaitSeconds(0.01);

            ppYawInitial = ppLocalizer.driver.getHeading(AngleUnit.RADIANS);
            chYawInitial = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


            telemetry.update();
        }

        // AFTER START IS PRESSED
        extras.setTurret(Math.toRadians(autoFun.redBlueT(-125)));

        Pose2d forwardRotation = new Pose2d(0,0, Math.toRadians(autoFun.redBlueT(270)));
        Pose2d backwardRotation = new Pose2d(0,0, Math.toRadians(autoFun.redBlueT(90)));
        Pose2d startPose = new Pose2d(61, autoFun.redBlueT(-37), Math.toRadians(autoFun.redBlueT(initialRotation)));
        drive.localizer.setPose(startPose);
        Pose2d toInitialLaunchPosition = new Pose2d(12,autoFun.redBlueT(-24),Math.toRadians(autoFun.redBlueT(initialRotation))); // old position(12,-17)
        Pose2d toSpike3 = new Pose2d(11,autoFun.redBlueT(-28),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d pickupSpike3 = new Pose2d(11,autoFun.redBlueT(-45),Math.toRadians(autoFun.redBlueT(initialRotation))); //-50
        Pose2d nearGate = new Pose2d(-12,autoFun.redBlueT(-57),Math.toRadians(autoFun.redBlueT(305))); //-31
        Pose2d toGate = new Pose2d(-10.3,autoFun.redBlueT(-59.4),Math.toRadians(autoFun.redBlueT(311)));
        Pose2d toSpike2 = new Pose2d(-10,autoFun.redBlueT(-27),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d pickupSpike2 = new Pose2d(-10,autoFun.redBlueT(-48),Math.toRadians(autoFun.redBlueT(initialRotation))); //-50
        Pose2d toSpike1 = new Pose2d(-35,autoFun.redBlueT(-30),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d pickupSpike1 = new Pose2d(-35,autoFun.redBlueT(-45),Math.toRadians(autoFun.redBlueT(initialRotation))); //-51
        Pose2d park = new Pose2d(-1,autoFun.redBlueT(-45),Math.toRadians(autoFun.redBlueT(initialRotation)));
        //Pose2d park2 = new Pose2d(30,autoFun.redBlueT(-10),Math.toRadians(autoFun.redBlueT(200)));
        extras.saveTeamColor(extras.teamColor);

        // turn on the LimeLight
        vision.limelight.start();

        // Turn on shooter to the expected speed
        double launcherSpeed = 1350.0;
        extras.setLauncher(launcherSpeed);
        extras.launcherSdown();

        //safeWaitSeconds(autoFun.startDelay);

        // drive off the line and shoot preload
        Action ToInitialPosition = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(new ParallelAction(
                ToInitialPosition,
                new SequentialAction(
                        new SleepAction(1.3),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD)),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.OFF)),
                        new SleepAction(0.7),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON))),
                extras.setLauncherAction(launcherSpeed)
        ));

        // pickup and launch spike 2
        Action GoToSpike2 = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(toSpike2.position, forwardRotation.heading)
                .splineToConstantHeading(pickupSpike2.position,pickupSpike2.heading)
                .build();
        Actions.runBlocking(new RaceAction(GoToSpike2,extras.checkIntakeAction()));
        //Actions.runBlocking(GoToSpike2);
        extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD);

        Action BackToLaunchSpotSpike2 = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(backwardRotation.heading)
                .splineToConstantHeading(toInitialLaunchPosition.position, Math.toRadians(0))
                .build();
        Actions.runBlocking(new ParallelAction(
                BackToLaunchSpotSpike2,
                new SequentialAction(
                        new SleepAction(1.0),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD)),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.OFF)),
                        new SleepAction(0.7),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON))),
                extras.setLauncherAction(launcherSpeed)
        ));

        // 1st gate pickup and launch
        Action ToGate1 = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(toGate,forwardRotation.heading)
                .build();
        Actions.runBlocking(new RaceAction(ToGate1,extras.checkIntakeAction()));
        //Actions.runBlocking(ToGate1);
        extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD);
        safeWaitSeconds(1.4);

        Action BackToLaunchSpotGate1 = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(backwardRotation.heading)
                .splineToLinearHeading(toInitialLaunchPosition, Math.toRadians(0))
                .build();
        Actions.runBlocking(new ParallelAction(
                BackToLaunchSpotGate1,
                new SequentialAction(
                        new SleepAction(0.25),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.OFF))),
                new SequentialAction(
                        new SleepAction(1.2),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD)),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.OFF)),
                        new SleepAction(0.7),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON))),
                extras.setLauncherAction(launcherSpeed)
        ));

        // 2nd gate pickup and launch
        Action ToGate2 = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(toGate,forwardRotation.heading)
                .build();
        Actions.runBlocking(new RaceAction(ToGate2,extras.checkIntakeAction()));
        //Actions.runBlocking(ToGate2);
        extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD);
        safeWaitSeconds(1.4);

        Action BackToLaunchSpotGate2 = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(backwardRotation.heading)
                .splineToLinearHeading(toInitialLaunchPosition, Math.toRadians(0))
                .build();
        Actions.runBlocking(new ParallelAction(
                BackToLaunchSpotGate2,
                new SequentialAction(
                        new SleepAction(0.25),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.OFF))),
                new SequentialAction(
                        new SleepAction(1.2),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD)),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.OFF)),
                        new SleepAction(0.7),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON))),
                extras.setLauncherAction(launcherSpeed)
        ));

        // 3rd gate pickup and launch
        Action ToGate3 = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(toGate,forwardRotation.heading)
                .build();
        Actions.runBlocking(new RaceAction(ToGate3,extras.checkIntakeAction()));
        //Actions.runBlocking(ToGate3);
        extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD);
        safeWaitSeconds(1.4);

        Action BackToLaunchSpotGate3 = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(backwardRotation.heading)
                .splineToLinearHeading(toInitialLaunchPosition, Math.toRadians(0))
                .build();
        Actions.runBlocking(new ParallelAction(
                BackToLaunchSpotGate3,
                new SequentialAction(
                        new SleepAction(0.25),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.OFF))),
                new SequentialAction(
                        new SleepAction(1.2),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD)),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.OFF)),
                        new SleepAction(0.7),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON))),
                extras.setLauncherAction(launcherSpeed)
        ));

        // 1st gate pickup and launch
        Action ToGate4 = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(toGate,forwardRotation.heading)
                .build();
        Actions.runBlocking(new RaceAction(ToGate4,extras.checkIntakeAction()));
        //Actions.runBlocking(ToGate1);
        extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD);
        safeWaitSeconds(1.4);

        Action BackToLaunchSpotGate4 = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(backwardRotation.heading)
                .splineToLinearHeading(toInitialLaunchPosition, Math.toRadians(0))
                .build();
        Actions.runBlocking(new ParallelAction(
                BackToLaunchSpotGate4,
                new SequentialAction(
                        new SleepAction(0.25),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.OFF))),
                new SequentialAction(
                        new SleepAction(1.2),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD)),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.OFF)),
                        new SleepAction(0.7),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON))),
                extras.setLauncherAction(launcherSpeed)
        ));

        // pickup and launch spike 3
        extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD);
        Action GoToSpike3 = drive.actionBuilder(drive.localizer.getPose())
                .splineToConstantHeading(toSpike3.position, toSpike3.heading)
                .splineToConstantHeading(pickupSpike3.position, pickupSpike3.heading)
                .build();
        Actions.runBlocking(new RaceAction(GoToSpike3,extras.checkIntakeAction()));
        //Actions.runBlocking(GoToSpike3);

        Action BackToLaunchSpotSpike3 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(new ParallelAction(
                BackToLaunchSpotSpike3,
                new SequentialAction(
                        new SleepAction(0.9),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD)),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.OFF)),
                        new SleepAction(0.7),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON))),
                extras.setLauncherAction(launcherSpeed)
        ));

        extras.setIntake(ExtraOpModeFunctions.IntakeStates.OFF);

        Action Park = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(park.position, park.heading)
                .build();
        Actions.runBlocking(Park);

        safeWaitSeconds(1);

        // turn the intake and shooter off
        extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON);
        //extras.setLauncher(0.0);
        extras.launcher1.setPower(0.0);
        extras.launcher2.setPower(0.0);

        // Save the ending location
        //extras.saveAutoStartRotation(drive.odo.getHeading()+ initialRotation - PI/2);
        //ppYawFinal = ppLocalizer.driver.getHeading(AngleUnit.RADIANS);

        chYawFinal = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        savedAngle = chYawFinal - chYawInitial;
        //extras.saveAutoStartRotation(savedAngle);
        PoseStorage.currentAngle = savedAngle;
        PoseStorage.currentPose = drive.localizer.getPose();

    }

    public void safeWaitSeconds(double time)
    {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time)
        {
            ;
        }
    }
}

