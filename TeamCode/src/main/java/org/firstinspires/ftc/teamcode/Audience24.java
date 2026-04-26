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

public class Audience24 extends LinearOpMode
{
    @Override

    public void runOpMode() throws InterruptedException
    {
        double initialRotation = 270;
        Pose2d initPose = new Pose2d(0,0,Math.toRadians(initialRotation));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        VisionFunctions vision = new VisionFunctions(hardwareMap, this, VisionFunctions.LLVisionType.ARTIFACT);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);
        AutoFunctions autoFun = new AutoFunctions(this, extras);

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

            telemetry.addData("ppYaw r: ", ppYawInitial);
            telemetry.addData("imuYaw r: ", chYawInitial);
            telemetry.addData("ppYaw d: ", Math.toDegrees(ppYawInitial));
            telemetry.addData("imuYaw d: ", Math.toDegrees(chYawInitial));
            telemetry.addData("savedAngle d: ", Math.toDegrees(savedAngle));

            telemetry.update();
        }

        // AFTER START IS PRESSED
        double turretAngle = Math.toRadians(autoFun.redBlueT(-115));
        extras.setTurret(turretAngle);

        Pose2d forwardRotation = new Pose2d(0,0, Math.toRadians(autoFun.redBlueT(270)));
        Pose2d backwardRotation = new Pose2d(0,0, Math.toRadians(autoFun.redBlueT(90)));
        Pose2d startPose = new Pose2d(-62, autoFun.redBlueT(-13.5), Math.toRadians(autoFun.redBlueT(initialRotation)));
        drive.localizer.setPose(startPose);
        Pose2d toInitialLaunchPosition = new Pose2d(-50,autoFun.redBlueT(-14),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d toSpike1 = new Pose2d(-38,autoFun.redBlueT(-29),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d pickupSpike1 = new Pose2d(-36,autoFun.redBlueT(-50),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d toCorner = new Pose2d(-62,autoFun.redBlueT(-58),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d pickupCorner = new Pose2d(-62,autoFun.redBlueT(-60),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d tooffCorner = new Pose2d(-38,autoFun.redBlueT(-29),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d pickupoffCorner = new Pose2d(-38,autoFun.redBlueT(-60),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d toCorner2 = new Pose2d(-62,autoFun.redBlueT(-63),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d toSpike2 = new Pose2d(-42,autoFun.redBlueT(-29),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d pickupSpike2 = new Pose2d(-42,autoFun.redBlueT(-62),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d pickupCorner2 = new Pose2d(-40,autoFun.redBlueT(-55),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d AngleCorner = new Pose2d(-40,autoFun.redBlueT(-55),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d toParkPosition = new Pose2d(-50,autoFun.redBlueT(-27),Math.toRadians(autoFun.redBlueT(initialRotation)));
        Pose2d backToLaunchZone = new Pose2d(autoFun.redBlueT(0),0,Math.toRadians(270));

        extras.saveTeamColor(extras.teamColor);

        safeWaitSeconds(autoFun.startDelay);
        // turn on the LimeLight
        vision.limelight.start();

        // Turn on shooter to the expected speed
        double launcherSpeed = 1850.0;
        extras.setLauncher(launcherSpeed);
        extras.launcherSup();

        // drive off the line and rotate towards the depot
        Action ToLaunchPosition = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        // launch ball sequence
        Actions.runBlocking(new ParallelAction(
                ToLaunchPosition,
                new SequentialAction(
                        new SleepAction(1.6),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD)),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.OFF)),
                        new SleepAction(0.5),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON)),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.OFF))
                ),
                extras.setLauncherAction(launcherSpeed, turretAngle)
        ));

        // pickup and launch spike 1
        extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD);
        Action GoToSpike1 = drive.actionBuilder(drive.localizer.getPose())
                .splineToConstantHeading(toSpike1.position, forwardRotation.heading)
                .splineToConstantHeading(pickupSpike1.position,forwardRotation.heading)
                .build();
        Actions.runBlocking(GoToSpike1);
        extras.setIntake(ExtraOpModeFunctions.IntakeStates.OFF);

        Action ToLaunchPosition2 = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(backwardRotation.heading)
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(new ParallelAction(
                ToLaunchPosition2,
                new SequentialAction(
                        new SleepAction(1.1),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD)),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.OFF)),
                        new SleepAction(0.5),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON))),
                extras.setLauncherAction(launcherSpeed, turretAngle)
        ));

        //go to corner 1 (corner)
        // pickup and launch Corner 1
        Action GoToCorner = drive.actionBuilder(drive.localizer.getPose())
                //.strafeToLinearHeading(toCorner.position, forwardRotation.heading)
                .splineToLinearHeading(pickupCorner, forwardRotation.heading)
                .build();
        //Actions.runBlocking(GoToCorner);
        Actions.runBlocking(new RaceAction(GoToCorner,extras.checkIntakeAction()));

        Action ToLaunchPosition3 = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(backwardRotation.heading)
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(new ParallelAction(
                ToLaunchPosition3,
                new SequentialAction(
                        new SleepAction(0.2),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.OFF)),
                        new SleepAction(1.1),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD)),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.OFF)),
                        new SleepAction(0.5),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON))),
                extras.setLauncherAction(launcherSpeed, turretAngle)
        ));

        //go to corner 5 (off corner)
        // pickup and launch Corner second time
        Action GoToCorner2 = drive.actionBuilder(drive.localizer.getPose())
                //.strafeToLinearHeading(toCorner.position, forwardRotation.heading)
                .splineToLinearHeading(pickupCorner, forwardRotation.heading)
                .build();

        //Actions.runBlocking(GoToCorner2);
        Actions.runBlocking(new RaceAction(GoToCorner2,extras.checkIntakeAction()));
        //extras.intakeOff();

        Action ToLaunchPosition4 = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(backwardRotation.heading)
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(new ParallelAction(
                ToLaunchPosition4,
                new SequentialAction(
                        new SleepAction(0.2),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.OFF)),
                        new SleepAction(1.1),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD)),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.OFF)),
                        new SleepAction(0.5),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON))),
                extras.setLauncherAction(launcherSpeed, turretAngle)
        ));

        //go to corner 3 (corner)
        // pickup and launch Corner third time
        Action GoToCorner3 = drive.actionBuilder(drive.localizer.getPose())
                //.strafeToLinearHeading(toCorner.position, forwardRotation.heading)
                .splineToLinearHeading(pickupCorner, forwardRotation.heading)
                .build();

        //Actions.runBlocking(GoToCorner3);
        Actions.runBlocking(new RaceAction(GoToCorner3,extras.checkIntakeAction()));
        //extras.intakeOff();

        Action ToLaunchPosition5 = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(backwardRotation.heading)
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(new ParallelAction(
                ToLaunchPosition5,
                new SequentialAction(
                        new SleepAction(0.2),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.OFF)),
                        new SleepAction(1.1),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD)),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.OFF)),
                        new SleepAction(0.5),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON))),
                extras.setLauncherAction(launcherSpeed, turretAngle)
        ));

        //go to corner 4 (off corner)
        extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD);
        Action GoToCorner4 = drive.actionBuilder(drive.localizer.getPose())
                //.strafeToLinearHeading(toCorner.position, forwardRotation.heading)
                .splineToLinearHeading(pickupCorner, forwardRotation.heading)
                .build();
        //Actions.runBlocking(GoToCorner);
        Actions.runBlocking(new RaceAction(GoToCorner4,extras.checkIntakeAction()));

        Action ToLaunchPosition6 = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(backwardRotation.heading)
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(new ParallelAction(
                ToLaunchPosition6,
                new SequentialAction(
                        new SleepAction(0.2),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.OFF)),
                        new SleepAction(1.1),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD)),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.OFF)),
                        new SleepAction(0.5),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON))),
                extras.setLauncherAction(launcherSpeed, turretAngle)
        ));

        //go to corner 5 (corner)
        extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD);
        Action GoToCorner5 = drive.actionBuilder(drive.localizer.getPose())
                //.strafeToLinearHeading(toCorner.position, forwardRotation.heading)
                .splineToLinearHeading(pickupCorner, forwardRotation.heading)
                .build();
        //Actions.runBlocking(GoToCorner);
        Actions.runBlocking(new RaceAction(GoToCorner5,extras.checkIntakeAction()));

        Action ToLaunchPosition7 = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(backwardRotation.heading)
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(new ParallelAction(
                ToLaunchPosition7,
                new SequentialAction(
                        new SleepAction(0.2),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.OFF)),
                        new SleepAction(1.1),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD)),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.OFF)),
                        new SleepAction(0.5),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON))),
                extras.setLauncherAction(launcherSpeed, turretAngle)
        ));
        // to corner 6 (off corner)
        extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD);
        Action GoToCorner6 = drive.actionBuilder(drive.localizer.getPose())
                //.strafeToLinearHeading(toCorner.position, forwardRotation.heading)
                .splineToLinearHeading(pickupCorner, forwardRotation.heading)
                .build();
        //Actions.runBlocking(GoToCorner);
        Actions.runBlocking(new RaceAction(GoToCorner6,extras.checkIntakeAction()));

        Action ToLaunchPosition8 = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(backwardRotation.heading)
                .strafeToLinearHeading(toInitialLaunchPosition.position, toInitialLaunchPosition.heading)
                .build();
        Actions.runBlocking(new ParallelAction(
                ToLaunchPosition8,
                new SequentialAction(
                        new SleepAction(0.2),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.OFF)),
                        new SleepAction(1.1),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD)),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.OFF)),
                        new SleepAction(0.5),
                        new InstantAction(() -> extras.stopLauncher()),
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON))),
                extras.setLauncherAction(launcherSpeed, turretAngle)
        ));
/*
        // park in corner while trying to pick up balls
        extras.setIntake(ExtraOpModeFunctions.IntakeStates.FORWARD);
        Action GoToCorner7 = drive.actionBuilder(drive.localizer.getPose())
                //.strafeToLinearHeading(toCorner.position, forwardRotation.heading)
                .splineToLinearHeading(pickupCorner, forwardRotation.heading)
                .build();
        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON)),
                        new RaceAction(GoToCorner7,extras.checkIntakeAction()),
                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.OFF)),
                        new InstantAction(() -> extras.stopLauncher())),
                extras.setLauncherAction(launcherSpeed, turretAngle)
        ));

*/
        // drive off the line
        Action ToPark = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toParkPosition.position, toParkPosition.heading)
                .build();
        //Actions.runBlocking(ToPark);
        Actions.runBlocking
                (
                        new ParallelAction
                                (
                                        //extras.setTurretAction(Math.toRadians(0.0)),
                                        new RaceAction(ToPark,extras.storePositionAction(drive, chYawInitial)),
                                        new InstantAction(() -> extras.stopLauncher()),
                                        new InstantAction(() -> extras.setBallStop(ExtraOpModeFunctions.BallStopStates.ON)),
                                        new InstantAction(() -> extras.setIntake(ExtraOpModeFunctions.IntakeStates.OFF)),
                                        extras.setLauncherAction(launcherSpeed, turretAngle)
                                )
                );




        // turn the intake and shooter off

        //extras.intakeOff();
        //extras.ballStopOn();
        //extras.setLauncher(0.0);

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
