package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Disabled
@Autonomous(group = "a")

public class BasicAuto extends LinearOpMode
{
    @Override

    public void runOpMode() throws InterruptedException
    {
        double initialRotation = 270;
        Pose2d initPose = new Pose2d(0,0,Math.toRadians(initialRotation));
        Pose2d toSubmursible = new Pose2d(-12,30,Math.toRadians(270));
        Pose2d backUpFromSubmursible = new Pose2d(20,26,Math.toRadians(270));
        Pose2d lineUpForSweep = new Pose2d(30,50,Math.toRadians(270));
        Pose2d slideOver1 = new Pose2d(36,53,Math.toRadians(270));
        Pose2d sweep = new Pose2d(35,10,Math.toRadians(270));
        Pose2d backTo1 = new Pose2d(35,52,Math.toRadians(270));
        Pose2d slideOver2 = new Pose2d(47,52,Math.toRadians(270));
        Pose2d sweep2 = new Pose2d(47,10,Math.toRadians(270));
        Pose2d park = new Pose2d(-20,-2,Math.toRadians(265));


        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);

        telemetry.addLine("Initialized");
        //telemetry.addData("x", drive.pose.position.x);
        //telemetry.addData("y", drive.pose.position.y);
        //telemetry.addData("heading", drive.pose.heading);
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive())
        {
            safeWaitSeconds(0.01);
        }

        // launch pre-loaded artifacts
        Action DriveToNearSubmursibleAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(toSubmursible.position, toSubmursible.heading, new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-50,50))
                .build();

        safeWaitSeconds(0.25);

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(backUpFromSubmursible.position, backUpFromSubmursible.heading, new TranslationalVelConstraint(50.0))
                        //.strafeToLinearHeading(lineUpForSweep.position, lineUpForSweep.heading, new TranslationalVelConstraint(50.0))
                        .strafeToLinearHeading(slideOver1.position, slideOver1.heading, new TranslationalVelConstraint(50.0))
                        .strafeToLinearHeading(sweep.position, sweep.heading, new TranslationalVelConstraint(50.0))
                        .strafeToLinearHeading(backTo1.position, backTo1.heading, new TranslationalVelConstraint(50.0))
                        .strafeToLinearHeading(slideOver2.position, slideOver2.heading, new TranslationalVelConstraint(50.0))
                        .strafeToLinearHeading(sweep2.position, sweep2.heading, new TranslationalVelConstraint(50.0))
                        //.strafeToLinearHeading(backTo2.position, backTo2.heading, new TranslationalVelConstraint(50.0))
                        //.strafeToLinearHeading(slideOver3.position, slideOver3.heading, new TranslationalVelConstraint(50.0))
                        //.strafeToLinearHeading(sweep3.position, sweep3.heading, new TranslationalVelConstraint(50.0))
                        .build());


        safeWaitSeconds(0.5);


        Action parkAction = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(park.position, park.heading, new TranslationalVelConstraint(50.0))
                // move to submersible
                .build();



        safeWaitSeconds(5);


        // Save the ending location
        //extras.saveAutoStartRotation(drive.odo.getHeading()+ initialRotation - PI/2);
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

