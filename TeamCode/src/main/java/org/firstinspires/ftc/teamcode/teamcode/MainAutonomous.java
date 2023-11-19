package org.firstinspires.ftc.teamcode.teamcode;

import static org.firstinspires.ftc.teamcode.teamcode.CatHW_Vision.UltimateGoalPipeline.conePosition.LEFT;
import static org.firstinspires.ftc.teamcode.teamcode.CatHW_Vision.UltimateGoalPipeline.conePosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.teamcode.CatHW_Vision.UltimateGoalPipeline.conePosition.NONE;
import static org.firstinspires.ftc.teamcode.teamcode.CatHW_Vision.UltimateGoalPipeline.conePosition.RIGHT;

import android.widget.Switch;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcode.drive.SampleMecanumDrive;


/**
 * MainAutonomous.java
 *
 *
 * A Linear OpMode class to be an autonomous method for both Blue & Red alliance sides where we pick
 * which side of the alliance bridge we start off at with gamepad1 as well as selecting alliance
 * color and whether we park under the alliance bridge closer or further from the game field wall.
 * Also will detect the position and deliver the skystone using machine vision and move the
 * foundation.
 *
 * Mec_Odo_AutonomousLevel6_Statey is written to use machine vision and SkyStone delivery to our
 * autonomous route with the help intake jaws that suck in a stone at any orientation using a
 * "touch it-own it" approach.  A servo and two motors make up TC-73/Bucky's arm and stack stones as
 * well as our team marker.

 * This autonomous is used for our State Championship(February 7-8, 2020).
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back.
 */

@Autonomous(name="MainAutonomous", group="CatAuto")

public class MainAutonomous extends LinearOpMode {

    /* Declare OpMode members. */

    CatHW_Async robot = new CatHW_Async();    // All the hardware classes init here.
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;

    private ElapsedTime runningTime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        /*
        Initialize the setDrivePowers system variables.  The init() methods of our hardware class
        does all the work:
         */
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        robot.init(hardwareMap, this);




        /*
        Init Delay Option Select:
         */

        // After init is pushed but before Start we can change the delay using dpad up/down //
        delayTimer.reset();
        // Runs a loop to change certain settings while we wait to start
        while (!opModeIsActive() && !isStopRequested()) {
            if (this.isStopRequested()) {
                // Leave the loop if STOP is pressed
                return;
            }
            if (gamepad1.dpad_up && (delayTimer.seconds() > 0.8)) {
                // Increases the amount of time we wait
                timeDelay += 1;
                delayTimer.reset();
            }
            if (gamepad1.dpad_down && (delayTimer.seconds() > 0.8)) {
                // Decreases the amount of time we wait
                if (timeDelay > 0) {
                    // No such thing as negative time
                    timeDelay -= 1;
                }
                delayTimer.reset();
            }
            if (((gamepad1.x) && delayTimer.seconds() > 0.5)) {
                // Changes Alliance Sides
                if (robot.isRedAlliance && !robot.isLeftAlliance) {

                    robot.isRedAlliance = true;
                    robot.isLeftAlliance = true;

                } else if (robot.isRedAlliance && robot.isLeftAlliance) {

                    robot.isLeftAlliance = true;
                    robot.isRedAlliance = false;
                } else if (!robot.isRedAlliance && robot.isLeftAlliance) {

                    robot.isLeftAlliance = false;
                    robot.isRedAlliance = false;
                } else if (!robot.isRedAlliance && !robot.isLeftAlliance) {

                    robot.isLeftAlliance = false;
                    robot.isRedAlliance = true;
                }
                delayTimer.reset();
            }





            /*
             * LED code:
             */
           /* if (CatHW_Async.isRedAlliance && !CatHW_Async.isLeftAlliance) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if(CatHW_Async.isRedAlliance && CatHW_Async.isLeftAlliance) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
            }else if(!CatHW_Async.isRedAlliance && !CatHW_Async.isLeftAlliance) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }else if(!CatHW_Async.isRedAlliance && CatHW_Async.isLeftAlliance){
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
            }*/



            /*
             * Telemetry while waiting for PLAY:
             */
            //telemetry.addData("Pos","%.3f %.3f %.3f",robot.drive.realSense.getXPos(),robot.drive.realSense.getYPos(), robot.drive.realSense.getRotation());
            dashboardTelemetry.addData("Analysis Left Red", robot.eyes.pipeline.avgLeftRed);
            dashboardTelemetry.addData("Analysis Middle Red", robot.eyes.pipeline.avgMiddleRed);
            dashboardTelemetry.addData("Analysis Right Red", robot.eyes.pipeline.avgRightRed);
            dashboardTelemetry.addData("Analysis Left Blue", robot.eyes.pipeline.avgLeftBlue);
            dashboardTelemetry.addData("Analysis Middle Blue", robot.eyes.pipeline.avgMiddleBlue);
            dashboardTelemetry.addData("Analysis Right Blue", robot.eyes.pipeline.avgRightBlue);
            //dashboardTelemetry.addData("Position", robot.eyes.pipeline.avgValue);
            dashboardTelemetry.addData("Position", robot.eyes.getConePos());
            telemetry.addData("Position", robot.eyes.getConePos());
            //telemetry.addData("POS ","Is Left:%s", robot.isLeftAlliance);
            if(robot.isLeftAlliance && robot.isRedAlliance){
                telemetry.addData("Alliance","Red, Left");
            }else if(!robot.isLeftAlliance && robot.isRedAlliance){
                telemetry.addData("Alliance","Red, Right");
            }else if(robot.isLeftAlliance && !robot.isRedAlliance){
                telemetry.addData("Alliance","Blue, Left");
            }else if(!robot.isLeftAlliance && !robot.isRedAlliance){
                telemetry.addData("Alliance","Blue, Right");
            }
            //telemetry.addData("Distance", robot.jaws.intakeDistance.getDistance(DistanceUnit.INCH));

            dashboardTelemetry.update();
            telemetry.update();


            /*
             * We don't need a "waitForStart()" since we've been running our own
             * loop all this time so that we can make some changes.
             */


        }



        /*
         * Runs after hit start:
         * DO STUFF FOR the OPMODE!!!
         */


        blueLeft();



        if(isStopRequested()) return;




    }
    public void redLeft(){
        CatHW_Vision.UltimateGoalPipeline.conePosition conePos = robot.eyes.getConePos();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        SampleMecanumDrive drive = robot.drive.drive;
        drive.setPoseEstimate(startPose);
        Trajectory leftPixel = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(18,5))
                .build();
        Trajectory middlePixel = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(23,-2))
                .build();
        Trajectory rightPixel = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(28,0,Math.toRadians(-85)))
                .build();
        Trajectory moveRightPixel = drive.trajectoryBuilder(rightPixel.end(),Math.toRadians(-85))
                .lineToConstantHeading(new Vector2d(15, 5))
                .build();
        Trajectory moveLeftPixel = drive.trajectoryBuilder(leftPixel.end())
                .lineToConstantHeading(new Vector2d(15, 5))
                .build();
        Trajectory moveMiddlePixel = drive.trajectoryBuilder(middlePixel.end())
                .lineToConstantHeading(new Vector2d(15, 5))
                .build();
        Trajectory getReadyLeft = drive.trajectoryBuilder(moveLeftPixel.end(),-85)
                .lineToLinearHeading(new Pose2d(28,-5,Math.toRadians(80)))
                .build();
        Trajectory getReadyMiddle = drive.trajectoryBuilder(moveMiddlePixel.end(),-85)
                .lineToLinearHeading(new Pose2d(28,-5,Math.toRadians(80)))
                .build();
        Trajectory getReadyRight = drive.trajectoryBuilder(moveRightPixel.end(),-85)
                .lineToLinearHeading(new Pose2d(28,-5,Math.toRadians(80)))
                .build();

        Trajectory driveToOtherSideLeft = drive.trajectoryBuilder(getReadyLeft.end(),Math.toRadians(80))
                .lineToLinearHeading(new Pose2d(28, -65,Math.toRadians(85)))
                .build();

        Trajectory driveToOtherSideMiddle = drive.trajectoryBuilder(getReadyMiddle.end(),Math.toRadians(80))
                .lineToLinearHeading(new Pose2d(28, -65,Math.toRadians(85)))
                .build();

        Trajectory driveToOtherSideRight = drive.trajectoryBuilder(getReadyRight.end(),Math.toRadians(80))
                .lineToLinearHeading(new Pose2d(28, -65,Math.toRadians(85)))
                .build();

        Trajectory dropMiddlePixel = drive.trajectoryBuilder(driveToOtherSideMiddle.end(),Math.toRadians(85))
                .lineToLinearHeading(new Pose2d(25,-94,Math.toRadians(84)))
                .build();
        Trajectory dropLeftPixel = drive.trajectoryBuilder(driveToOtherSideLeft.end(),Math.toRadians(85))
                .lineToLinearHeading(new Pose2d(34,-95.5,Math.toRadians(85)))
                .build();
        Trajectory dropRightPixel = drive.trajectoryBuilder(driveToOtherSideRight.end(),Math.toRadians(85))
                .lineToLinearHeading(new Pose2d(18,-93,Math.toRadians(80)))
                .build();

        switch(conePos) {
            case NONE:
            case RIGHT:
                drive.followTrajectory(rightPixel);
                robot.jaws.setIntakePower(-.33);
                robot.robotWait(.5);
                robot.jaws.setIntakePower(0);
                drive.followTrajectory(moveRightPixel);
                drive.followTrajectory(getReadyRight);
                drive.followTrajectory(driveToOtherSideRight);
                robot.jaws.autoSetBumpHexHeight();
                drive.followTrajectory(dropRightPixel);
                robot.robotWait(2);
                robot.jaws.dispence();
                robot.robotWait(2);
                robot.jaws.zeroPos();
                robot.robotWait(.25);
                robot.jaws.autoSetHexZero();
                robot.robotWait(3);
                break;
            case MIDDLE:
                drive.followTrajectory(middlePixel);
                robot.jaws.setIntakePower(-.33);
                robot.robotWait(1);
                robot.jaws.setIntakePower(0);
                drive.followTrajectory(moveMiddlePixel);
                drive.followTrajectory(getReadyMiddle);
                drive.followTrajectory(driveToOtherSideMiddle);
                robot.jaws.autoSetBumpHexHeight();
                drive.followTrajectory(dropMiddlePixel);
                robot.robotWait(2);
                robot.jaws.dispence();
                robot.robotWait(2);
                robot.jaws.zeroPos();
                robot.robotWait(.25);
                robot.jaws.autoSetHexZero();
                robot.robotWait(3);
                break;
            case LEFT:
                drive.followTrajectory(leftPixel);
                robot.jaws.setIntakePower(-.33);
                robot.robotWait(1);
                robot.jaws.setIntakePower(0);
                drive.followTrajectory(moveLeftPixel);
                drive.followTrajectory(getReadyLeft);
                drive.followTrajectory(driveToOtherSideLeft);
                robot.jaws.autoSetBumpHexHeight();
                drive.followTrajectory(dropLeftPixel);
                robot.robotWait(2);
                robot.jaws.dispence();
                robot.robotWait(2);
                robot.jaws.zeroPos();
                robot.robotWait(.25);
                robot.jaws.autoSetHexZero();
                robot.robotWait(3);
                break;
        }



    }

    public void redRight(){
        CatHW_Vision.UltimateGoalPipeline.conePosition conePos = robot.eyes.getConePos();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        SampleMecanumDrive drive = robot.drive.drive;
        drive.setPoseEstimate(startPose);
        Trajectory moveToMiddle = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(24,-6))
                .build();
        Trajectory middlePixel = drive.trajectoryBuilder(moveToMiddle.end())
                .lineToConstantHeading(new Vector2d(31,-6))
                .build();
        Trajectory leftPixel = drive.trajectoryBuilder(moveToMiddle.end())
                .lineToLinearHeading(new Pose2d(28, -4,Math.toRadians(85)))
                .build();
        Trajectory rightPixel = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(24,-12,Math.toRadians(0)))
                .build();
        Trajectory moveMiddlePixel = drive.trajectoryBuilder(middlePixel.end())
                .lineToConstantHeading(new Vector2d(25, -6))
                .build();
        Trajectory dropLeftPixel = drive.trajectoryBuilder(leftPixel.end(),Math.toRadians(85))
                .lineToLinearHeading(new Pose2d(41, -46.5,Math.toRadians(100)))
                .build();
        Trajectory dropMiddlePixel = drive.trajectoryBuilder(moveMiddlePixel.end())
                .lineToLinearHeading(new Pose2d(44, -44,Math.toRadians(110)))
                .build();
        Trajectory dropRightPixel = drive.trajectoryBuilder(rightPixel.end())
                .lineToLinearHeading(new Pose2d(46, -43,Math.toRadians(110)))
                .build();
        Trajectory leftPark = drive.trajectoryBuilder(dropLeftPixel.end(),Math.toRadians(100))
                .lineToLinearHeading(new Pose2d(14, -45,Math.toRadians(80)))
                .build();
        Trajectory middlePark = drive.trajectoryBuilder(dropMiddlePixel.end(),Math.toRadians(110))
                .lineToLinearHeading(new Pose2d(14, -45,Math.toRadians(80)))
                .build();
        Trajectory leftParkFinal = drive.trajectoryBuilder(leftPark.end(),Math.toRadians(80))
                .lineToLinearHeading(new Pose2d(8, -47,Math.toRadians(80)))
                .build();
        Trajectory middleParkFinal = drive.trajectoryBuilder(middlePark.end(),Math.toRadians(80))
                .lineToLinearHeading(new Pose2d(8, -47,Math.toRadians(80)))
                .build();
        Trajectory rightParkFinal = drive.trajectoryBuilder(dropRightPixel.end(),Math.toRadians(110))
                .lineToLinearHeading(new Pose2d(20, -47,Math.toRadians(110)))
                .build();


        switch(conePos) {
            case NONE:
            case RIGHT:
                //drive.followTrajectory(moveToMiddle);
                drive.followTrajectory(rightPixel);
                robot.jaws.setIntakePower(-.25);
                robot.robotWait(1);
                robot.jaws.setIntakePower(0);
                //drive.followTrajectory(moveMiddlePixel);
                robot.jaws.autoSetBumpHexHeight();
                drive.followTrajectory(dropRightPixel);
                robot.robotWait(2);
                robot.jaws.dispence();
                robot.robotWait(2);
                robot.jaws.zeroPos();
                robot.robotWait(1);
                robot.jaws.autoSetHexZero();
                robot.robotWait(1);
                //drive.followTrajectory(middlePark);
                drive.followTrajectory(rightParkFinal);
                break;
            case MIDDLE:
                drive.followTrajectory(moveToMiddle);
                drive.followTrajectory(middlePixel);
                robot.jaws.setIntakePower(-.25);
                robot.robotWait(1);
                robot.jaws.setIntakePower(0);
                drive.followTrajectory(moveMiddlePixel);
                robot.jaws.autoSetBumpHexHeight();
                drive.followTrajectory(dropMiddlePixel);
                robot.robotWait(2);
                robot.jaws.dispence();
                robot.robotWait(2);
                robot.jaws.zeroPos();
                robot.robotWait(1);
                robot.jaws.autoSetHexZero();
                robot.robotWait(1);
                drive.followTrajectory(middlePark);
                drive.followTrajectory(middleParkFinal);
                break;
            case LEFT:
                drive.followTrajectory(moveToMiddle);
                drive.followTrajectory(leftPixel);
                robot.jaws.setIntakePower(-.25);
                robot.robotWait(1);
                robot.jaws.setIntakePower(0);
                robot.jaws.autoSetBumpHexHeight();
                drive.followTrajectory(dropLeftPixel);
                robot.robotWait(2);
                robot.jaws.dispence();
                robot.robotWait(2);
                robot.jaws.zeroPos();
                robot.robotWait(1);
                robot.jaws.autoSetHexZero();
                robot.robotWait(1);
                drive.followTrajectory(leftPark);
                drive.followTrajectory(leftParkFinal);
                break;
        }
    }

    public void blueLeft(){
        CatHW_Vision.UltimateGoalPipeline.conePosition conePos = robot.eyes.getConePos();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        SampleMecanumDrive drive = robot.drive.drive;
        drive.setPoseEstimate(startPose);
        Trajectory leftPixel = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(18,5))
                .build();

        Trajectory middlePixel = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(22,-3))
                .build();

        Trajectory rightPixel = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(34,2,Math.toRadians(-100)))
                .build();

        Trajectory dropLeftPixel = drive.trajectoryBuilder(leftPixel.end())
                .lineToLinearHeading(new Pose2d(32, 37,Math.toRadians(-100)))
                .build();
        Trajectory dropMiddlePixel = drive.trajectoryBuilder(middlePixel.end())
                .lineToLinearHeading(new Pose2d(44, 37,Math.toRadians(-100)))
                .build();

        Trajectory dropRightPixel = drive.trajectoryBuilder(rightPixel.end(), Math.toRadians(-100))
                .lineToLinearHeading(new Pose2d(54, 26,Math.toRadians(-120)))
                .build();

        Trajectory LeftPark = drive.trajectoryBuilder(dropLeftPixel.end(),Math.toRadians(-100))
                .lineToLinearHeading(new Pose2d(15, 37,Math.toRadians(-100)))
                .build();
        Trajectory MiddlePark = drive.trajectoryBuilder(dropMiddlePixel.end(),Math.toRadians(-100))
                .lineToLinearHeading(new Pose2d(15, 37,Math.toRadians(-100)))
                .build();
        Trajectory RightPark = drive.trajectoryBuilder(dropRightPixel.end(),Math.toRadians(-100))
                .lineToLinearHeading(new Pose2d(15, 37,Math.toRadians(-100)))
                .build();

        drive.followTrajectory(rightPixel);
        robot.jaws.setIntakePower(-.25);
        robot.robotWait(1);
        robot.jaws.setIntakePower(0);
        drive.followTrajectory(dropRightPixel);
        robot.jaws.autoSetBumpHexHeight();
        robot.robotWait(2);
        robot.jaws.dispence();
        robot.robotWait(2);
        robot.jaws.zeroPos();
        robot.robotWait(1);
        robot.jaws.autoSetHexZero();
        robot.robotWait(1);
        drive.followTrajectory(LeftPark);

        switch(conePos) {
            case NONE:
            case RIGHT:
                drive.followTrajectory(rightPixel);
                robot.jaws.setIntakePower(-.25);
                robot.robotWait(1);
                robot.jaws.setIntakePower(0);
                drive.followTrajectory(dropRightPixel);
                robot.jaws.autoSetBumpHexHeight();
                robot.robotWait(2);
                robot.jaws.dispence();
                robot.robotWait(2);
                robot.jaws.zeroPos();
                robot.robotWait(1);
                robot.jaws.autoSetHexZero();
                robot.robotWait(1);
                drive.followTrajectory(RightPark);
                break;
            case MIDDLE:
                drive.followTrajectory(middlePixel);
                robot.jaws.setIntakePower(-.25);
                robot.robotWait(1);
                robot.jaws.setIntakePower(0);
                drive.followTrajectory(dropMiddlePixel);
                robot.jaws.autoSetBumpHexHeight();
                robot.robotWait(2);
                robot.jaws.dispence();
                robot.robotWait(2);
                robot.jaws.zeroPos();
                robot.robotWait(1);
                robot.jaws.autoSetHexZero();
                robot.robotWait(1);
                drive.followTrajectory(MiddlePark);
                break;
            case LEFT:
                drive.followTrajectory(leftPixel);
                robot.jaws.setIntakePower(-.25);
                robot.robotWait(1);
                robot.jaws.setIntakePower(0);
                drive.followTrajectory(dropLeftPixel);
                robot.jaws.autoSetBumpHexHeight();
                robot.robotWait(2);
                robot.jaws.dispence();
                robot.robotWait(2);
                robot.jaws.zeroPos();
                robot.robotWait(1);
                robot.jaws.autoSetHexZero();
                robot.robotWait(1);
                drive.followTrajectory(LeftPark);
                break;


        }
    }

}
