package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
            if (CatHW_Async.isRedAlliance && !CatHW_Async.isLeftAlliance) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if(CatHW_Async.isRedAlliance && CatHW_Async.isLeftAlliance) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
            }else if(!CatHW_Async.isRedAlliance && !CatHW_Async.isLeftAlliance) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }else if(!CatHW_Async.isRedAlliance && CatHW_Async.isLeftAlliance){
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
            }



            /*
             * Telemetry while waiting for PLAY:
             */
            robot.drive.updateOdo();
            telemetry.addData("Pos","%.3f %.3f %.3f",robot.drive.realSense.getXPos(),robot.drive.realSense.getYPos(), robot.drive.realSense.getRotation());
            dashboardTelemetry.addData("Analysis Red", robot.eyes.pipeline.avgRed);
            dashboardTelemetry.addData("Analysis Blue", robot.eyes.pipeline.avgBlue);
            dashboardTelemetry.addData("Analysis Green", robot.eyes.pipeline.avgGreen);
            dashboardTelemetry.addData("Position", robot.eyes.pipeline.avgValue);
            telemetry.addData("Position", robot.eyes.pipeline.avgValue);
            telemetry.addData("POS ","Is Left:%s", robot.isLeftAlliance);
            if(robot.isLeftAlliance && robot.isRedAlliance){
                telemetry.addData("Alliance","Red, Left");
            }else if(!robot.isLeftAlliance && robot.isRedAlliance){
                telemetry.addData("Alliance","Red, Right");
            }else if(robot.isLeftAlliance && !robot.isRedAlliance){
                telemetry.addData("Alliance","Blue, Left");
            }else if(!robot.isLeftAlliance && !robot.isRedAlliance){
                telemetry.addData("Alliance","Blue, Right");
            }
            telemetry.addData("Distance", robot.jaws.intakeDistance.getDistance(DistanceUnit.INCH));

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
        runningTime.reset();
        if(robot.isLeftAlliance){

            left();
        }else if(!robot.isLeftAlliance){
            right();
        }
        robot.jaws.ourThread.stop();
    }
    public void left(){

        CatHW_Vision.UltimateGoalPipeline.conePosition conePos = robot.eyes.getConePos();

        telemetry.addData("Position",conePos.toString());
        //telemetry.update();
        int stackedConeHeight = 40;

        robot.jaws.grabPos();
        robot.robotWait(.16);

        robot.jaws.setLiftMiddleFront(.7);
        robot.robotWait(.16);

        robot.drive.quickDrive(3.5,8,0,1,5);

        //robot.drive.quickDrive(11,37,40,.3,5);
        robot.drive.quickDrive(15,31,0,1,5);
        robot.drive.setTightTolerance();
        robot.drive.quickDrive(15,32.5,0,.2,5);
        robot.robotWait(.5);
        robot.jaws.setArmHeight(95);
        robot.jaws.unGrab();
        robot.robotWait(.25);


        //drives over to the stack of cones and picks one up
        robot.drive.setNormalTolerance();
        robot.drive.quickDrive(4,27,0,1,5);
        robot.jaws.setLiftBottom(.7);
        robot.jaws.setArmHeight(stackedConeHeight);
        robot.drive.quickDrive(4,53,0,1,5);

        //DELETE BELOW AFTER TESTING

        robot.robotWait(.25);
        robot.drive.setLooseTolerance();
        robot.drive.quickDrive(-4,45,-90,1,5);
        robot.drive.setTightTolerance();
        robot.drive.quickDrive(-9,45,-90,.2,5);
        ElapsedTime grabTimer = new ElapsedTime();
        while(!robot.jaws.haveCone() && (grabTimer.seconds() < 2)){
            robot.drive.setDrivePowers(.15,.15,.15,.15);
        }
        robot.jaws.grabPos();
        robot.robotWait(.5);
        robot.jaws.setArmHeight(70);
        robot.robotWait(.25);
        robot.drive.quickDrive(12,45,-90,1,5);
        robot.jaws.setLiftMiddleBack(.3);
        robot.drive.quickDrive(8,35,-90,.5,5);
        robot.drive.quickDrive(5,35,-90,.5,5);
        robot.robotWait(.2);
        robot.jaws.setLiftHeight(380,.5);
        robot.robotWait(.2);
        robot.jaws.unGrab();
        robot.robotWait(.2);
        robot.drive.quickDrive(10,45,-90,1,5);
        robot.jaws.grabPos();
        robot.robotWait(.15);
        robot.jaws.setLiftBottom(.7);
        robot.jaws.unGrab();
        robot.robotWait(1);

        robot.drive.setLooseTolerance();
        switch(conePos) {
            case NONE:
            case RIGHT:
                robot.drive.quickDrive(28, 50, -90, 1, 5);
                break;
            case MIDDLE:
                robot.drive.quickDrive(7,45,-90,1,5);
                break;
            case LEFT:
                robot.drive.quickDrive(-14, 46, -90, 1, 6);
                break;
        }


        /*while(runningTime.seconds() < 20){

            robot.jaws.setLiftHeight(stackedConeHeight,.5);
            robot.robotWait(.25);
            robot.drive.setTightTolerance();
            robot.drive.quickDrive(-22,45,-90,.2,5);

            ElapsedTime grabTimer = new ElapsedTime();
            while(!robot.jaws.haveCone() && (grabTimer.seconds() < 1)){
                robot.drive.setDrivePowers(.15,.15,.15,.15);
            }
            robot.jaws.grabPos();
            robot.robotWait(.5);
            robot.jaws.setLiftLowBack(.5);
            robot.drive.setDrivePowers(-.1,-.1,-.1,-.1);


            //drives over to drop cone onto short junction
            robot.robotWait(.5);
            robot.drive.setNormalTolerance();
            robot.drive.quickDrive(3,48,-90,.3,5);
            robot.drive.setTightTolerance();
            robot.drive.quickDrive(3,49,-180,.2,5);

            robot.robotWait(.1);
            robot.jaws.setLiftGroundJunction(.25);
            robot.jaws.unGrab();
            robot.robotWait(.25);
            robot.drive.setNormalTolerance();
            robot.drive.quickDrive(0,52,-180,.3,5);
            stackedConeHeight -= 5;
        }
        robot.jaws.setLiftBottom(.5);

        robot.drive.setLooseTolerance();
        robot.jaws.setLiftBottom(.25);
        switch(conePos) {
            case NONE:
            case RIGHT:
                robot.drive.quickDrive(38, 53, -180, .5, 5);
                break;
            case MIDDLE:
                robot.drive.quickDrive(13, 50, -180, .5, 5);
                break;
            case LEFT:
                robot.drive.quickDrive(-8, 50, -180, .5, 6);
                break;
        }
        robot.robotWait(2);*/




    }
    public void right(){
        CatHW_Vision.UltimateGoalPipeline.conePosition conePos = robot.eyes.getConePos();
        int stackedConeHeight = 40;

        //Grabs Cone and lifts the lift
        robot.jaws.grabPos();
        robot.robotWait(.16);
        robot.jaws.setLiftMiddleFront(.5);
        robot.robotWait(.1);
        robot.drive.setLooseTolerance();

        //Drives to the medium junction and drops the cone

        robot.drive.quickDrive(3.5,22,0,1,5);
        robot.drive.quickDrive(-7,30,0,1,5);
        robot.drive.setTightTolerance();
        robot.drive.quickDrive(-7,33.8,0,.25,5);
        robot.robotWait(.2);
        robot.jaws.unGrab();
        robot.robotWait(.25);
        robot.drive.quickDrive(-4,29,0,1,5);

        robot.drive.quickDrive(4,29,0,1,5);

        //drives over to the stack of cones and picks one up
        robot.drive.setLooseTolerance();
        robot.drive.quickDrive(4,40,0,1,5);
        robot.jaws.setLiftBottom(.25);
        robot.jaws.setArmHeight(stackedConeHeight);
        robot.drive.quickDrive(15,48,0,1,5);


        robot.robotWait(.25);
        robot.drive.setTightTolerance();
        robot.drive.quickDrive(18,43,90,.25,5);

        ElapsedTime grabTimer = new ElapsedTime();
        while(!robot.jaws.haveCone() && (grabTimer.seconds() < 2)){
            robot.drive.setDrivePowers(.15,.15,.15,.15);
        }
        robot.jaws.grabPos();
        robot.robotWait(.25);
        robot.jaws.setArmHeight(73);
        robot.drive.setLooseTolerance();
        robot.drive.quickDrive(2,43,90,1,5);
        robot.jaws.setLiftMiddleBack(.3);

        robot.drive.quickDrive(2,31,90,1,5);
        robot.drive.quickDrive(6,31,90,1,5);
        robot.jaws.setLiftHeight(380,1);
        robot.robotWait(.25);
        robot.jaws.unGrab();
        robot.robotWait(.5);
        robot.jaws.grabPos();
        robot.drive.setLooseTolerance();
        robot.drive.quickDrive(2,43,90,1,5);
        robot.jaws.setLiftBottom(.5);

        if(conePos == CatHW_Vision.UltimateGoalPipeline.conePosition.RIGHT) {
            robot.drive.setTightTolerance();
            robot.jaws.unGrab();
            robot.jaws.setArmHeight(stackedConeHeight - 5);
            robot.drive.quickDrive(18, 43, 90, .25, 5);

            ElapsedTime secondGrabTimer = new ElapsedTime();
            while (!robot.jaws.haveCone() && (secondGrabTimer.seconds() < 2)) {
                robot.drive.setDrivePowers(.15, .15, .15, .15);
            }
            robot.jaws.grabPos();
            robot.robotWait(.25);
            robot.jaws.setLiftLowBack(.7);
            robot.drive.quickDrive(23, 32, 90, 1, 5);
            robot.drive.quickDrive(24.5, 32, 90, 1, 5);
            robot.jaws.unGrab();
            robot.robotWait(.25);
            robot.jaws.grabPos();
            robot.drive.quickDrive(20, 32, 90, 1, 5);

            robot.jaws.setLiftBottom(1);
            robot.robotWait(1);
            robot.drive.quickDrive(23, 32, 90, 1, 5);
        }else if(conePos == CatHW_Vision.UltimateGoalPipeline.conePosition.MIDDLE){
            robot.jaws.unGrab();
            robot.jaws.setArmHeight(stackedConeHeight-5);
            robot.drive.quickDrive(18,43,90,.25,5);

            ElapsedTime thirdGrabTimer = new ElapsedTime();
            while(!robot.jaws.haveCone() && (thirdGrabTimer.seconds() < 2)){
                robot.drive.setDrivePowers(.15,.15,.15,.15);
            }
            robot.jaws.grabPos();
            robot.robotWait(.25);
            robot.jaws.setArmHeight(73);
            robot.drive.setLooseTolerance();
            robot.drive.quickDrive(2,43,90,1,5);
            robot.jaws.setLiftMiddleBack(.3);

            robot.drive.quickDrive(2,31,90,1,5);
            robot.drive.quickDrive(6,31,90,1,5);
            robot.jaws.setLiftHeight(380,1);
            robot.robotWait(.25);
            robot.jaws.unGrab();
            robot.robotWait(.5);
            robot.jaws.grabPos();
            robot.drive.setLooseTolerance();
            robot.drive.quickDrive(-1,31,90,1,5);
            robot.jaws.setLiftBottom(.5);
            robot.robotWait(1);
        }else if(conePos == CatHW_Vision.UltimateGoalPipeline.conePosition.LEFT){
            robot.jaws.unGrab();
            robot.jaws.setArmHeight(stackedConeHeight-5);
            robot.drive.quickDrive(18,43,90,.25,5);

            ElapsedTime thirdGrabTimer = new ElapsedTime();
            while(!robot.jaws.haveCone() && (thirdGrabTimer.seconds() < 2)){
                robot.drive.setDrivePowers(.15,.15,.15,.15);
            }
            robot.jaws.grabPos();
            robot.robotWait(.25);
            robot.jaws.setArmHeight(73);
            robot.drive.setLooseTolerance();
            robot.drive.quickDrive(2,43,90,1,5);
            robot.jaws.setLiftMiddleBack(.3);

            robot.drive.quickDrive(2,31,90,1,5);
            robot.drive.quickDrive(6,31,90,1,5);
            robot.jaws.setLiftHeight(380,1);
            robot.robotWait(.25);
            robot.jaws.unGrab();
            robot.robotWait(.5);
            robot.jaws.grabPos();
            robot.drive.quickDrive(6,43,90,1,5);
            robot.drive.quickDrive(-20,43,90,1,5);
            robot.jaws.setLiftBottom(.3);
            robot.robotWait(1);

        }


        /*while(runningTime.seconds() < 20){

            robot.jaws.setLiftHeight(stackedConeHeight,.5);
            robot.robotWait(.25);
            robot.drive.setTightTolerance();
            robot.drive.quickDrive(31,56,90,.2,5);

            ElapsedTime grabTimer = new ElapsedTime();
            while(!robot.jaws.haveCone() && (grabTimer.seconds() < 2)){
                robot.drive.setDrivePowers(.15,.15,.15,.15);
            }
            robot.jaws.grabPos();
            robot.robotWait(.5);
            robot.jaws.setLiftLowBack(.5);
            robot.drive.setDrivePowers(-.1,-.1,-.1,-.1);


            //drives over to drop cone onto short junction
            robot.robotWait(.5);
            robot.drive.setNormalTolerance();
            robot.drive.quickDrive(25,55,90,.3,5);
            robot.drive.quickDrive(25,55,180,.25,5);
            robot.drive.setTightTolerance();
            robot.drive.quickDrive(25,48,180,.2,5);
            robot.robotWait(.1);
            robot.jaws.setLiftGroundJunction(.25);
            robot.jaws.unGrab();
            robot.robotWait(.25);
            robot.drive.setNormalTolerance();
            robot.drive.quickDrive(28,55,180,.3,5);
            stackedConeHeight -= 20;
        }
        robot.jaws.setLiftBottom(.5);

        robot.drive.setLooseTolerance();
        switch(conePos) {
            case NONE:
                robot.drive.quickDrive(37, 55, 180, 1, 5);
                break;
            case RIGHT:
                robot.drive.quickDrive(37, 55, 180, 1, 5);
                break;
            case MIDDLE:
                robot.drive.quickDrive(12, 55, 180, 1, 5);
                break;
            case LEFT:
                robot.drive.quickDrive(-12, 55, 180, 1, 6);
                break;
        }
        robot.robotWait(.25);*/


    }
}
