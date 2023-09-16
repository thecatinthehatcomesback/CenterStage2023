package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

/**
 * MainTeleOp.java
 *
 *
 * A Linear opMode class that is used as our TeleOp method for the driver controlled period.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
@TeleOp(name = "MainTeleOp", group = "CatTeleOp")
public class MainTeleOp extends LinearOpMode
{
    /* Declare OpMode members. */
    private ElapsedTime elapsedGameTime = new ElapsedTime();


    /* Declare OpMode members. */
    CatHW_Async robot;  // Use our new mecanum async hardware


    /* Constructor */
    public MainTeleOp() {
        robot = new CatHW_Async();
    }



    @Override
    public void runOpMode() throws InterruptedException {

        // Informs driver the robot is trying to init
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();



        // Initialize the hardware
        robot.init(hardwareMap, this);

        // Finished!  Now tell the driver...
        telemetry.addData("Status", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
         ElapsedTime delayTimer = new ElapsedTime();

        while(!opModeIsActive() && !isStopRequested()){
            if (((gamepad1.x) && delayTimer.seconds() > 0.8)) {
                delayTimer.reset();

                // Changes Alliance Sides
                CatHW_Async.isRedAlliance = !CatHW_Async.isRedAlliance;
            }
            telemetry.addData("Alliance","%s",CatHW_Async.isRedAlliance?"Red":"Blue");
            telemetry.update();

            if (CatHW_Async.isRedAlliance) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }


        }



        // Go! (Presses PLAY)
        elapsedGameTime.time(TimeUnit.SECONDS);
        elapsedGameTime.reset();
        double driveSpeed;
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;
        double SF;
        boolean endGame = false;
        boolean under10Sec = false;
        boolean turningMode = false;



        ElapsedTime buttontime = new ElapsedTime();
        buttontime.reset();



        // Run infinitely until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //--------------------------------------------------------------------------------------
            // Driver 1 Controls:
            //--------------------------------------------------------------------------------------

            if((elapsedGameTime.time() >= 80) && (elapsedGameTime.time() <= 82)){
                gamepad1.rumble(100);
                gamepad2.rumble(100);
            }else if(elapsedGameTime.time() >= 90 && (elapsedGameTime.time() <= 92)){
                gamepad1.rumble(100);
                gamepad2.rumble(100);
            }

            if(elapsedGameTime.time() > 90 && CatHW_Async.isRedAlliance && !endGame){
                robot.lights.blink(15, RevBlinkinLedDriver.BlinkinPattern.RED,1000 );
                endGame = true;

            }else if(elapsedGameTime.time() > 90 && !CatHW_Async.isRedAlliance && ! endGame){
                robot.lights.blink(15, RevBlinkinLedDriver.BlinkinPattern.BLUE,1000 );
                endGame = true;
            }

            // Drive train speed control:

            if (gamepad1.right_trigger > .1 || gamepad1.left_trigger > .1) {
                driveSpeed = 0.50;
            } else if (gamepad1.right_bumper || gamepad1.left_bumper) {
                driveSpeed = 0.20;
            } else {
                driveSpeed = 0.30;
            }

            double forward = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y);
            forward = forward - (gamepad1.left_bumper? 1.0 : 0) * 0.3 + (gamepad1.right_bumper? 1.0:0) *.3;
            double strafe = ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x);
            if(gamepad1.dpad_left){
                strafe = strafe - 0.5;
            }else if(gamepad1.dpad_right){
                strafe = strafe + 0.5;
            }
            double turn = gamepad1.left_stick_x;


            // Input for setDrivePowers train and sets the dead-zones:
            leftFront = forward + strafe + turn;
            rightFront = forward - strafe - turn;
            leftBack = forward - strafe + turn;
            rightBack = forward + strafe - turn;

            // Calculate the scale factor:
            SF = robot.drive.findScalor(leftFront, rightFront, leftBack, rightBack);
            // Set powers to each setDrivePowers motor:
            leftFront = leftFront * SF * driveSpeed;
            rightFront = rightFront * SF * driveSpeed;
            leftBack = leftBack * SF * driveSpeed;
            rightBack = rightBack * SF * driveSpeed;

            // DRIVE!!!
            if (!turningMode) {
                robot.drive.setDrivePowers(leftFront, rightFront, leftBack, rightBack);
            }

            //--------------------------------------------------------------------------------------
            // Driver 2 Controls:
            //--------------------------------------------------------------------------------------

            //lets gamepad 2 set the lift to all five heights
            if(gamepad2.dpad_up){
                robot.jaws.setLiftHighPole(.3);

            }else if(gamepad2.dpad_left){
                robot.jaws.setLiftMiddleBack(.3);

            }else if(gamepad2.dpad_down){
                robot.jaws.setLiftGroundJunction(.25);

            } else if(gamepad2.ps) {
                robot.jaws.resetLift();

            } else if(gamepad2.dpad_right){
                robot.jaws.setLiftLowBack(.25);

            } else if(gamepad2.left_bumper){
                robot.jaws.setLiftBottom(.25);
            }


            //manually be able to move lift on both gamepad 1 and 2
            if(-gamepad2.left_stick_y > 0.5){
                robot.jaws.bumpLift(50);
            }else if(-gamepad2.left_stick_y < -0.5){
                robot.jaws.bumpLift(-50);
            }

            if(-gamepad2.right_stick_y > 0.5){
                robot.jaws.bumpArm(1);
            }else if(-gamepad2.right_stick_y < -0.5){
                robot.jaws.bumpArm(-1);
            }

            if(gamepad2.triangle){
                robot.jaws.armBack();
            }else if(gamepad2.cross){
                robot.jaws.armFront();
            }else if(gamepad2.square){
                robot.jaws.setLiftMiddleFront(.7);
            }else if(gamepad2.circle){
                robot.jaws.setLiftLowFront(.7);
            }

            robot.jaws.isDone();

            //gamepad 2 can close the claw
            if(gamepad2.right_trigger>0.1 || (robot.jaws.haveCone() && gamepad2.left_trigger <= 0.1)){
                robot.jaws.grabPos();
            }else if (gamepad2.left_trigger>.1){
                robot.jaws.unGrab();
            } else{
                robot.jaws.unGrab();
            }


            //--------------------------------------------------------------------------------------
            // Telemetry Data:
            //--------------------------------------------------------------------------------------
            telemetry.addData("Power", "LF %.2f RF %.2f LB %.2f RB %.2f", robot.drive.leftFrontMotor.getPower(), robot.drive.rightFrontMotor.getPower(),robot.drive.leftRearMotor.getPower(),robot.drive.rightRearMotor.getPower());
            //telemetry.addData("Power", "LF %s RF %s LB %s RB %s", robot.drive.leftFrontMotor.getDirection().toString(), robot.drive.rightFrontMotor.getDirection().toString(),robot.drive.leftRearMotor.getDirection().toString(),robot.drive.rightRearMotor.getDirection().toString());

            telemetry.addData("lift pos","Cur:%d target:%d pow:%.2f",robot.jaws.left_lift.getCurrentPosition(), robot.jaws.left_lift.getTargetPosition(), robot.jaws.left_lift.getPower());
            telemetry.addData("Game Timer","%.2f",elapsedGameTime.time());
            telemetry.addData("Tilt Pos","Cur:%d target:%d pow:%.2f",robot.jaws.tilt.getCurrentPosition(), robot.jaws.tilt.getTargetPosition(),robot.jaws.tilt.getPower());
            telemetry.addData("Claw POS",robot.jaws.claw.getPosition());
            telemetry.update();
            dashboardTelemetry.update();
        }

        robot.jaws.ourThread.stop();
        robot.eyes.stop();
    }

}