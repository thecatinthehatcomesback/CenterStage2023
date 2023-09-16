package org.firstinspires.ftc.teamcode.teamcode;


import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * CatHW_Jaws.java
 *
 *
 * This class containing common code accessing hardware specific to the movement of the jaws/intake.
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_Jaws extends CatHW_Subsystem
{

    // Motors: //
    //public CRServo intakeMotor = null;
    //public DcMotor intakeLift= null;
    public DcMotor left_lift = null;
    public DcMotor right_lift = null;
    public DcMotor tilt = null;
    public Servo claw = null;

    public ColorSensor intakeColor = null;
    public DistanceSensor intakeDistance = null;
    public ElapsedTime liftTime = null;
    public ElapsedTime pidTimer = null;

    private double lastError;
    private double lastTime;

    public Update_PID ourThread;

    private enum TiltMode{
        ARMBACK,
        ARMFRONT,
        ARMFRONTMEDIUM,
        ARMFRONTLOW,
        MANUEL,
        IDLE
    };
    private TiltMode tiltMode = TiltMode.IDLE;
    // Timers: //

    /* Constructor */
    public CatHW_Jaws(CatHW_Async mainHardware) {
        super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init() {

        // Define and initialize motors: //


        left_lift = hwMap.dcMotor.get("left_lift");
        left_lift.setDirection(DcMotorSimple.Direction.FORWARD);
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_lift.setTargetPosition(0);
        left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        right_lift = hwMap.dcMotor.get("right_lift");
        right_lift.setDirection(DcMotorSimple.Direction.REVERSE);
        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_lift.setTargetPosition(0);
        right_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        tilt = hwMap.dcMotor.get("tilt");
        tilt.setDirection(DcMotorSimple.Direction.REVERSE);
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setTargetPosition(0);
        tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        claw = hwMap.servo.get("claw");

        intakeColor = hwMap.colorSensor.get("intake_color");

        intakeDistance = hwMap.get(DistanceSensor.class, "intake_color");

        liftTime = new ElapsedTime();
        pidTimer = new ElapsedTime();

        ourThread = new Update_PID(this);
        Thread th = new Thread(ourThread,"update pid");
        th.start();
    }



    //----------------------------------------------------------------------------------------------
    // Jaw Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * Set the power of intake motor.
     *
     * @param power at which the motors will spin.
     */
    public void setJawPower(double power) {
        // Max of 80% power for VEX 393 motors
        if (power > 0.8) {
            power = 0.8;
        }
        if (power < -0.8) {
            power = -0.8;
        }

        //intakeMotor.setPower(power);
    }

    //Lift mechanism

    // TOP 6189
    public void setLiftBottom(double power){
        setLiftHeight(0,.25);
        armFront();
    }
    public void setLiftGroundJunction(double power){
        setLiftHeight(0,power);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tilt.setTargetPosition(8);
    }
    public void setLiftLowBack(double power){
        setLiftHeight(193,power);
        armBack();
    }
    public void setLiftMiddleBack(double power){
        setLiftHeight(425,power);
        armBack();
    }
    public void setLiftLowFront(double power){
        setLiftHeight(0,power);

        armFrontLow();
    }
    public void setLiftMiddleFront(double power){
        setLiftHeight(170, power);
        armFrontMedium();
    }
    public void setLiftHighPole(double power){
        left_lift.setTargetPosition(504);
        left_lift.setPower(power);
        right_lift.setTargetPosition(504);
        right_lift.setPower(power);
        setArmHeight(160);
    }

    public void setLiftHeight(int height, double power){
        left_lift.setTargetPosition(height);
        left_lift.setPower(power);
        right_lift.setTargetPosition(height);
        right_lift.setPower(power);

    }
    public void setArmHeight(int height){
        tilt.setTargetPosition(height);
        tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tiltMode = TiltMode.MANUEL;

    }
    public void armBack(){
        tilt.setTargetPosition(185);
        tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tiltMode = TiltMode.ARMBACK;
    }

    public void armFront(){
        tilt.setTargetPosition(0);
        tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tiltMode = TiltMode.ARMFRONT;
    }
    public void armFrontMedium(){

        tilt.setTargetPosition(100);
        tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tilt.setPower(.4);
        tiltMode = TiltMode.ARMFRONTMEDIUM;
    }
    public void armFrontLow(){
        tilt.setTargetPosition(75);
        tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastTime = pidTimer.seconds();
        lastError = 0;
        tiltMode = TiltMode.ARMFRONTLOW;
    }


    public void resetLift(){
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void bumpArm(int bumpAmount) {
        tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        tilt.setTargetPosition(bumpAmount + tilt.getTargetPosition());
        tiltMode = TiltMode.MANUEL;

    }
    public void bumpLift(int bumpAmount) {
        if (bumpAmount > 0.5){
            left_lift.setTargetPosition(bumpAmount + left_lift.getCurrentPosition());
            right_lift.setTargetPosition(bumpAmount + right_lift.getCurrentPosition());
            left_lift.setPower(1);
            right_lift.setPower(1);

        }else if(bumpAmount <-0.5){
            left_lift.setTargetPosition(bumpAmount + left_lift.getCurrentPosition());
            left_lift.setPower(.7);
            right_lift.setPower(.7);
            right_lift.setTargetPosition(bumpAmount + right_lift.getCurrentPosition());


        }
    }

    public void setLiftPower(double power){
        left_lift.setPower(power);
    }

    public void grabPos(){
        claw.setPosition(.95);
    }
    public void unGrab()  { claw.setPosition(.83); }


    //intake color sensor methods
    public boolean haveCone() {
        //Log.d("catbot", String.format("Have Freight r/g/b/a %4d %4d %4d %4d",
        //        intakeColor.red(),intakeColor.green(),intakeColor.blue(),intakeColor.alpha()));

        if(intakeDistance.getDistance(DistanceUnit.INCH)< 1.2 ){
            return true;
        }
        return false;
    }






    public void updatePID(){
        /*if(tiltMode == TiltMode.ARMBACK){
            if(tilt.getCurrentPosition() < 100 ){
                tilt.setPower(.9);
            }else if(tilt.getCurrentPosition() < 150){
                tilt.setPower(.2);
            }else if(tilt.getCurrentPosition() < 180){
                tilt.setPower(.1);
            }else{
                tilt.setPower(0);
                tiltMode = TiltMode.IDLE;
                result = true;
            }
        }else if(tiltMode == TiltMode.ARMFRONT){
            if(tilt.getCurrentPosition() > 135){
                tilt.setPower(-.9);
            }else if(tilt.getCurrentPosition()> 40){
                tilt.setPower(-.2);
            }else if(tilt.getCurrentPosition()>15){
                tilt.setPower(-.1);
            }else {
                tilt.setPower(0);
                result = true;
            }
        }else*/
        //if(tiltMode == TiltMode.ARMFRONTLOW || tiltMode == TiltMode.ARMFRONTMEDIUM|| tiltMode == TiltMode.MANUEL){
        if(tiltMode!= TiltMode.IDLE){
            double Kp = 0.03;
            double Kd = 0.0015;
            double curTime = pidTimer.seconds();
            double error = tilt.getTargetPosition() - tilt.getCurrentPosition();
            double derivative = (error - lastError)/(curTime - lastTime);
            double theta = tilt.getCurrentPosition() * 1.33; //this is in degrees
            theta *= 3.14/180.0;
            double gravityAdjustment = Math.sin(theta) * 0.15;

            lastTime = curTime;
            lastError = error;
            tilt.setPower(Kp * error + Kd * derivative + gravityAdjustment);
            Log.d("catbot",String.format("pow: %.3f error: %.1f der: %.2f",
                    Kp * error + Kd * derivative + gravityAdjustment,error,derivative));
        }
    }

    //----------------------------------------------------------------------------------------------
    // isDone Method:
    //----------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {
        //Log.d("catbot", String.format(" intake power %.2f,", transferMotor.getPower()));
        boolean result = false;



        if(Math.abs(left_lift.getCurrentPosition()-left_lift.getTargetPosition())<20){
            if(Math.abs(tilt.getCurrentPosition()-tilt.getTargetPosition()) < 15){
                result = true;
            }
        }

        // turn off lift when it's all the way down.
        //if ( (left_lift.getTargetPosition() == 0) && (Math.abs(left_lift.getCurrentPosition()) < 50)) {
           // left_lift.setPower(0);
        //}

        return result;
    }
}