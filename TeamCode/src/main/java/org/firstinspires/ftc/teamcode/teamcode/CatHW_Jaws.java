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
    public DcMotor liftHook = null;

    public DcMotor intake = null;
    public DcMotor tilt = null;
    public DcMotor hexLift = null;
    public Servo dump = null;


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

        liftHook = hwMap.dcMotor.get("liftHook");
        liftHook.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftHook.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hwMap.dcMotor.get("intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        hexLift = hwMap.dcMotor.get("hexLift");
        hexLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hexLift.setTargetPosition(0);
        hexLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        dump = hwMap.servo.get("dump");

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
        setHexHeight(0,.25);
    }
    public void setLiftGroundJunction(double power){
        setHexHeight(0,power);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tilt.setTargetPosition(8);
    }
    public void setLiftLowBack(double power){
        setHexHeight(193,power);
    }
    public void setLiftMiddleBack(double power){
        setHexHeight(425,power);
    }
    public void setLiftLowFront(double power){
        setHexHeight(0,power);
    }
    public void setLiftMiddleFront(double power){
        setHexHeight(170, power);

    }

    public void setHexHeight(double height, double power){
        //hook.setTargetPosition(height);
        hexLift.setPower(power);

    }
    public void setRobotLift(double height, double power){
        //hook.setTargetPosition(height);
        liftHook.setPower(power);

    }
    public void setIntakePower(double power){
        intake.setPower(power);

    }



    public void resetLift(){
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftHook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftHook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void bumpArm(int bumpAmount) {
        tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        tilt.setTargetPosition(bumpAmount + tilt.getTargetPosition());
        tiltMode = TiltMode.MANUEL;

    }
    public void bumpHexHeight(int bumpAmount) {
        if (bumpAmount > 0.5){
            hexLift.setTargetPosition(bumpAmount + hexLift.getCurrentPosition());
            hexLift.setPower(1);

        }else if(bumpAmount <-0.5){
            hexLift.setTargetPosition(bumpAmount + hexLift.getCurrentPosition());
            hexLift.setPower(.7);


        }
    }
    public void autoSetBumpHexHeight() {
        hexLift.setTargetPosition(652);
        hexLift.setPower(1);


    }
    public void autoSetHexZero() {
        hexLift.setTargetPosition(0);
        hexLift.setPower(1);


    }

    public void setLiftPower(double power){
        left_lift.setPower(power);
    }

    public void zeroPos(){
        dump.setPosition(0.55);
    }
    public void dispence()  { dump.setPosition(.85); }




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