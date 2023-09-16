package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TestPIDOpmode extends LinearOpMode {

    CatHW_Async robot = new CatHW_Async();

    DcMotorEx leftFrontMotor = null;
    DcMotorEx rightFrontMotor = null;
    DcMotorEx leftRearMotor = null;
    DcMotorEx rightRearMotor = null;




    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, this);

        leftFrontMotor = hardwareMap.get(DcMotorEx.class,"left_front_motor");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class,"right_front_motor");
        leftRearMotor = hardwareMap.get(DcMotorEx.class,"left_rear_motor");
        rightRearMotor = hardwareMap.get(DcMotorEx.class,"right_rear_motor");

        // Define motor directions: //
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        double referenceAngle = 90;
        while (opModeIsActive()){

            double power = PIDControl(referenceAngle, robot.T265.getRotation());
            leftFrontMotor.setPower(power);
            leftRearMotor.setPower(power);
            rightFrontMotor.setPower(-power);
            rightRearMotor.setPower(-power);

        }
    }
    public double PIDControl(double reference, double state){
        double error = robot.T265.reduceAngle(reference - state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;


        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;

    }
}
