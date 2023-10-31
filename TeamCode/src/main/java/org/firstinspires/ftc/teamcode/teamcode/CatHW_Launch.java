package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * CatHW_Launch.Java
 *
 *
 * This class containing common code accessing hardware specific to the drone launch
 * .
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_Launch extends CatHW_Subsystem {
    public Servo launcher;

    /* Constructor */
    public CatHW_Launch(CatHW_Async mainHardware) {
        super(mainHardware);
    }
    /* Initialize standard Hardware interfaces */
    public void init() {
        // Define and initialize motors: //
        launcher = hwMap.servo.get("drone");
        arm();
    }
    public void arm( ){
        launcher.setPosition(0.7);
    }
    public void launch( ){
        launcher.setPosition(0.5);
    }
}

