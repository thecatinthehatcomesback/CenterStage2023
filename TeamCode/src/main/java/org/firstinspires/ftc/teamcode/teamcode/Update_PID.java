package org.firstinspires.ftc.teamcode.teamcode;

public class Update_PID implements Runnable {
    boolean isStopRequested = false;
    public CatHW_Jaws jaws = null;
    public Update_PID(CatHW_Jaws Jaws){
        jaws = Jaws;
    }
    @Override
    public void run(){

        while(!isStopRequested){
            jaws.updatePID();
            try {
                Thread.sleep(30);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
    }
    public void stop(){
        isStopRequested = true;
    }

}
