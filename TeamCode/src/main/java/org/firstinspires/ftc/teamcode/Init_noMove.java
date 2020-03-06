package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Init_noMove", group = "Autonomous")
@Disabled
public class Init_noMove extends org.firstinspires.ftc.teamcode.Maincanum {


    @Override
    public void runOpMode() throws InterruptedException {

        hereWeGoAgain();
        hereWeGoAuto();
        telemetry.addData("none auto, nothing will happen. auto init only","");
        telemetry.update();
        waitForGo();




    }
}
