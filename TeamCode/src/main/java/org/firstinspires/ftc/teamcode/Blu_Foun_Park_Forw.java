package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blu_Foun_Park_Forw", group = "Autonomous")
@Disabled
public class Blu_Foun_Park_Forw extends org.firstinspires.ftc.teamcode.Maincanum {


    @Override
    public void runOpMode() throws InterruptedException {

        hereWeGoAgain(); //init
        hereWeGoAuto(); //autonomous init
        waitForGo(); // hmmmm, what could this do

        grabFReturn(false);

        sleep(50);
        driveStrafe(1,true); //drive towards line
        sleep(50);
        driveStrafe(26,false);

        driveNormal(-18); // drive forwards out of way of other robot

        driveStrafe(19, false); // move onto line



    }
}
