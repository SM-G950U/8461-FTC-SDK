package org.firstinspires.ftc.teamcode;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blu_Foun_Park_Forw", group = "Autonomous")
public class Blu_Foun_Park_Forw extends org.firstinspires.ftc.teamcode.Maincanum {


    @Override
    public void runOpMode() throws InterruptedException {

        hereWeGoAgain(); //init
        hereWeGoAuto(); //autonomous init
        waitForGo(); // hmmmm, what could this do

        grabFReturn(false);

        driveStrafe(25,false); //drive towards line

        driveNormal(-23); // drive forwards out of way of other robot

        driveStrafe(20, false); // move onto line



    }
}