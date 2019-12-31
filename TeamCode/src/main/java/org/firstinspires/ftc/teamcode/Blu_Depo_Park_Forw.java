package org.firstinspires.ftc.teamcode;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blu_Depo_Park_Forw", group = "Autonomous")
public class Blu_Depo_Park_Forw extends org.firstinspires.ftc.teamcode.Maincanum {

    @Override
    public void runOpMode() throws InterruptedException {

        hereWeGoAgain(); //init
        hereWeGoAuto();  //autoinit
        waitForGo();


        driveNormal(-27); //drive away from wall

        driveStrafe(38,true); //drive to line








    }

}
