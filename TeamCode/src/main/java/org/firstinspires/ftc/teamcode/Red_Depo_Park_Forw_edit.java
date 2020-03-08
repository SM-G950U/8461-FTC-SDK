package org.firstinspires.ftc.teamcode;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red_Depo_Park_Forw_Edit", group = "Autonomous")
public class Red_Depo_Park_Forw_edit extends Maincanum {

    @Override
    public void runOpMode() throws InterruptedException {
        //-----DONE-----
        hereWeGoAgain(); //init
        hereWeGoAuto();  //autoinit
        waitForGo();

        sleep(20000);

        driveNormal(-13); //drive away from wall

        driveStrafeEdit(25,true); //drive to line

        driveNormal(-5); //drive into bridge

    }

}
