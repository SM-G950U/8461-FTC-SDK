package org.firstinspires.ftc.teamcode;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red_Depo_Park_Forw", group = "Autonomous")
public class Red_Depo_Park_Forw extends org.firstinspires.ftc.teamcode.Maincanum {

    @Override
    public void runOpMode() throws InterruptedException {
        //-----DONE-----
        hereWeGoAgain(); //init
        hereWeGoAuto();  //autoinit
        waitForGo();


        driveNormal(-17); //drive away from wall

        driveStrafeEdit(43,false); //drive to line

        driveNormal(-5); //drive into bridge

    }

}
