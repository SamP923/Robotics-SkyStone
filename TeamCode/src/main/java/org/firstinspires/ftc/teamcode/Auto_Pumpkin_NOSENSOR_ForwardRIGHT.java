package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Created by Sam on 2/9/20
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="NO_SENSORS: FORWARD AND RIGHT", group="Pumpkin: NoSensors")
public class Auto_Pumpkin_NOSENSOR_ForwardRIGHT extends LinearOpMode{
    Hardware_MecanumTest autopumpkin = new Hardware_MecanumTest();
    public void runOpMode(){

        autopumpkin.init(hardwareMap);

        waitForStart();

        //move FORWARD
        movement(.75,.75,.75,.75);
        sleep (600);
        movement(.5,-.5,-.5,.5);
        sleep(1500);

        movement(0,0,0,0);
    }

    public void movement(double LF, double LB, double RF, double RB)
    {
        autopumpkin.LFmotor.setPower(LF);
        autopumpkin.LBmotor.setPower(LB);
        autopumpkin.RFmotor.setPower(RF);
        autopumpkin.RBmotor.setPower(RB);
    }
}
