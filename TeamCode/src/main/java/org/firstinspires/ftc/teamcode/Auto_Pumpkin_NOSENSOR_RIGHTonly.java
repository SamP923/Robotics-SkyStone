package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Created by Sam on 2/24/2020
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="NO_SENSORS:  Drive RIGHT ONLY", group="Pumpkin: NoSensors")
public class Auto_Pumpkin_NOSENSOR_RIGHTonly extends LinearOpMode{
    Hardware_MecanumTest autopumpkin = new Hardware_MecanumTest();
    public void runOpMode(){

        autopumpkin.init(hardwareMap);

        waitForStart();
        movement(.5,-.5,-.5,.5);

        sleep(2000);

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
