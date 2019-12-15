package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Created by Sam on 11/26/19/
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Auto_Pumpkin: ColorSensorTest", group="Pushbot")
public class Auto_Pumpkin_ColorSensorTest extends LinearOpMode{
    Hardware_MecanumTest autopumpkin = new Hardware_MecanumTest();

    double colorCondition;
    public void runOpMode(){

        autopumpkin.init(hardwareMap);

        waitForStart();

        /*
            Planned Pseudocode
                1. drive forward to stones
                2. move all the way left
                3. move right until see skystone
                4. move servo to grab skystone
                5. move backwards to remove skystone from quarry set up
                6. move right to cross skybridge
         */

        // moves FORWARD to Stones
        moveForward(-.75);
        sleep(2000);
        stopMovement();

        // moves RIGHT to wall
        moveRight(.75);
        sleep(2500);
        stopMovement();

        // moves LEFT until it finds a regular stone
        /*do {
            if (autopumpkin.stoneColorS.blue() != 0) colorCondition = autopumpkin.stoneColorS.red() * autopumpkin.stoneColorS.green() / (autopumpkin.stoneColorS.blue() * autopumpkin.stoneColorS.blue());
            moveLeft(.25);
        }while (colorCondition < 2);

        // CONTINUES to move LEFT until it finds a Skystone
        do {
            if (autopumpkin.stoneColorS.blue() != 0) colorCondition = autopumpkin.stoneColorS.red() * autopumpkin.stoneColorS.green() / (autopumpkin.stoneColorS.blue() * autopumpkin.stoneColorS.blue());
            moveLeft(.25);
        }while (colorCondition > 2);
        */
        // captures block
        autopumpkin.blockStealer.setPosition(1);

        // moves LEFT until it sees a BLUE line
        do {
            moveLeft(.25);
        } while (autopumpkin.parkColorS.red() > autopumpkin.parkColorS.blue());
        // moves the bot a bit past the line
        sleep(1500);

        // releases block
        autopumpkin.blockStealer.setPosition(0);

        // moves RIGHT until it sees a BLUE line
        do {
            moveRight(.25);
        } while (autopumpkin.parkColorS.red() > autopumpkin.parkColorS.blue());

        // stops the bot
        stopMovement();
    }

    public void moveRobot(double LF, double LB, double RF, double RB)
    {
        autopumpkin.LFmotor.setPower(LF);
        autopumpkin.LBmotor.setPower(LB);
        autopumpkin.RFmotor.setPower(RF);
        autopumpkin.RBmotor.setPower(RB);
    }
    public void moveForward(double motorPower)
    {
        autopumpkin.LFmotor.setPower(motorPower);
        autopumpkin.LBmotor.setPower(motorPower);
        autopumpkin.RFmotor.setPower(motorPower);
        autopumpkin.RBmotor.setPower(motorPower);
    }

    public void moveLeft(double motorPower)
    {
        autopumpkin.LFmotor.setPower(-motorPower);
        autopumpkin.LBmotor.setPower(motorPower);
        autopumpkin.RFmotor.setPower(motorPower);
        autopumpkin.RBmotor.setPower(-motorPower);
    }

    public void moveRight(double motorPower)
    {
        autopumpkin.LFmotor.setPower(motorPower);
        autopumpkin.LBmotor.setPower(-motorPower);
        autopumpkin.RFmotor.setPower(-motorPower);
        autopumpkin.RBmotor.setPower(motorPower);
    }

    public void stopMovement()
    {
        autopumpkin.LFmotor.setPower(0.0);
        autopumpkin.LBmotor.setPower(0.0);
        autopumpkin.RFmotor.setPower(0.0);
        autopumpkin.RBmotor.setPower(0.0);
    }
}
