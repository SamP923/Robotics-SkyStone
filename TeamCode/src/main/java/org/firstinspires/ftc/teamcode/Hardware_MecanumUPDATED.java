package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 *  Created by Sam on 9/19/2019.
 ****************************
 *  HORIZONTAL HUB: HUB 2
 *  LC 0         RC 1
 *        HUB 2
 *  FB 2
 *
 *  HUB 2: address 3
 *  Servo Port 0: rotateClaw
 *  Servo Port 4: claw
 ****************************
 *  VERTICAL HUB: HUB 1
 *  LF 0         RF 1
 *        HUB 1
 *  LB 2         RB 3
 *
 *  HUB 1: address 2
 *  Servo Port 5: blockPusher
 ****************************
 */

class Hardware_MecanumUPDATED {

    //Drive train:
    DcMotor LFmotor;
    DcMotor LBmotor;
    DcMotor RFmotor;
    DcMotor RBmotor;

    DcMotor LCompliantmotor;
    DcMotor RCompliantmotor;

    DcMotor FourBarmotor;

    Servo clawControl;
    Servo liftClaw;
    Servo fMover;

    ColorSensor parkColorS;
    ColorSensor stoneColorS;
    DistanceSensor stoneDistance;

    public void init(HardwareMap hwMap){

        // assigns names
        LFmotor = hwMap.dcMotor.get("LFmotor");
        LBmotor = hwMap.dcMotor.get("LBmotor");
        RFmotor = hwMap.dcMotor.get("RFmotor");
        RBmotor = hwMap.dcMotor.get("RBmotor");

        LCompliantmotor = hwMap.dcMotor.get("RCmotor");
        RCompliantmotor = hwMap.dcMotor.get("LCmotor");

        FourBarmotor = hwMap.dcMotor.get("4Bmotor");

        clawControl = hwMap.servo.get("clawControl");
        liftClaw = hwMap.servo.get("liftClaw");
        fMover = hwMap.servo.get("fMover");
        parkColorS = hwMap.get(ColorSensor.class, "parkColorS");
        stoneColorS = hwMap.get(ColorSensor.class, "stoneColorS");
        stoneDistance = hwMap.get(DistanceSensor.class, "stoneDistance");


        // set brakes on motors
        LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LCompliantmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RCompliantmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FourBarmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set direction of motors facing opposite directions
        // DcMotors: Clockwise by default; clockwise on left side = forward
        // FRONT = non-compliant wheels
        LFmotor.setDirection(DcMotor.Direction.FORWARD);
        LBmotor.setDirection(DcMotor.Direction.FORWARD);
        RFmotor.setDirection(DcMotor.Direction.REVERSE);
        RBmotor.setDirection(DcMotor.Direction.REVERSE);

        LCompliantmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RCompliantmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        FourBarmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        stopAllMotors();
        openClaw();
        rotateClawUpmost();
        rotateArmUpmost();
    }

    public void stopAllMotors(){
        //set pwr to 0
        LFmotor.setPower(0.0);
        LBmotor.setPower(0.0);
        RFmotor.setPower(0.0);
        RBmotor.setPower(0.0);

        LCompliantmotor.setPower(0.0);
        RCompliantmotor.setPower(0.0);
        FourBarmotor.setPower(0);
    }

    public void moveMecanum( int leftY, int rightX, int leftX){
        double maxPower = 0.5;
        double minPower = -0.5;

        double LFpower = Range.clip(leftY + rightX + leftX, minPower, maxPower);
        double RFpower = Range.clip(leftY - rightX - leftX, minPower, maxPower);
        double LBpower = Range.clip(leftY + rightX - leftX, minPower, maxPower);
        double RBpower = Range.clip(leftY - rightX + leftX, minPower, maxPower);

        LFmotor.setPower(LFpower);
        RFmotor.setPower(RFpower);
        LBmotor.setPower(LBpower);
        RBmotor.setPower(RBpower);
    }

    public void openClaw(){
        clawControl.setPosition(0);
    }

    public void closeClaw(){
        clawControl.setPosition(1);
    }

    public void rotateClawUpmost(){
        liftClaw.setPosition(1);
    }

    public void rotateClawDown(){
        liftClaw.setPosition(0);
    }

    public void rotateArmUpmost(){

    }

    public void rotateArmDown(){

    }
}