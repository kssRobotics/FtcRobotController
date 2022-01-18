package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "FinalTeleOp", group = "TeleOp")

public class FinalTeleOp extends LinearOpMode {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor1 = null;
    private DcMotor armMotor2 = null;
    private CRServo crServo = null;
    private Servo clawServo = null;
    private DigitalChannel topLimitSwitch1 = null;
    //private DigitalChannel topLimitSwitch2 = null;
    private DigitalChannel bottomLimitSwitch1 = null;
    //private DigitalChannel bottomLimitSwitch2= null;

    double motorPower = (0.5);


    @Override
    public void runOpMode() {
        //mapping
        leftDrive = hardwareMap.get(DcMotor.class, "motor1");
        rightDrive = hardwareMap.get(DcMotor.class, "motor2");
        armMotor1 = hardwareMap.get(DcMotor.class, "motor3");
        armMotor2 = hardwareMap.get(DcMotor.class, "motor4");
        crServo = hardwareMap.get(CRServo.class, "servo2");
        clawServo = hardwareMap.get(Servo.class, "servo1");
        topLimitSwitch1 = hardwareMap.get(DigitalChannel.class, "digital_touch1");
        //topLimitSwitch2 = hardwareMap.get(DigitalChannel.class, "digital_touch2");
        bottomLimitSwitch1 = hardwareMap.get(DigitalChannel.class, "digital_touch2");
        //getBottomLimitSwitch2= hardwareMap.get(DigitalChannel.class, "digital_touch4");


        //direction
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armMotor1.setDirection(DcMotor.Direction.FORWARD);
        armMotor2.setDirection(DcMotor.Direction.REVERSE);
        clawServo.setDirection(Servo.Direction.FORWARD);
        crServo.setDirection(CRServo.Direction.FORWARD);

        //servo
        clawServo.setPosition(1);
        clawServo.scaleRange(0.75, 1);

        //LimitSwitch
        topLimitSwitch1.setMode(DigitalChannel.Mode.INPUT);
        bottomLimitSwitch1.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        telemetry.addData("LeftMotorDirection", leftDrive.getDirection());
        telemetry.addData("RightMotorDirection", rightDrive.getDirection());
        telemetry.addData("ArmMotorDirection", armMotor1.getDirection());
        telemetry.addData("ClawServoDirection", clawServo.getDirection());
        telemetry.update();


        while (opModeIsActive()) {
            chasisDrive();
            armControl();
            limitSwitch();
            claw();
            telemetry.addData("Velocity", ((DcMotorEx) armMotor1).getVelocity());
            telemetry.addData("Top", topLimitSwitch1.getState());
            telemetry.addData("Bottom ", bottomLimitSwitch1.getState());
            telemetry.addData("Position", clawServo.getPosition());

            telemetry.update();

        }
    }

    private void chasisDrive() {
        leftDrive.setPower(-gamepad1.left_stick_x * motorPower + gamepad1.left_stick_y * motorPower);
        rightDrive.setPower(gamepad1.left_stick_x * motorPower + gamepad1.left_stick_y * motorPower);
        telemetry.addData("leftStickPositionx", gamepad1.left_stick_x);
        telemetry.addData("leftStickPositionY", gamepad1.left_stick_y);
    }

    private void armControl() {
        if (gamepad1.y) {
            ((DcMotorEx) armMotor1).setVelocity(1500);
            ((DcMotorEx) armMotor2).setVelocity(1500);
        } else if (gamepad1.a) {
            ((DcMotorEx) armMotor1).setVelocity(-1500);
            ((DcMotorEx) armMotor2).setVelocity(-1500);
        } else if (gamepad1.b) {
            ((DcMotorEx) armMotor1).setVelocity(0);
            ((DcMotorEx) armMotor2).setVelocity(0);
        }

    }

    private void limitSwitch() {
        if (topLimitSwitch1.getState() == true && bottomLimitSwitch1.getState() == false) ;
        {
            ((DcMotorEx) armMotor1).setVelocity(-240);
            ((DcMotorEx) armMotor2).setVelocity(-240);
            sleep(250);
            ((DcMotorEx) armMotor1).setVelocity(0);
            ((DcMotorEx) armMotor2).setVelocity(0);
            telemetry.update();
        }
        if (topLimitSwitch1.getState() == false && bottomLimitSwitch1.getState() == true) {
            ((DcMotorEx) armMotor2).setVelocity(240);
            ((DcMotorEx) armMotor1).setVelocity(240);
            sleep(250);
            ((DcMotorEx) armMotor2).setVelocity(0);
            ((DcMotorEx) armMotor1).setVelocity(0);
            telemetry.update();
        } else if (topLimitSwitch1.getState() == true && bottomLimitSwitch1.getState() == true) {
            ((DcMotorEx) armMotor2).setVelocity(0);
            ((DcMotorEx) armMotor1).setVelocity(0);
            telemetry.update();
        }
    }
    private void claw(){
        clawServo.setPosition(1- gamepad1.right_trigger);
        telemetry.addData("Position", clawServo.getPosition());

    }

}


