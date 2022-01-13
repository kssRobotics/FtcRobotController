package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "motorControl", group = "TeleOp")

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
        crServo = hardwareMap.get(CRServo.class, "servo1");
        clawServo = hardwareMap.get(Servo.class, "servo2");
        topLimitSwitch1 = hardwareMap.get(DigitalChannel.class, "digital_touch1");
        //topLimitSwitch2 = hardwareMap.get(DigitalChannel.class, "digital_touch2");
        bottomLimitSwitch1 = hardwareMap.get(DigitalChannel.class, "digital_touch3");
        //getBottomLimitSwitch2= hardwareMap.get(DigitalChannel.class, "digital_touch4");


        //direction
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armMotor1.setDirection(DcMotor.Direction.FORWARD);
        armMotor2.setDirection(DcMotor.Direction.REVERSE);
        clawServo.setDirection(Servo.Direction.FORWARD);
        crServo.setDirection(CRServo.Direction.FORWARD);

        //power

        double armPower = (0.65);

        //position
        clawServo.setPosition(0);

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

            chaseeDrive();
            armControl();
            limitSwitch();

            telemetry.update();

            }
        }
    private void limitSwitch() {
        leftDrive.setPower(-gamepad1.left_stick_x * motorPower + gamepad1.left_stick_y * motorPower);
        rightDrive.setPower(gamepad1.left_stick_x * motorPower + gamepad1.left_stick_y * motorPower);
        telemetry.addData("leftStickPositionx", gamepad1.left_stick_x);
        telemetry.addData("leftStickPositionY", gamepad1.left_stick_y);
    }
    private void armControl(){

    }
    private void chaseeDrive(){

    }
}
