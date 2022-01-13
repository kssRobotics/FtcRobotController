package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp (name = " armEncoder", group = "TeleOp")

public class armEncoder extends LinearOpMode {

    private DcMotor armMotor1=null;
    private DcMotor armMotor2=null;

    @Override
    public void runOpMode(){
         armMotor1 = hardwareMap.get(DcMotor.class, "motor3");
         armMotor2 = hardwareMap.get(DcMotor.class, "motor4");

         //direction
         armMotor1.setDirection(DcMotor.Direction.FORWARD);
         armMotor2.setDirection(DcMotor.Direction.FORWARD);

         //power
         double armPower=0.65;

         //encoder


  }
}


