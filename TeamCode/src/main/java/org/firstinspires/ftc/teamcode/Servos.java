package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



// todo: write your code here
@TeleOp(name = "Servos", group = "Iterative OpMode")

public class Servos extends BasicOpMode_Iterative {
    //variable declarations
    private Servo servo0 = null;
    private Servo servo1 = null;
    private Servo servo2 = null;
    double targetPosition0 = 0;
    double targetPosition1 = 0;
    public void init() {
        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo0.setDirection(Servo.Direction.FORWARD);
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo1.setDirection(Servo.Direction.REVERSE);
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo2.setDirection(Servo.Direction.FORWARD);

    }

    public void start(){
        targetPosition0 = 0.46;
        servo0.setPosition(servo0.getPosition());
        targetPosition1 = 0.15;
        servo1.setPosition(targetPosition1);
        servo2.setPosition(targetPosition1);
    }

    public void loop(){
        servo0.setPosition(targetPosition0);
        servo1.setPosition(targetPosition1);
        servo2.setPosition(targetPosition1);
        if(gamepad1.y){
            targetPosition0 += 0.0003;
        } else if (gamepad1.a){
            targetPosition0 -= 0.0003;
        }
        if(gamepad1.x && targetPosition1 < 1.0){
            targetPosition1 += 0.003;
        } else if (gamepad1.b && targetPosition1 > 0){
            targetPosition1 -= 0.003;
        }



        telemetry.addData("Current Position: ", servo0.getPosition());
        telemetry.addData("Current Position2: ", targetPosition1);
        telemetry.addData("Current Position3: ", targetPosition1);
        telemetry.update();
    }

}