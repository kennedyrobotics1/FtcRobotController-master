package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@TeleOp

public class TestServo extends OpMode {
    ServoImplEx servo0;
    ServoImplEx servo1;
    boolean servo0WasPressed = false;
    boolean servo1WasPressed = false;
    boolean bothPressed = false;
    double servo1Counter = 0;
    double servo2Counter = 0;

    public void init() {
        servo0 = hardwareMap.get(ServoImplEx.class, "servo1");
        servo0.setPosition(0.08);
        servo1 = hardwareMap.get(ServoImplEx.class, "servo2");
        servo1.setPosition(0.92);
    }

    public void loop() {
        if(gamepad1.dpad_right){
            servo0.setPosition(0);
        } else if(gamepad1.dpad_left){
            servo0.setPosition(0.2);
        }
        if(gamepad1.x){
            servo1.setPosition(1);
        } else if(gamepad1.b){
            servo1.setPosition(0.8);
        }
        /*
        if (gamepad1.a && !servo0WasPressed) {
            if (servo0.getPosition() == 0.5) {
                servo0.setPosition(0);
            }
            else {
                servo0.setPosition(0.5);
            }
            servo0WasPressed = true;
        }
        else if (!gamepad1.a) {
            servo0WasPressed = false;
        }

        if (gamepad1.b && !servo1WasPressed) {
            if (servo1.getPosition() == 0.5) {
                servo1.setPosition(1);
            }
            else {
                servo1.setPosition(0.5);
            }
            servo1WasPressed = true;
        }
        else if (!gamepad1.b) {
            servo1WasPressed = false;
        }*/
        telemetry.addData("Servo1 position ", servo0.getPosition());
        telemetry.addData("Servo2 position ", servo1.getPosition());
        telemetry.addData("counter1 ", servo1Counter);
        telemetry.addData("counter2 ", servo2Counter);
    }
}