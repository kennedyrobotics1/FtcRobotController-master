package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
    private CRServo servo0 = null;
    //private CRServo servo1 = null;
    //private CRServo servo2 = null;
    private AnalogInput analogInput;
    double kp = 1.0/200;
    double kd = 0.01/120;
    double startTime;
    double previousTime;
    double time;
    double deltaTime;
    double targetPosition0 = 0;
    double targetPosition1 = 0;
    double setPointClaw;
    double positionClaw;
    double startClaw = 0;
    double errorClaw;
    double previousPositionClaw;
    double deltaPositionClaw;
    double velocityClaw;

    boolean forward = true;
    boolean back = true;
    double counter = 0;
    double servoPosition;
    public void init() {
        servo0 = hardwareMap.get(CRServo.class, "servo0");
        //servo0.setDirection(Servo.Direction.FORWARD);
        //servo1 = hardwareMap.get(CRServo.class, "servo1");
        //servo1.setDirection(Servo.Direction.REVERSE);
        //servo2 = hardwareMap.get(CRServo.class, "servo2");
        //servo2.setDirection(Servo.Direction.FORWARD);
        analogInput = hardwareMap.get(AnalogInput.class, "servoEncoder");


    }

    public void start(){
        previousPositionClaw = servoPosition;
        servoPosition = analogInput.getVoltage() / 3.3 * 360 + counter * 360;
        targetPosition1 = 0;
        startClaw = servoPosition;
        deltaPositionClaw = positionClaw - previousPositionClaw;

        positionClaw = servoPosition - startClaw;
        setPointClaw = servoPosition;
        errorClaw = setPointClaw - servoPosition;
        /*if(servoPosition >= 357.5 && setPointClaw > 200){
            setPointClaw -= 350;
        }
        if(servoPosition <= 2.5 && setPointClaw < 150){
            setPointClaw += 350;
        }*/

        counter = 0;
        servo0.setPower(-(errorClaw * kp));
        //servo1.setPower(0);
        //servo2.setPower(0);


    }

    public void loop(){

        updateTime();
        servoPosition = analogInput.getVoltage() / 3.3 * 360 + counter * 360;

        //positionClaw = servoPosition - startClaw;
        /*if(errorClaw >= 200){
            setPointClaw = servoPosition;
            errorClaw = 0;
        }
        if(errorClaw <= -200){
            setPointClaw = servoPosition;
            errorClaw = 0;
        }*/

        if(errorClaw >= 200 && forward){
            counter += 1;
            forward = false;
            back = true;
        }
        if(errorClaw <= -200 && back){
            counter -= 1;
            back = false;
            forward = true;
        }
        /*if(servoPosition == 5 && velocityClaw < 0){
            counter -= 1;
        }*/

        previousPositionClaw = servoPosition;
//        servoPosition = analogInput.getVoltage() / 3.3 * 360 + counter * 360;
        deltaPositionClaw = servoPosition - previousPositionClaw;

        errorClaw = setPointClaw - servoPosition;
        velocityClaw = deltaPositionClaw / deltaTime;

        if(gamepad2.left_stick_y <= -0.2){
            setPointClaw += 250 * deltaTime;
        } else if (gamepad2.left_stick_y >= 0.2){
            setPointClaw -= 250 * deltaTime;
        }

        servo0.setPower(-(errorClaw * kp));

        if(gamepad2.dpad_up){
            setPointClaw = 180;
        }

        //servo1.setPosition(targetPosition1);
        //servo2.setPosition(targetPosition1);
        //servo1.setPower(-targetPosition1);
        //servo2.setPower(targetPosition1);
        if(gamepad1.y){
            targetPosition0 += 0.0004;
        } else if (gamepad1.a){
            targetPosition0 -= 0.0004;
        }

        if(gamepad1.x){
           // targetPosition1 += 0.003;
            targetPosition1 = 0.1;
        } else if (gamepad1.b){
           // targetPosition1 -= 0.003;
            targetPosition1 = -0.1;
        } else {
            targetPosition1 = 0;
        }

        telemetry.addData("deltaPos ", deltaPositionClaw);
        telemetry.addData("deltaTime ", deltaTime);
        telemetry.addData("Counter ", counter);
        telemetry.addData("Velocity ", velocityClaw);
        telemetry.addData("Current servoPosition: ", servoPosition);
        telemetry.addData("target ", setPointClaw);
        telemetry.addData("Power ", (errorClaw * kp));

        //telemetry.addData("Current Position2: ", targetPosition1);
        //telemetry.addData("Current Position3: ", targetPosition1);
        telemetry.update();
    }
    public void updateTime() {
        previousTime = time;
        time = ((double) (System.nanoTime() - startTime) ) / Math.pow(10,9);
        deltaTime = time - previousTime;
    }

}