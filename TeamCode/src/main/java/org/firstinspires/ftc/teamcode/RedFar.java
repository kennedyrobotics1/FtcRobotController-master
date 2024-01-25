package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name="RedFar", group="Linear OpMode")
public class RedFar extends LinearOpMode {

    private DcMotor motor0 = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;


    private ElapsedTime runtime = new ElapsedTime();

    double kp = 1.0 / 500;
    double start0 = 0;
    double start1 = 0;
    double start2 = 0;
    double start3 = 0;
    double setPoint0 = 0;
    double setPoint1 = 0;
    double setPoint2 = 0;
    double setPoint3 = 0;
    double position0;
    double position1;
    double position2;
    double position3;
    double error0;
    double error1;
    double error2;
    double error3;
    double motorPower0;
    double motorPower1;
    double motorPower2;
    double motorPower3;
    double headingError;
    double setYaw;
    double kpA = 1.0/100;

    private CRServo servo0 = null;
    private CRServo servo1 = null;
    private CRServo servo2 = null;
    double targetPosition0 = 0;
    double targetPosition1 = 0;

    private DcMotor arm0 = null;
    private DcMotor arm1 = null;
    double kpArm = 1.0/100;
    double kpClaw = 1.0/200;
    double ki = 0;
    double kd = 0.01/120;
    double setPoint = 0;
    double position;
    double start = 0;
    double error = setPoint - position;

    double startTime;
    double previousTime;
    double time;
    double deltaTime;

    double previousPosition;
    double deltaPosition;
    double velocity;

    double oldTime;
    double newTime;

    private AnalogInput analogInput;

    double setPointClimb = 0;
    double positionClimb;
    double startClimb = 0;
    double errorClimb = setPointClimb - positionClimb;

    double setPointClaw = 0;
    double positionClaw;
    double startClaw = 0;
    double errorClaw = setPointClaw - positionClaw;

    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;

    private static final boolean USE_WEBCAM = true;

    private TfodProcessor tfod;

    private VisionPortal visionPortal;
    private static final String TFOD_MODEL_ASSET = "BlueBox.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/BlueBox.tflite";
    private static final String[] LABELS = {
            "BB",
    };

    double getPreviousPositionClimb;
    double deltaPositionClimb;
    double velocityClimb;

    double previousPositionClaw;
    double deltaPositionClaw;
    double velocityClaw;

    double servoPosition;

    double recognitions;
    boolean forward = true;
    boolean back = true;
    boolean back2 = false;
    double counter = 0;

    double clawPower;

    double armSetDown;
    @Override
    public void runOpMode() {
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");

        motor0.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);


        position0 = motor0.getCurrentPosition() - start0;
        position1 = motor1.getCurrentPosition() - start1;
        position2 = motor2.getCurrentPosition() - start2;
        position3 = motor3.getCurrentPosition() - start3;

        servo0 = hardwareMap.get(CRServo.class, "servo0");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        arm0  = hardwareMap.get(DcMotor.class, "arm0");
        arm1  = hardwareMap.get(DcMotor.class, "arm1");

        analogInput = hardwareMap.get(AnalogInput.class, "servoEncoder");

        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward


        initTfod();

        waitForStart();

        imu.resetYaw();

        counter = 0;

        start = arm0.getCurrentPosition();
        position = arm0.getCurrentPosition() - start;



        startTime = System.nanoTime();
        updateTime();

        servoPosition = analogInput.getVoltage() / 3.3 * 360 + counter * 360;


        startClaw = servoPosition;
        positionClaw = servoPosition - startClaw;
        setPointClaw = servoPosition;
        errorClaw = setPointClaw - servoPosition;

        targetPosition0 = 0;
        targetPosition1 = 0.1;
        servo1.setPower(-targetPosition1);
        servo2.setPower(targetPosition1);

        start0 = motor0.getCurrentPosition();
        start1 = motor1.getCurrentPosition();
        start2 = motor2.getCurrentPosition();
        start3 = motor3.getCurrentPosition();

        counter = 0;

        boolean push = false;
        boolean pushFirst = true;
        boolean armDown = false;
        boolean armDownFirst = true;
        boolean left = false;
        boolean middle = false;
        boolean right = false;
        boolean move1 = true;
        boolean move1First = true;
        boolean strafe1 = false;
        boolean strafe1First = true;
        boolean move2 = false;
        boolean move2First = true;
        boolean move3 = false;
        boolean move3First = true;
        boolean move4 = false;
        boolean move4First = true;
        boolean move5 = false;
        boolean move5First = true;
        boolean move6 = false;
        boolean move6First = true;
        boolean move7 = false;
        boolean move7First = true;
        boolean move8 = false;
        boolean move8First = true;
        boolean move9 = false;
        boolean move9First = true;
        boolean move10 = false;
        boolean move10First = true;
        boolean move11 = false;
        boolean move11First = true;
        boolean move12 = false;
        boolean move12First = true;
        boolean move13 = false;
        boolean move13First = true;
        boolean move14 = false;
        boolean move14First = true;



        position();
        while (opModeIsActive()) {

            telemetryTfod();
            servoPosition = analogInput.getVoltage() / 3.3 * 360 + counter * 360;




            if (move1) {
                if(move1First){
                    start0 -= 200;
                    startClaw = servoPosition;
                    move1First = false;

                }
                setPoint0 = 600;
                setPoint1 = 600;
                setPoint2 = 600;
                setPoint3 = 600;
                if(position0 < error0){
                    motorPower0 = position0 * kp;
                    motorPower1 = position1 * kp;
                    motorPower2 = position2 * kp;
                    motorPower3 = position3 * kp;
                } else {
                    motorPower0 = error0 * kp;
                    motorPower1 = error1 * kp;
                    motorPower2 = error2 * kp;
                    motorPower3 = error3 * kp;
                }

                if((motorPower0 < 0.15 && motorPower0 > 0)){
                    motorPower0 = 0.15;
                }
                if((motorPower0 > -0.15 && motorPower0 < 0)){
                    motorPower0 = -0.15;
                }
                if((motorPower1 < 0.15 && motorPower1 > 0)){
                    motorPower1 = 0.15;
                }
                if((motorPower1 > -0.15 && motorPower1 < 0)){
                    motorPower1 = -0.15;
                }
                motor0.setPower(motorPower0);
                motor1.setPower(motorPower0);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower0);
                position();

                setPointClaw = startClaw;

                servo0.setPower(-(errorClaw *  kpClaw));
                if ((error0 <= 20 && error0 >= -20)) {
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                    move1 = false;
                    strafe1 = true;

                }

            }

            else if (strafe1) {
                if(strafe1First){
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    strafe1First = false;
                }
                setPoint0 = 500;
                setPoint1 = -500;
                setPoint2 = -500;
                setPoint3 = 500;
                if(position0 < error0){
                    motorPower0 = position0 * kp;
                    motorPower1 = position1 * kp;
                    motorPower2 = position2 * kp;
                    motorPower3 = position3 * kp;
                } else {
                    motorPower0 = error0 * kp;
                    motorPower1 = error1 * kp;
                    motorPower2 = error2 * kp;
                    motorPower3 = error3 * kp;
                }

                if((motorPower0 < 0.25 && motorPower0 > 0)){
                    motorPower0 = 0.25;
                }
                if((motorPower0 > -0.25 && motorPower0 < 0)){
                    motorPower0 = -0.25;
                }
                if((motorPower1 < 0.25 && motorPower1 > 0)){
                    motorPower1 = 0.25;
                }
                if((motorPower1 > -0.25 && motorPower1 < 0)){
                    motorPower1 = -0.25;
                }

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower1);
                motor3.setPower(motorPower0);
                position();
                if ((error0 <= 20 && error0 >= -20) || (error1 <= 20 && error1 >= -20)) {
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                    strafe1 = false;
                    move2 = true;
                }

            }
            else if(move2){
                if(move2First){
                    oldTime = runtime.seconds();

                    move2First = false;
                }
                newTime = runtime.seconds();
                setPointClaw = startClaw;
                if(recognitions >= 1){
                    middle = true;
                    move2 = false;
                    armDown = true;
                }
                if(newTime - oldTime >= 1.5){
                    move2 = false;
                    move3 = true;
                }
            }

            //turn left
            else if (move3) {
                if(move3First){
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    move3First = false;
                }
                double motorPower0 = -(headingError * kpA);
                double motorPower1 = (headingError * kpA);
                if((motorPower0 < 0.225 && motorPower0 > 0)){
                    motorPower0 = 0.225;
                }
                if((motorPower0 > -0.225 && motorPower0 < 0)){
                    motorPower0 = -0.225;
                }
                if((motorPower1 < 0.225 && motorPower1 > 0)){
                    motorPower1 = 0.225;
                }
                if((motorPower1 > -0.225 && motorPower1 < 0)){
                    motorPower1 = -0.225;
                }
                setYaw = 45;

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower1);
                position();

                setPointClaw = startClaw;

                servo0.setPower(-(errorClaw *  kpClaw));
                if (headingError <= 0.3 && headingError >= -0.3){
                    move3 = false;
                    move4 = true;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }
            }


            else if(move4){
                if(move4First){
                    oldTime = runtime.seconds();

                    move4First = false;
                }
                newTime = runtime.seconds();
                setPointClaw = startClaw;

                if(recognitions >= 1){
                    left = true;
                    move4 = false;
                    armDown = true;
                }
                if(newTime - oldTime >= 1.0){
                    move4 = false;
                    move5 = true;
                }
            }
            //turn right
            else if(move5 && !left){
                if(move5First){
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    move5First = false;
                }

                double motorPower0 = -(headingError * kpA);
                double motorPower1 = (headingError * kpA);
                if((motorPower0 < 0.225 && motorPower0 > 0)){
                    motorPower0 = 0.225;
                }
                if((motorPower0 > -0.225 && motorPower0 < 0)){
                    motorPower0 = -0.225;
                }
                if((motorPower1 < 0.225 && motorPower1 > 0)){
                    motorPower1 = 0.225;
                }
                if((motorPower1 > -0.225 && motorPower1 < 0)){
                    motorPower1 = -0.225;
                }
                setYaw = -25;

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower1);

                position();

                setPointClaw = startClaw;

                servo0.setPower(-(errorClaw *  kpClaw));


                if (headingError <= 0.3 && headingError >= -0.3){
                    move5 = false;
                    right = true;
                    armDown = true;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }
            }

            //turn right
            /*
            else if(move6 && !left && !middle){
                if(move6First){
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition()
                    move6First = false;
                }
                double motorPower0 = -(headingError * kpA);
                double motorPower1 = (headingError * kpA);
                if((motorPower0 < 0.125 && motorPower0 > 0)){
                    motorPower0 = 0.125;
                }
                if((motorPower0 > -0.125 && motorPower0 < 0)){
                    motorPower0 = -0.125;
                }
                if((motorPower1 < 0.125 && motorPower1 > 0)){
                    motorPower1 = 0.125;
                }
                if((motorPower1 > -0.125 && motorPower1 < 0)){
                    motorPower1 = -0.125;
                }
                setYaw = -90;

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower1);
                position();

                setPointClaw = startClaw;
                servo0.setPower(-(errorClaw *  kpClaw));


                if (headingError <= 0.15 && headingError >= -0.15){
                    move6 = false;
                    move7 = true;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }
            }*/

            //scan3
            /*else if(move7){
                if(move7First){
                    oldTime = runtime.seconds();

                    move7First = false;
                }
                newTime = runtime.seconds();
                setPointClaw = startClaw;
                servo0.setPower(-(errorClaw *  kpClaw));

                if(recognitions >= 1){
                    right = true;
                    move7 = false;
                    armDown = true;
                }
            }*/

            //arm down
            else if ((left || middle || right) && armDown){
                if(armDownFirst){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();

                    final double newSetPointClaw = servoPosition -= 50;
                    armSetDown = newSetPointClaw;

                    setPointClaw = newSetPointClaw;


                    armDownFirst = false;

                }

                if(middle){
                    setPoint0 = 0;
                    setPoint1 = 0;
                    setPoint2 = 0;
                    setPoint3 = 0;
                }
                if(right){
                    setPoint0 = -180;
                    setPoint1 = -180;
                    setPoint2 = -180;
                    setPoint3 = -180;
                }
                if(left){
                    setPoint0 = 0;
                    setPoint1 = 0;
                    setPoint2 = 0;
                    setPoint3 = 0;
                }


                motorPower0 = error0 * kp;
                motorPower1 = error1 * kp;
                motorPower2 = error2 * kp;
                motorPower3 = error3 * kp;



                position();

                motor0.setPower(error0 * kp);
                motor1.setPower(error1 * kp);
                motor2.setPower(error2 * kp);
                motor3.setPower(error3 * kp);



                newTime = runtime.seconds();
                setPoint = 0;
                previousPosition = position;
                position = arm0.getCurrentPosition() - start;
                deltaPosition = position - previousPosition;
                error = setPoint - position;
                arm0.setPower((error * kp - (kd * velocity)));
                arm1.setPower(-(error * kp - (kd * velocity)));



                servoPosition = analogInput.getVoltage() / 3.3 * 360 + counter * 360;
                errorClaw = setPointClaw - servoPosition;
                if(errorClaw >= 200 && forward){
                    counter += 1;
                    forward = false;
                    back = true;
                }
                if(errorClaw <= -200 && back){
                    counter -= 1;
                    back = false;
                    forward = true;
                    back2 = true;
                }
                if(errorClaw <= -200 && back2){
                    counter -= 1;
                    back = true;
                    back2 = false;
                }


                if(servoPosition > 0 && !(errorClaw <= 30 && errorClaw >= -30)){
                    clawPower = -0.75;

                } else {
                    servoPosition = analogInput.getVoltage() / 3.3 * 360 + counter * 360;

                    errorClaw = setPointClaw - servoPosition;

                    if(errorClaw <= -200 && back){
                        counter -= 1;
                        back = false;
                        forward = true;
                    }
                    clawPower = -(errorClaw * kpClaw);
                }
                servo0.setPower(clawPower);





                updateTime();

                velocity = deltaPosition / deltaTime;


                targetPosition0 = 0;
                targetPosition1 = 0.125;
                servo1.setPower(-targetPosition1);
                servo2.setPower(targetPosition1);

                if(newTime - oldTime >= 1.0){
                    servo0.setPower(0);
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                    armDown = false;
                    if(middle || left){
                        push = true;
                    }else{
                        move8 = true;
                    }

                }
            }
            //push forward
            else if (push){
                if(pushFirst){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();


                    pushFirst = false;

                }

                if(middle){
                    setPoint0 = 150;
                    setPoint1 = 150;
                    setPoint2 = 150;
                    setPoint3 = 150;
                }
                if(left){
                    setPoint0 = 150;
                    setPoint1 = 150;
                    setPoint2 = 150;
                    setPoint3 = 150;
                }




                motorPower0 = error0 * kp;
                motorPower1 = error1 * kp;
                motorPower2 = error2 * kp;
                motorPower3 = error3 * kp;



                position();

                motor0.setPower(error0 * kp);
                motor1.setPower(error1 * kp);
                motor2.setPower(error2 * kp);
                motor3.setPower(error3 * kp);



                newTime = runtime.seconds();
                setPoint = 0;
                previousPosition = position;
                position = arm0.getCurrentPosition() - start;
                deltaPosition = position - previousPosition;
                error = setPoint - position;
                arm0.setPower((error * kp - (kd * velocity)));
                arm1.setPower(-(error * kp - (kd * velocity)));



                servoPosition = analogInput.getVoltage() / 3.3 * 360 + counter * 360;
                errorClaw = setPointClaw - servoPosition;
                if(errorClaw >= 200 && forward){
                    counter += 1;
                    forward = false;
                    back = true;
                }
                if(errorClaw <= -200 && back){
                    counter -= 1;
                    back = false;
                    forward = true;
                    back2 = true;
                }
                if(errorClaw <= -200 && back2){
                    counter -= 1;
                    back = true;
                    back2 = false;
                }


                if(servoPosition > 0 && !(errorClaw <= 30 && errorClaw >= -30)){
                    clawPower = -0.75;

                } else {
                    servoPosition = analogInput.getVoltage() / 3.3 * 360 + counter * 360;

                    errorClaw = setPointClaw - servoPosition;

                    if(errorClaw <= -200 && back){
                        counter -= 1;
                        back = false;
                        forward = true;
                    }
                    clawPower = -(errorClaw * kpClaw);
                }
                servo0.setPower(0);





                updateTime();

                velocity = deltaPosition / deltaTime;


                targetPosition0 = 0;
                targetPosition1 = 0.125;
                servo1.setPower(-targetPosition1);
                servo2.setPower(targetPosition1);

                if(newTime - oldTime >= 1.0){
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                    servo0.setPower(0);
                    push = false;
                    move8 = true;
                }
            }
            //arm open
            else if (move8){
                if(move8First){
                    oldTime = runtime.seconds();
                    setPointClaw = armSetDown;

                    move8First = false;
                }

                newTime = runtime.seconds();
                setPoint = 0;
                previousPosition = position;
                position = arm0.getCurrentPosition() - start;
                deltaPosition = position - previousPosition;
                error = setPoint - position;
                arm0.setPower((error * kp - (kd * velocity)));
                arm1.setPower(-(error * kp - (kd * velocity)));




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
                errorClaw = setPointClaw - servoPosition;

                servo0.setPower(0);






                updateTime();

                velocity = deltaPosition / deltaTime;


                targetPosition0 = 0;
                servo0.setPower(0);
                targetPosition1 = 0.2;
                servo1.setPower(targetPosition1 * 2);
                servo2.setPower(targetPosition1);

                if(newTime - oldTime >= 1.0){
                    move8 = false;
                    move9 = true;
                }
            }
            //arm up
            else if (move9){
                if(move9First){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();

                    setPointClaw = startClaw - 20;
                    move9First = false;
                }
                if(middle){
                    setPoint0 = -150;
                    setPoint1 = -150;
                    setPoint2 = -150;
                    setPoint3 = -150;
                }
                if(right){
                    setPoint0 = 180;
                    setPoint1 = 180;
                    setPoint2 = 180;
                    setPoint3 = 180;
                }
                if(left){
                    setPoint0 = -150;
                    setPoint1 = -150;
                    setPoint2 = -150;
                    setPoint3 = -150;
                }


                motorPower0 = error0 * kp;
                motorPower1 = error1 * kp;
                motorPower2 = error2 * kp;
                motorPower3 = error3 * kp;

                position();

                motor0.setPower(error0 * kp);
                motor1.setPower(error1 * kp);
                motor2.setPower(error2 * kp);
                motor3.setPower(error3 * kp);

                newTime = runtime.seconds();
                setPoint = 0;
                previousPosition = position;
                position = arm0.getCurrentPosition() - start;
                deltaPosition = position - previousPosition;
                error = setPoint - position;
                arm0.setPower((error * kp - (kd * velocity)));
                arm1.setPower(-(error * kp - (kd * velocity)));



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
                errorClaw = setPointClaw - servoPosition;

                if(servoPosition < 10){
                    clawPower = 0.75;

                } else {
                    servoPosition = analogInput.getVoltage() / 3.3 * 360 + counter * 360;
                    errorClaw = setPointClaw - servoPosition;

                    if(errorClaw >= 200 && forward){
                        counter += 1;
                        forward = false;
                        back = true;
                    }
                    clawPower = -(errorClaw * kpClaw);
                }
                servo0.setPower(1);





                updateTime();

                velocity = deltaPosition / deltaTime;


                targetPosition0 = 0;



                targetPosition1 = 0.2;
                servo1.setPower(-targetPosition1 * 0.05);
                servo2.setPower(targetPosition1);

                if(newTime - oldTime >= 0.5){
                    servo0.setPower(0);
                }
                if(newTime - oldTime >= 1.0){
                    move9 = false;
                    move10 = true;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }
            }

            //turn left
            /*else if(!left && move10){
                if(move10First){
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    setPointClaw = servoPosition;
                    move10First = false;
                }
                double motorPower0 = -(headingError * kpA);
                double motorPower1 = headingError * kpA;
                if((motorPower0 < 0.125 && motorPower0 > 0)){
                    motorPower0 = 0.125;
                }
                if((motorPower0 > -0.125 && motorPower0 < 0)){
                    motorPower0 = -0.125;
                }
                if((motorPower1 < 0.125 && motorPower1 > 0)){
                    motorPower1 = 0.125;
                }
                if((motorPower1 > -0.125 && motorPower1 < 0)){
                    motorPower1 = -0.125;
                }
                setYaw = 90;

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower1);
                position();



                if (headingError <= 0.15 && headingError >= -0.15){
                    move10 = false;
                    move11 = true;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }
            } else if(left && move10){
                move10 = false;
                move11 = true;
            }
            */
            //strafe
            else if (move11) {
                if(move11First){
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    move11First = false;
                }
                setPoint0 = 1000;
                setPoint1 = -1000;
                setPoint2 = -1000;
                setPoint3 = 1000;
                if(position0 < error0){
                    motorPower0 = position0 * kp;
                    motorPower1 = position1 * kp;
                    motorPower2 = position2 * kp;
                    motorPower3 = position3 * kp;
                } else {
                    motorPower0 = error0 * kp;
                    motorPower1 = error1 * kp;
                    motorPower2 = error2 * kp;
                    motorPower3 = error3 * kp;
                }

                motor0.setPower(error0 * kp);
                motor1.setPower(error1 * kp);
                motor2.setPower(error2 * kp);
                motor3.setPower(error3 * kp);
                position();
                if ((error0 <= 10 && error0 >= -10) && (error1 <= 10 && error1 >= -10)) {
                    move11 = false;
                    move12 = true;
                }
            }
            /*
            // move forward
            else if(move12){
                if(move12First){
                    start0 -= 200;
                    move12First = false;
                }
                setPoint0 = 1800;
                setPoint1 = 1800;
                setPoint2 = 1800;
                setPoint3 = 1800;
                if(position0 < error0){
                    motorPower0 = position0 * kp;
                    motorPower1 = position1 * kp;
                    motorPower2 = position2 * kp;
                    motorPower3 = position3 * kp;
                } else {
                    motorPower0 = error0 * kp;
                    motorPower1 = error1 * kp;
                    motorPower2 = error2 * kp;
                    motorPower3 = error3 * kp;
                }
                motor0.setPower(motorPower0);
                motor1.setPower(motorPower0);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower0);
                position();
                if ((error0 <= 10 && error0 >= -10)) {
                    move12 = false;
                    move13 = true;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }

            }

            //strafe?
            else if(move13){



            }*/

            errorClaw = setPointClaw - servoPosition;


            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            /*telemetry.addData("error0 ", error0);
            telemetry.addData("error1 ", error1);
            telemetry.addData("position0 ", position0);
            telemetry.addData("position1 ", position1);
            telemetry.addData("start0 ", start0);*/
            telemetry.addData("Objects: ", recognitions);
            telemetry.addData("Yaw: ", orientation.getYaw(AngleUnit.DEGREES));
            //telemetry.addData("Time ", runtime.seconds());
            telemetry.addData("Position0 ", position0);
            telemetry.addData("Position1 ", position1);
            telemetry.addData("Position2 ", position2);
            telemetry.addData("Position3 ", position3);
            telemetry.addData("servoPosition ", servoPosition);
            telemetry.addData("setpoint ", setPointClaw);
            telemetry.addData("Servo Power ", servo0.getPower());
            telemetry.update();

        }
    }
    public void updateTime() {
        previousTime = time;
        time = ((double) (System.nanoTime() - startTime) ) / Math.pow(10,9);
        deltaTime = time - previousTime;
    }
    public void position() {
        position0 = motor0.getCurrentPosition() - start0;
        position1 = motor1.getCurrentPosition() - start1;
        position2 = motor2.getCurrentPosition() - start2;
        position3 = motor3.getCurrentPosition() - start3;
        error0 = setPoint0 - position0;
        error1 = setPoint1 - position1;
        error2 = setPoint2 - position2;
        error3 = setPoint3 - position3;

        servoPosition = analogInput.getVoltage() / 3.3 * 360 + counter * 360;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        headingError = setYaw - orientation.getYaw(AngleUnit.DEGREES);
    }
    public void initTfod() {

        tfod = new TfodProcessor.Builder()

            .setModelAssetName(TFOD_MODEL_ASSET)

            .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

            .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    public void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            recognitions = currentRecognitions.size();

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
}


