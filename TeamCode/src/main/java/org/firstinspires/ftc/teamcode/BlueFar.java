package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name="BlueFar", group="Linear OpMode")
public class BlueFar extends LinearOpMode {

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
    private ServoImplEx servo1 = null;
    private ServoImplEx servo2 = null;
    double targetPosition0;
    double targetPosition1;

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

    double previousPosition0;
    double deltaPosition0;
    double velocity0;

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
        servo1 = hardwareMap.get(ServoImplEx.class, "servo1");
        servo2 = hardwareMap.get(ServoImplEx.class, "servo2");

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

        targetPosition0 = 0.08;
        targetPosition1 = 0.92;
        servo1.setPosition(targetPosition0);
        servo2.setPosition(targetPosition1);

        start0 = motor0.getCurrentPosition();
        start1 = motor1.getCurrentPosition();
        start2 = motor2.getCurrentPosition();
        start3 = motor3.getCurrentPosition();

        counter = 0;


        boolean moveTurn = false;
        boolean moveTurnFirst = true;
        boolean moveForward = false;
        boolean moveForwardFirst = true;
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
        boolean move15 = false;
        boolean move15First = true;
        boolean move16 = false;
        boolean move16First = true;

        boolean turnCenter = false;
        boolean turnCenterFirst = true;


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
                setPoint0 = 550;
                setPoint1 = 550;
                setPoint2 = 550;
                setPoint3 = 550;
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
                    servo0.setPower(0);
                    move1 = false;
                    move3 = true;

                }

            }

            else if (move3) {
                if(move3First){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    move3First = false;
                }
                newTime = runtime.seconds();
                double motorPower0 = -(headingError * kpA);
                double motorPower1 = (headingError * kpA);
                if((motorPower0 < 0.2 && motorPower0 > 0)){
                    motorPower0 = 0.2;
                }
                if((motorPower0 > -0.2 && motorPower0 < 0)){
                    motorPower0 = -0.2;
                }
                if((motorPower1 < 0.2 && motorPower1 > 0)){
                    motorPower1 = 0.2;
                }
                if((motorPower1 > -0.2 && motorPower1 < 0)){
                    motorPower1 = -0.2;
                }
                setYaw = 53;

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower1);
                position();

                setPointClaw = startClaw;

                servo0.setPower(0);
                if ((headingError <= 0.3 && headingError >= -0.3) || newTime - oldTime >= 1.25){
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                    move3 = false;
                    move2 = true;

                }
            }
            else if(move2){
                if(move2First){
                    oldTime = runtime.seconds();
                    move2First = false;
                }
                newTime = runtime.seconds();
                telemetryTfod();
                if(recognitions >= 1 || left){
                    right = true;
                    move2 = false;
                    armDown = true;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }
                if(newTime - oldTime >= 2.75){
                    move2 = false;
                    move5 = true;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }
            }

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
                if((motorPower0 < 0.175 && motorPower0 > 0)){
                    motorPower0 = 0.175;
                }
                if((motorPower0 > -0.175 && motorPower0 < 0)){
                    motorPower0 = -0.175;
                }
                if((motorPower1 < 0.175 && motorPower1 > 0)){
                    motorPower1 = 0.175;
                }
                if((motorPower1 > -0.175 && motorPower1 < 0)){
                    motorPower1 = -0.175;
                }
                setYaw = -30;

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower1);

                position();

                setPointClaw = startClaw;

                servo0.setPower(-(errorClaw *  kpClaw));


                if (headingError <= 0.3 && headingError >= -0.3){
                    move5 = false;
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
                telemetryTfod();
                if(recognitions >= 1){
                    left = true;
                    move4 = false;
                    armDown = true;
                }
                if(newTime - oldTime >= 1.9){
                    move4 = false;
                    move6 = true;
                }
            }

            else if(move6){
                if(move6First){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    move6First = false;
                }
                newTime = runtime.seconds();
                double motorPower0 = -(headingError * kpA);
                double motorPower1 = (headingError * kpA);
                if((motorPower0 < 0.14 && motorPower0 > 0)){
                    motorPower0 = 0.14;
                }
                if((motorPower0 > -0.14 && motorPower0 < 0)){
                    motorPower0 = -0.14;
                }
                if((motorPower1 < 0.14 && motorPower1 > 0)){
                    motorPower1 = 0.14;
                }
                if((motorPower1 > -0.14 && motorPower1 < 0)){
                    motorPower1 = -0.14;
                }
                setYaw = 0;

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower1);
                position();

                setPointClaw = startClaw;
                servo0.setPower(-(errorClaw *  kpClaw));


                if (headingError <= 0.15 && headingError >= -0.15 || newTime - oldTime >= 2){
                    move6 = false;
                    armDown = true;
                    middle = true;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }
            }


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
                    setPoint0 = -250;
                    setPoint1 = -250;
                    setPoint2 = -250;
                    setPoint3 = -250;
                }
                if(right){
                    setPoint0 = -250;
                    setPoint1 = -250;
                    setPoint2 = -250;
                    setPoint3 = -250;
                }
                if(left){
                    setPoint0 = -400;
                    setPoint1 = -250;
                    setPoint2 = -400;
                    setPoint3 = -250;
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
                servo0.setPower(-1);





                updateTime();

                velocity = deltaPosition / deltaTime;


                targetPosition0 = 0.08;
                targetPosition1 = 0.92;
                servo1.setPosition(targetPosition0);
                servo2.setPosition(targetPosition1);

                if(!left){
                    if(newTime - oldTime >= 0.925){
                        servo0.setPower(0);
                        motor0.setPower(0);
                        motor1.setPower(0);
                        motor2.setPower(0);
                        motor3.setPower(0);
                        armDown = false;
                        push = true;

                    }
                } else {
                    if(newTime - oldTime >= 1){
                        servo0.setPower(0);
                        motor0.setPower(0);
                        motor1.setPower(0);
                        motor2.setPower(0);
                        motor3.setPower(0);
                        armDown = false;
                        push = true;

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
                    setPoint0 = 155;
                    setPoint1 = 155;
                    setPoint2 = 155;
                    setPoint3 = 155;
                }
                if(left){
                    setPoint0 = 370;
                    setPoint1 = 220;
                    setPoint2 = 370;
                    setPoint3 = 220;
                }
                if(right){
                    setPoint0 = 250;
                    setPoint1 = 250;
                    setPoint2 = 250;
                    setPoint3 = 250;
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



                servo0.setPower(0);





                updateTime();

                velocity = deltaPosition / deltaTime;


                targetPosition0 = 0.08;
                targetPosition1 = 0.92;
                servo1.setPosition(targetPosition0);
                servo2.setPosition(targetPosition1);

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





                updateTime();

                velocity = deltaPosition / deltaTime;


                targetPosition0 = 0.08;
                servo0.setPower(0);
                targetPosition1 = 0.54;
                servo1.setPosition(targetPosition0);
                servo2.setPosition(targetPosition1);

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
                    setPoint0 = -100;
                    setPoint1 = -100;
                    setPoint2 = -100;
                    setPoint3 = -100;
                }
                if(right){
                    setPoint0 = -200;
                    setPoint1 = -200;
                    setPoint2 = -200;
                    setPoint3 = -200;
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

                servo0.setPower(1);





                updateTime();

                velocity = deltaPosition / deltaTime;


                targetPosition0 = 0.08;
                targetPosition1 = 0.92;
                servo1.setPosition(targetPosition0);
                servo2.setPosition(targetPosition1);

                if(newTime - oldTime >= 1.15){
                    servo0.setPower(0);
                }
                if(newTime - oldTime >= 1.15){
                    move9 = false;
                    move10 = true;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }
            }
            else if(move10 && (right)){
                if(move10First){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    move10First = false;
                }
                newTime = runtime.seconds();
                double motorPower0 =-(headingError * kpA);
                double motorPower1 = (headingError * kpA);
                if((motorPower0 < 0.155 && motorPower0 > 0)){
                    motorPower0 = 0.155;
                }
                if((motorPower0 > -0.155 && motorPower0 < 0)){
                    motorPower0 = -0.155;
                }
                if((motorPower1 < 0.155 && motorPower1 > 0)){
                    motorPower1 = 0.155;
                }
                if((motorPower1 > -0.155 && motorPower1 < 0)){
                    motorPower1 = -0.155;
                }
                setYaw = -3;

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower1);
                position();

                setPointClaw = startClaw;
                servo0.setPower(0);


                if ((headingError <= 0.15 && headingError >= -0.15 && newTime - oldTime >= 1.0) || newTime - oldTime >= 1.5){
                    move10 = false;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                    moveForward = true;
                }
            }
            else if(move10 && left){
                if(move10First){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    move10First = false;
                }
                position();
                double motorPower0 = -(headingError * kpA);
                double motorPower1 = (headingError * kpA);
                if((motorPower0 < 0.14 && motorPower0 > 0)){
                    motorPower0 = 0.14;
                }
                if((motorPower0 > -0.14 && motorPower0 < 0)){
                    motorPower0 = -0.14;
                }
                if((motorPower1 < 0.14 && motorPower1 > 0)){
                    motorPower1 = 0.14;
                }
                if((motorPower1 > -0.14 && motorPower1 < 0)){
                    motorPower1 = -0.14;
                }
                newTime = runtime.seconds();
                setYaw = -2;

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower1);
                position();

                setPointClaw = startClaw;
                servo0.setPower(0);


                if ((headingError <= 0.15 && headingError >= -0.15 && newTime - oldTime >= 1.5) || newTime - oldTime >= 2){
                    move10 = false;
                    servo0.setPower(0);
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                    moveForward = true;
                }
            }

            else if ((move10 && middle)){
                if(move10First){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    move10First = false;
                }
                newTime = runtime.seconds();
                setPoint0 = 400;
                setPoint1 = -400;
                setPoint2 = -400;
                setPoint3 = 400;


                motorPower0 = 0.5;
                motorPower1 = -0.5;
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
                if ((error0 <= 20 && error0 >= -20) || (error1 <= 20 && error1 >= -20) || newTime - oldTime >= 1) {
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                    servo0.setPower(0);
                    move10 = false;
                    moveForward = true;
                }

            }

            else if(moveForward){
                if(moveForwardFirst){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    start0 -= 150;
                    startClaw = servoPosition;
                    moveForwardFirst = false;

                }
                updateTime();
                previousPosition0 = position0;
                position0 = motor0.getCurrentPosition() - start0;
                deltaPosition0 = position0 - previousPosition0;
                velocity0 = deltaPosition0 / deltaTime;

                if(middle){
                    setPoint0 = 1640;
                    setPoint1 = 1640;
                    setPoint2 = 1640;
                    setPoint3 = 1640;
                }
                if(left){
                    setPoint0 = 1480;
                    setPoint1 = 1480;
                    setPoint2 = 1480;
                    setPoint3 = 1480;
                }
                if(right){
                    setPoint0 = 1490;
                    setPoint1 = 1490;
                    setPoint2 = 1490;
                    setPoint3 = 1490;
                }

                newTime = runtime.seconds();
                position();
                if(position0 < error0 - 400){
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

                motor0.setPower((motorPower0 - (velocity0 * 1.0/600)) * 0.5);
                motor1.setPower((motorPower0 - (velocity0 * 1.0/600)) * 0.5);
                motor2.setPower((motorPower0 - (velocity0 * 1.0/600)) * 0.5);
                motor3.setPower((motorPower0 - (velocity0 * 1.0/600)) * 0.5);
                position();

                setPointClaw = startClaw;


                servo0.setPower(0);
                if (((error0 <= 20 && error0 >= -20) && newTime - oldTime >= 2.0) || newTime - oldTime >= 2.5){
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                    servo0.setPower(0);
                    moveForward = false;
                    moveTurn = true;

                }
            }
            else if(moveTurn){
                if(moveTurnFirst){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    setPointClaw = servoPosition;
                    moveTurnFirst = false;
                }
                newTime = runtime.seconds();
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



                if ((headingError <= 0.15 && headingError >= -0.15 && newTime - oldTime >= 1.65) || newTime - oldTime >= 2.5){
                    moveTurn = false;
                    move11 = true;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }
            }
            else if(move11){
                if(move11First){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    start0 -= 200;
                    startClaw = servoPosition;
                    move11First = false;

                }
                updateTime();
                newTime = runtime.seconds();
                previousPosition0 = position0;
                position0 = motor0.getCurrentPosition() - start0;
                deltaPosition0 = position0 - previousPosition0;
                velocity0 = deltaPosition0 / deltaTime;

                if(middle){
                    setPoint0 = 3070;
                    setPoint1 = 3070;
                    setPoint2 = 3070;
                    setPoint3 = 3070;
                } else if(left){
                    setPoint0 = 2700;
                    setPoint1 = 2700;
                    setPoint2 = 2700;
                    setPoint3 = 2700;
                }
                else if(right){
                    setPoint0 = 2920;
                    setPoint1 = 2920;
                    setPoint2 = 2920;
                    setPoint3 = 2920;
                }
                if(position0 < error0 - 600){
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


                motor0.setPower((motorPower0 - (velocity0 * 1.0/600)) * 0.5);
                motor1.setPower((motorPower0 - (velocity0 * 1.0/600)) * 0.5);
                motor2.setPower((motorPower0 - (velocity0 * 1.0/600)) * 0.5);
                motor3.setPower((motorPower0 - (velocity0 * 1.0/600)) * 0.5);
                position();

                setPointClaw = startClaw;


                servo0.setPower(-(errorClaw *  kpClaw));
                if (((error0 <= 20 && error0 >= -20) && newTime - oldTime >= 2.5) || newTime - oldTime >= 4) {
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                    servo0.setPower(0);
                    move11 = false;
                    turnCenter = true;

                }
            }
            else if(turnCenter){
                if(turnCenterFirst){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    turnCenterFirst = false;
                }
                newTime = runtime.seconds();
                double motorPower0 = -(headingError * kpA);
                double motorPower1 = (headingError * kpA);
                if((motorPower0 < 0.17 && motorPower0 > 0)){
                    motorPower0 = 0.17;
                }
                if((motorPower0 > -0.17 && motorPower0 < 0)){
                    motorPower0 = -0.17;
                }
                if((motorPower1 < 0.17 && motorPower1 > 0)){
                    motorPower1 = 0.17;
                }
                if((motorPower1 > -0.17 && motorPower1 < 0)){
                    motorPower1 = -0.17;
                }
                setYaw = 90;

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower1);
                position();

                setPointClaw = startClaw;
                servo0.setPower(-(errorClaw *  kpClaw));


                if (headingError <= 0.15 && headingError >= -0.15 || newTime - oldTime >= 0.5){
                    turnCenter = false;
                    move12 = true;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }
            }
            else if(move12){
                if(move12First){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    start0 += 50;
                    start1 -= 50;
                    move12First = false;
                }
                position();
                newTime = runtime.seconds();
                if(middle){
                    setPoint0 = -750;
                    setPoint1 = 750;
                    setPoint2 = 750;
                    setPoint3 = -750;
                }
                else if(left){
                    setPoint0 = -470;
                    setPoint1 = 470;
                    setPoint2 = 470;
                    setPoint3 = -470;
                }
                else if(right){
                    setPoint0 = -600;
                    setPoint1 = 600;
                    setPoint2 = 600;
                    setPoint3 = -600;
                }


                position();
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
                if(left || middle) {
                    motor0.setPower(motorPower0);
                    motor1.setPower(motorPower1);
                    motor2.setPower(motorPower1);
                    motor3.setPower(motorPower0);
                }
                if(right){
                    motor0.setPower(-0.4);
                    motor1.setPower(0.4);
                    motor2.setPower(0.4);
                    motor3.setPower(-0.4);
                }
                if(left) {
                    if (newTime - oldTime >= 0.405) {
                        motor0.setPower(0);
                        motor1.setPower(0);
                        motor2.setPower(0);
                        motor3.setPower(0);
                        servo0.setPower(0);
                        move12 = false;
                        move13 = true;
                    }
                }
                if(middle){
                    if (newTime - oldTime >= 0.6) {
                        motor0.setPower(0);
                        motor1.setPower(0);
                        motor2.setPower(0);
                        motor3.setPower(0);
                        servo0.setPower(0);
                        move12 = false;
                        move13 = true;
                    }
                }
                if(right){
                    if (newTime - oldTime >= 1.83) {
                        motor0.setPower(0);
                        motor1.setPower(0);
                        motor2.setPower(0);
                        motor3.setPower(0);
                        servo0.setPower(0);
                        move12 = false;
                        move13 = true;
                    }
                }


            }

            else if(move13){
                if(move13First){

                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    start0 -= 100;
                    startClaw = servoPosition;
                    move13First = false;

                }
                position();
                newTime = runtime.seconds();
                if(left){
                    setPoint0 = 450;
                    setPoint1 = 450;
                    setPoint2 = 450;
                    setPoint3 = 450;
                    motorPower0 = 0.355;
                }
                if(middle){
                    setPoint0 = 450;
                    setPoint1 = 450;
                    setPoint2 = 450;
                    setPoint3 = 450;
                    motorPower0 = 0.32;
                }
                else if(right){
                    setPoint0 = 200;
                    setPoint1 = 200;
                    setPoint2 = 200;
                    setPoint3 = 200;
                    motorPower0 = 0.335;
                }

                setPoint = 365;
                previousPosition = position;
                position = arm0.getCurrentPosition() - start;
                deltaPosition = position - previousPosition;
                error = setPoint - position;
                arm0.setPower((error * kp - (kd * velocity)));
                arm1.setPower(-(error * kp - (kd * velocity)));

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
                motor0.setPower(motorPower0 * 0.6);
                motor1.setPower(motorPower0 * 0.6);
                motor2.setPower(motorPower0 * 0.6);
                motor3.setPower(motorPower0 * 0.6);
                position();

                setPointClaw = startClaw;


                if (((error0 <= 20 && error0 >= -20) && newTime - oldTime >= 1.75) || newTime - oldTime >= 1.0) {
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                    servo0.setPower(0);
                    move13 = false;
                    move14 = true;

                }
            }

            else if(move14){
                if(move14First){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();

                    start0 += 50;
                    start1 -= 50;
                    final double newSetPointClaw = servoPosition -= 50;
                    armSetDown = newSetPointClaw;

                    setPointClaw = newSetPointClaw;


                    move14First = false;

                }
                newTime = runtime.seconds();
                position();

                setPoint = 365;
                previousPosition = position;
                position = arm0.getCurrentPosition() - start;
                deltaPosition = position - previousPosition;
                error = setPoint - position;
                arm0.setPower((error * kp - (kd * velocity)));
                arm1.setPower(-(error * kp - (kd * velocity)));
                if(middle){
                    setPoint0 = 0;
                    setPoint1 = 0;
                    setPoint2 = 0;
                    setPoint3 = 0;
                }
                if(right){
                    setPoint0 = 0;
                    setPoint1 = 0;
                    setPoint2 = 0;
                    setPoint3 = 0;
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



                if(newTime - oldTime >= 0.31){
                    servo0.setPower(-1);
                }




                updateTime();

                velocity = deltaPosition / deltaTime;


                targetPosition0 = 0.08;
                targetPosition1 = 0.92;
                servo1.setPosition(targetPosition0);
                servo2.setPosition(targetPosition1);

                if(newTime - oldTime >= 1.0){
                    servo0.setPower(0);
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                    move14 = false;
                    move15 = true;
                }
            }
            if(move15){
                if(move15First){
                    oldTime = runtime.seconds();
                    setPointClaw = armSetDown;

                    move15First = false;
                }





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

                newTime = runtime.seconds();


                setPoint = 365;
                previousPosition = position;
                position = arm0.getCurrentPosition() - start;
                deltaPosition = position - previousPosition;
                error = setPoint - position;
                arm0.setPower((error * kp - (kd * velocity)));
                arm1.setPower(-(error * kp - (kd * velocity)));


                updateTime();

                velocity = deltaPosition / deltaTime;


                targetPosition0 = 0.55;
                servo0.setPower(0);
                targetPosition1 = 0.92;
                servo2.setPosition(targetPosition1);
                if(left || middle){
                    servo1.setPosition(targetPosition0);
                }
                if(right){
                    servo1.setPosition(targetPosition0);
                }


                if(newTime - oldTime >= 1.85){
                    move15 = false;
                    move16 = true;
                }
            }
            if(move16){
                if(move16First){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();

                    setPointClaw = startClaw - 20;
                    move16First = false;
                }



                motorPower0 = error0 * kp;
                motorPower1 = error1 * kp;
                motorPower2 = error2 * kp;
                motorPower3 = error3 * kp;

                position();


                newTime = runtime.seconds();
                setPoint = 365;
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

                servo0.setPower(1);





                updateTime();

                velocity = deltaPosition / deltaTime;


                targetPosition0 = 0.08;



                targetPosition1 = 0.92;
                servo1.setPosition(targetPosition0);
                servo2.setPosition(targetPosition1);

                if(newTime - oldTime >= 1.25){
                    move16 = false;
                    servo0.setPower(0);
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }
            }




            errorClaw = setPointClaw - servoPosition;
            position();

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            telemetry.addData("Objects: ", recognitions);
            telemetry.addData("Yaw: ", orientation.getYaw(AngleUnit.DEGREES));
            //telemetry.addData("Time ", runtime.seconds());
            telemetry.addData("Position0 ", position0);
            telemetry.addData("Position1 ", position1);
            telemetry.addData("Position2 ", position2);
            telemetry.addData("Position3 ", position3);
            telemetry.addData("setPoint0 ", setPoint0);
            telemetry.addData("setPoint1 ", setPoint1);
            telemetry.addData("setPoint2 ", setPoint2);
            telemetry.addData("setPoint3 ", setPoint3);
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
        recognitions = currentRecognitions.size();

    }   // end method telemetryTfod()
}


