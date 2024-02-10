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
    
    //movements are usually over 500
    double kp = 1.0 / 500;
    double start0 = 0;
    double start1 = 0;
    double start2 = 0;
    double start3 = 0;
    double setPoint0 = 0;
    double setPoint1 = 0;
    double setPoint2 = 0;
    double setPoint3 = 0;
    //position = current position - start position 
    //start position is arbitrary (can be negative)
    double position0;
    double position1;
    double position2;
    double position3;
    //error = setPointArm - position
    double error0;
    double error1;
    double error2;
    double error3;
    //strafes use motorPower combined with a time limit since encoder ticks are weird (makes PID malfunction on strafes)
    double motorPower0;
    double motorPower1;
    double motorPower2;
    double motorPower3;
    //setYaw - currentYaw
    double headingError;
    double setYaw;
    double kpHeading = 1.0/100;

    private CRServo servo0 = null;
    //servo 1 and 2 are for claw fingers
    //left and right when facing back of the robot
    private ServoImplEx servoLeft = null;
    private ServoImplEx servoRight = null;
    //targetPosition is position for claw fingers
    double targetPositionLeft;
    double targetPositionRight;
    private DcMotor arm0 = null;
    private DcMotor arm1 = null;
    double kpArm = 1.0/100;
    double kd = 0.01/120;
    double setPointArm = 0;
    double positionArm;
    double startArm = 0;
    double errorArm = setPointArm - positionArm;

    double startTime;
    double previousTime;
    double time;
    double deltaTime;

    double previousPositionArm;
    double deltaPositionArm;
    double velocityArm;

    //0 for motor0
    //using motor0 for all drivetrain motors on only forward movements
    double previousPosition0;
    double deltaPosition0;
    double velocity0;

    double oldTime;
    double newTime;
    IMU imu;
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;

    private static final boolean USE_WEBCAM = true;

    //tfod is all the camera processes
    private TfodProcessor tfod;

    private VisionPortal visionPortal;
    private static final String TFOD_MODEL_ASSET = "BlueBox.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/BlueBox.tflite";
    private static final String[] LABELS = {
            "BB",
    };

    double recognitions;
    double clawPower;

    final double closedLeft = 0.08;
    final double closedRight = 0.92;
    @Override
    public void runOpMode() {
        //0 is front left 
        //1 is front right
        //2 is back left
        //3 is back right
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
        servoLeft = hardwareMap.get(ServoImplEx.class, "servoLeft");
        servoRight = hardwareMap.get(ServoImplEx.class, "servoRight");

        arm0  = hardwareMap.get(DcMotor.class, "arm0");
        arm1  = hardwareMap.get(DcMotor.class, "arm1");


        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward


        initTfod();

        waitForStart();

        imu.resetYaw();

        startArm = arm0.getCurrentPosition();
        positionArm = arm0.getCurrentPosition() - startArm;
        
        startTime = System.nanoTime();
        updateTime();
        
       
        targetPositionLeft = closedLeft;
        targetPositionRight = closedRight;
        servoLeft.setPosition(targetPositionLeft);
        servoRight.setPosition(targetPositionRight);

        start0 = motor0.getCurrentPosition();
        start1 = motor1.getCurrentPosition();
        start2 = motor2.getCurrentPosition();
        start3 = motor3.getCurrentPosition();

        position0 = motor0.getCurrentPosition() - start0;
        position1 = motor1.getCurrentPosition() - start1;
        position2 = motor2.getCurrentPosition() - start2;
        position3 = motor3.getCurrentPosition() - start3;



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
        boolean move17 = false;
        boolean move17First = true;
        boolean moveTurn = false;
        boolean moveTurnFirst = true;
        boolean moveForward = false;
        boolean moveForwardFirst = true;
        boolean push = false;
        boolean pushFirst = true;
        boolean armDown = false;
        boolean armDownFirst = true;
        boolean far = false;
        boolean middle = false;
        boolean close = false;
        boolean turnCenter = false;
        boolean turnCenterFirst = true;


        position();
        while (opModeIsActive()) {
            telemetryTfod();

            if (move1) {
                if(move1First){
                    //setting start to behind the robot allows the power to start at a number greater than 0
                    start0 -= 200;
                    move1First = false;
                }
                setPoint0 = 570;
                setPoint1 = 570;
                setPoint2 = 570;
                setPoint3 = 570;
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

                setMinimumPower(0.15);

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower0);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower0);
                position();

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

            //turn to left tape
            else if (move2) {
                if(move2First){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    move2First = false;
                }
                newTime = runtime.seconds();
                double motorPower0 = -(headingError * kpHeading);
                double motorPower1 = (headingError * kpHeading);
                setMinimumPower(0.13);
                setYaw = 48.5;

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower1);
                position();


                servo0.setPower(0);
                if ((headingError <= 0.3 && headingError >= -0.3) || newTime - oldTime >= 1.5){
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                    move2 = false;
                    move3 = true;

                }
            }
            //scan
            else if(move3){
                if(move3First){
                    oldTime = runtime.seconds();
                    move3First = false;
                }
                newTime = runtime.seconds();
                telemetryTfod();
                if(recognitions >= 1){
                    close = true;
                    move3 = false;
                    armDown = true;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }
                if(newTime - oldTime >= 2.55){
                    move3 = false;
                    move4 = true;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }
            }

            //turn to right tape
            else if(move4 && !far){
                if(move4First){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    move4First = false;
                }
                newTime = runtime.seconds();
                double motorPower0 = -(headingError * kpHeading);
                double motorPower1 = (headingError * kpHeading);
                setMinimumPower(0.125);
                setYaw = -30;

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower1);

                position();


                if (headingError <= 0.3 && headingError >= -0.3 && newTime - oldTime >= 0.5 || newTime - oldTime >= 1.1){
                    move4 = false;
                    move5 = true;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }
            }

            //scan
            else if(move5){
                if(move5First){
                    oldTime = runtime.seconds();
                    move5First = false;
                }
                newTime = runtime.seconds();
                telemetryTfod();
                if(recognitions >= 1){
                    far = true;
                    move5 = false;
                    armDown = true;
                }
                if(newTime - oldTime >= 1.8){
                    move5 = false;
                    move6 = true;
                }
            }

            //turn to center tape
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
                double motorPower0 = -(headingError * kpHeading);
                double motorPower1 = (headingError * kpHeading);
                setMinimumPower(0.13);
                setYaw = 0;

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower1);
                position();


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
            else if ((far || middle || close) && armDown){
                if(armDownFirst){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    armDownFirst = false;
                }

                if(middle){
                    setPoint0 = -250;
                    setPoint1 = -250;
                    setPoint2 = -250;
                    setPoint3 = -250;
                }
                if(close){
                    setPoint0 = -250;
                    setPoint1 = -250;
                    setPoint2 = -250;
                    setPoint3 = -250;
                }
                //backs up to the side to avoid trusses
                if(far){
                    setPoint0 = -440;
                    setPoint1 = -280;
                    setPoint2 = -440;
                    setPoint3 = -280;
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
                setPointArm = 0;
                arm0.setPower((errorArm * kp - (kd * velocityArm)));
                arm1.setPower(-(errorArm * kp - (kd * velocityArm)));


                servo0.setPower(-1);





                updateTime();

                velocityArm = deltaPositionArm / deltaTime;


                targetPositionLeft = closedLeft;
                targetPositionRight = closedRight;
                servoLeft.setPosition(targetPositionLeft);
                servoRight.setPosition(targetPositionRight);

                if(!far){
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
                    setPoint0 = 205;
                    setPoint1 = 205;
                    setPoint2 = 205;
                    setPoint3 = 205;
                }
                if(far){
                    setPoint0 = 390;
                    setPoint1 = 240;
                    setPoint2 = 390;
                    setPoint3 = 240;
                }
                if(close){
                    setPoint0 = 242;
                    setPoint1 = 242;
                    setPoint2 = 242;
                    setPoint3 = 242;
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
                setPointArm = 0;
                arm0.setPower((errorArm * kp - (kd * velocityArm)));
                arm1.setPower(-(errorArm * kp - (kd * velocityArm)));


                servo0.setPower(0);





                updateTime();

                velocityArm = deltaPositionArm / deltaTime;


                targetPositionLeft = closedLeft;
                targetPositionRight = closedRight;
                servoLeft.setPosition(targetPositionLeft);
                servoRight.setPosition(targetPositionRight);

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

                    move8First = false;
                }

                newTime = runtime.seconds();
                setPointArm = 0;
                arm0.setPower((errorArm * kp - (kd * velocityArm)));
                arm1.setPower(-(errorArm * kp - (kd * velocityArm)));

                updateTime();

                velocityArm = deltaPositionArm / deltaTime;


                targetPositionLeft = closedLeft;
                servo0.setPower(0);
                targetPositionRight = 0.54;
                servoLeft.setPosition(targetPositionLeft);
                servoRight.setPosition(targetPositionRight);

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

                    move9First = false;
                }
                if(middle){
                    setPoint0 = -100;
                    setPoint1 = -100;
                    setPoint2 = -100;
                    setPoint3 = -100;
                }
                if(close){
                    setPoint0 = -200;
                    setPoint1 = -200;
                    setPoint2 = -200;
                    setPoint3 = -200;
                }
                if(far){
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
                setPointArm = 0;
                arm0.setPower((errorArm * kp - (kd * velocityArm)));
                arm1.setPower(-(errorArm * kp - (kd * velocityArm)));



                servo0.setPower(1);





                updateTime();

                velocityArm = deltaPositionArm / deltaTime;


                targetPositionLeft = closedLeft;
                targetPositionRight = closedRight;
                servoLeft.setPosition(targetPositionLeft);
                servoRight.setPosition(targetPositionRight);

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
            else if(move10 && (close)){
                if(move10First){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    move10First = false;
                }
                newTime = runtime.seconds();
                double motorPower0 =-(headingError * kpHeading);
                double motorPower1 = (headingError * kpHeading);
                setMinimumPower(0.1);
                setYaw = -3;

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower1);
                position();

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
            else if(move10 && far){
                if(move10First){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    move10First = false;
                }
                position();
                double motorPower0 = -(headingError * kpHeading);
                double motorPower1 = (headingError * kpHeading);
                setMinimumPower(0.134);
                newTime = runtime.seconds();
                setYaw = -2;

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower1);
                position();

                servo0.setPower(0);


                if ((headingError <= 0.15 && headingError >= -0.15 && newTime - oldTime >= 1.55) || newTime - oldTime >= 2.6){
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
                setMinimumPower(0.25);

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower1);
                motor3.setPower(motorPower0);
                position();
                if (newTime - oldTime >= 0.55) {
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
                    moveForwardFirst = false;

                }
                updateTime();
                previousPosition0 = position0;
                position0 = motor0.getCurrentPosition() - start0;
                deltaPosition0 = position0 - previousPosition0;
                velocity0 = deltaPosition0 / deltaTime;

                if(middle){
                    setPoint0 = 1580;
                    setPoint1 = 1580;
                    setPoint2 = 1580;
                    setPoint3 = 1580;
                }
                if(far){
                    setPoint0 = 1540;
                    setPoint1 = 1540;
                    setPoint2 = 1540;
                    setPoint3 = 1540;
                }
                if(close){
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

                setMinimumPower(0.25);

                motor0.setPower((motorPower0 - (velocity0 * 1.0/600)) * 0.5);
                motor1.setPower((motorPower0 - (velocity0 * 1.0/600)) * 0.5);
                motor2.setPower((motorPower0 - (velocity0 * 1.0/600)) * 0.5);
                motor3.setPower((motorPower0 - (velocity0 * 1.0/600)) * 0.5);
                position();


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
                    moveTurnFirst = false;
                }
                newTime = runtime.seconds();
                double motorPower0 = -(headingError * kpHeading);
                double motorPower1 = headingError * kpHeading;
                setMinimumPower(0.105);
                setYaw = 90;

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower1);
                position();



                if ((headingError <= 0.15 && headingError >= -0.15 && newTime - oldTime >= 1.5) || newTime - oldTime >= 2){
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
                } else if(far){
                    setPoint0 = 2700;
                    setPoint1 = 2700;
                    setPoint2 = 2700;
                    setPoint3 = 2700;
                }
                else if(close){
                    setPoint0 = 2770;
                    setPoint1 = 2770;
                    setPoint2 = 2770;
                    setPoint3 = 2770;
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
                setMinimumPower(0.25);

                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();


                motor0.setPower((motorPower0 - (velocity0 * 1.0/600) - headingError * 0.0125) * 0.5);
                motor1.setPower((motorPower0 - (velocity0 * 1.0/600) + headingError * 0.0125) * 0.5);
                motor2.setPower((motorPower0 - (velocity0 * 1.0/600) - headingError * 0.0125) * 0.5);
                motor3.setPower((motorPower0 - (velocity0 * 1.0/600) + headingError * 0.0125) * 0.5);

                position();

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
                double motorPower0 = -(headingError * kpHeading);
                double motorPower1 = (headingError * kpHeading);
                setMinimumPower(0.14);
                setYaw = 90;

                motor0.setPower(motorPower0);
                motor1.setPower(motorPower1);
                motor2.setPower(motorPower0);
                motor3.setPower(motorPower1);
                position();


                if (headingError <= 0.15 && headingError >= -0.15 || newTime - oldTime >= 0.65){
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
                    setPoint0 = -670;
                    setPoint1 = 670;
                    setPoint2 = 670;
                    setPoint3 = -670;
                    motorPower0 = -0.43;
                    motorPower1 = 0.43;
                }
                else if(far){
                    setPoint0 = -470;
                    setPoint1 = 470;
                    setPoint2 = 470;
                    setPoint3 = -470;
                    motorPower0 = -0.3515;
                    motorPower1 = 0.3515;
                }
                else if(close){
                    setPoint0 = -600;
                    setPoint1 = 600;
                    setPoint2 = 600;
                    setPoint3 = -600;

                    motorPower0 = -0.498;
                    motorPower1 = 0.498;
                }


                position();
                setMinimumPower(0.25);

                    motor0.setPower(motorPower0 + headingError * 0.0125);
                    motor1.setPower(motorPower1 + headingError * 0.0125);
                    motor2.setPower(motorPower1 - headingError * 0.0125);
                    motor3.setPower(motorPower0 - headingError * 0.0125);
                if(far) {
                    if (newTime - oldTime >= 1.2) {
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
                    if (newTime - oldTime >= 1.275) {
                        motor0.setPower(0);
                        motor1.setPower(0);
                        motor2.setPower(0);
                        motor3.setPower(0);
                        servo0.setPower(0);
                        move12 = false;
                        move13 = true;
                    }
                }
                if(close){
                    if (newTime - oldTime >= 1.2) {
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
                    move13First = false;

                }
                position();
                newTime = runtime.seconds();
                if(far){
                    setPoint0 = 450;
                    setPoint1 = 450;
                    setPoint2 = 450;
                    setPoint3 = 450;
                    motorPower0 = 0.339;
                }
                if(middle){
                    setPoint0 = 450;
                    setPoint1 = 450;
                    setPoint2 = 450;
                    setPoint3 = 450;
                    motorPower0 = 0.36;
                }
                else if(close){
                    setPoint0 = 200;
                    setPoint1 = 200;
                    setPoint2 = 200;
                    setPoint3 = 200;
                    motorPower0 = 0.3425;
                }

                setPointArm = 335;
                arm0.setPower((errorArm * kp - (kd * velocityArm)));
                arm1.setPower(-(errorArm * kp - (kd * velocityArm)));

                setMinimumPower(0.25);
                motor0.setPower(motorPower0 * 0.6);
                motor1.setPower(motorPower0 * 0.6);
                motor2.setPower(motorPower0 * 0.6);
                motor3.setPower(motorPower0 * 0.6);
                position();


                if (((error0 <= 20 && error0 >= -20) && newTime - oldTime >= 1.75) || newTime - oldTime >= 1) {
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


                    move14First = false;

                }
                newTime = runtime.seconds();
                position();

                setPointArm = 335;
                arm0.setPower((errorArm * kp - (kd * velocityArm)));
                arm1.setPower(-(errorArm * kp - (kd * velocityArm)));
                if(middle){
                    setPoint0 = 0;
                    setPoint1 = 0;
                    setPoint2 = 0;
                    setPoint3 = 0;
                }
                if(close){
                    setPoint0 = 0;
                    setPoint1 = 0;
                    setPoint2 = 0;
                    setPoint3 = 0;
                }
                if(far){
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


                if(middle){
                    if(newTime - oldTime >= 0.3375){
                        servo0.setPower(-1);
                    }

                } else if(close){
                    if(newTime - oldTime >= 0.315){
                        servo0.setPower(-1);
                    }

                } else {
                    if(newTime - oldTime >= 0.2575){
                        servo0.setPower(-1);
                    }
                }




                updateTime();

                velocityArm = deltaPositionArm / deltaTime;


                targetPositionLeft = closedLeft;
                targetPositionRight = closedRight;
                servoLeft.setPosition(targetPositionLeft);
                servoRight.setPosition(targetPositionRight);

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
            else if(move15){
                if(move15First){
                    oldTime = runtime.seconds();

                    move15First = false;
                }





                servo0.setPower(0);

                newTime = runtime.seconds();


                setPointArm = 260;
                arm0.setPower((errorArm * kp - (kd * velocityArm)));
                arm1.setPower(-(errorArm * kp - (kd * velocityArm)));


                updateTime();

                velocityArm = deltaPositionArm / deltaTime;

                if(!far){
                    if(targetPositionLeft < 0.55){
                        targetPositionLeft += 0.009;
                    }
                } else {
                    if(targetPositionLeft < 0.55){
                        targetPositionLeft += 0.012;
                    }
                }

                servo0.setPower(0);
                targetPositionRight = closedRight;
                servoRight.setPosition(targetPositionRight);
                if(far || middle){
                    servoLeft.setPosition(targetPositionLeft);
                }
                if(close){
                    servoLeft.setPosition(targetPositionLeft);
                }

                if(middle){
                    if(newTime - oldTime >= 0.5){
                        move15 = false;
                        move17 = true;
                    }
                } else if(close){
                    if(newTime - oldTime >= 0.75){
                        move15 = false;
                        move17 = true;
                    }
                } else {
                    if(newTime - oldTime >= 0.6){
                        move15 = false;
                        move17 = true;
                    }
                }

            }
            else if(move17){
                if(move17First){

                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    start0 += 50;

                    move17First = false;

                }
                position();
                newTime = runtime.seconds();
                if(far){
                    setPoint0 = 450;
                    setPoint1 = 450;
                    setPoint2 = 450;
                    setPoint3 = 450;
                    motorPower0 = -0.29;
                }
                if(middle){
                    setPoint0 = 450;
                    setPoint1 = 450;
                    setPoint2 = 450;
                    setPoint3 = 450;
                    motorPower0 = -0.27;
                }
                else if(close){
                    setPoint0 = 200;
                    setPoint1 = 200;
                    setPoint2 = 200;
                    setPoint3 = 200;
                    motorPower0 = -0.27;
                }

                setPointArm = 260;
                arm0.setPower((errorArm * kp - (kd * velocityArm)));
                arm1.setPower(-(errorArm * kp - (kd * velocityArm)));

                setMinimumPower(0.25);
                motor0.setPower(motorPower0 * 0.6);
                motor1.setPower(motorPower0 * 0.6);
                motor2.setPower(motorPower0 * 0.6);
                motor3.setPower(motorPower0 * 0.6);
                position();


                if(!far){
                    if (((error0 <= 20 && error0 >= -20) && newTime - oldTime >= 0.25) || newTime - oldTime >= 0.25) {
                        motor0.setPower(0);
                        motor1.setPower(0);
                        motor2.setPower(0);
                        motor3.setPower(0);
                        servo0.setPower(0);
                        move17 = false;
                        move16 = true;

                    }
                } else {
                    if (((error0 <= 20 && error0 >= -20) && newTime - oldTime >= 0.25) || newTime - oldTime >= 0.335) {
                        motor0.setPower(0);
                        motor1.setPower(0);
                        motor2.setPower(0);
                        motor3.setPower(0);
                        servo0.setPower(0);
                        move17 = false;
                        move16 = true;

                    }
                }
            }
            else if(move16){
                if(move16First){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();

                    move16First = false;
                }



                motorPower0 = error0 * kp;
                motorPower1 = error1 * kp;
                motorPower2 = error2 * kp;
                motorPower3 = error3 * kp;

                position();


                newTime = runtime.seconds();
                setPointArm = 335;
                arm0.setPower((errorArm * kp - (kd * velocityArm)));
                arm1.setPower(-(errorArm * kp - (kd * velocityArm)));




                servo0.setPower(1);





                updateTime();

                velocityArm = deltaPositionArm / deltaTime;


                targetPositionLeft = closedLeft;



                targetPositionRight = closedRight;
                servoLeft.setPosition(targetPositionLeft);
                servoRight.setPosition(targetPositionRight);

                if(newTime - oldTime >= 1.25){
                    move16 = false;
                    servo0.setPower(0);
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }
            }

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

        previousPositionArm = positionArm;
        positionArm = arm0.getCurrentPosition() - startArm;
        deltaPositionArm = positionArm - previousPositionArm;
        errorArm = setPointArm - positionArm;

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

    public void setMinimumPower(double power){
        if((motorPower0 < power && motorPower0 > 0)){
            motorPower0 = power;
        }
        if((motorPower0 > -power && motorPower0 < 0)){
            motorPower0 = -power;
        }
        if((motorPower1 < power && motorPower1 > 0)){
            motorPower1 = power;
        }
        if((motorPower1 > -power && motorPower1 < 0)){
            motorPower1 = -power;
        }
    }
}


