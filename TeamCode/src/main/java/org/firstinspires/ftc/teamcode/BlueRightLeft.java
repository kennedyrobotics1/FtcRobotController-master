package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.ConceptTensorFlowObjectDetectionEasy;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Locale;
@Autonomous(name="BlueRightLeft", group="Linear OpMode")
public class BlueRightLeft extends LinearOpMode {

    private DcMotor motor0 = null;
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;


    private ElapsedTime runtime = new ElapsedTime();

    double kp = 1.0 / 1350;
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
    double kpClaw = 1.0/100;
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

    private DcMotor climb0 = null;
    private DcMotor climb1 = null;
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

    //ConceptTensorFlowObjectDetectionEasy camera = new ConceptTensorFlowObjectDetectionEasy();


    double getPreviousPositionClimb;
    double deltaPositionClimb;
    double velocityClimb;

    double previousPositionClaw;
    double deltaPositionClaw;
    double velocityClaw;

    double servoPosition;

    double recognitions;

    @Override
    public void runOpMode() {
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");

        motor0.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        climb0 = hardwareMap.get(DcMotor.class, "climb0");
        climb1 = hardwareMap.get(DcMotor.class, "climb1");

        position0 = motor0.getCurrentPosition() - start0;
        position1 = motor1.getCurrentPosition() - start1;
        position2 = motor2.getCurrentPosition() - start2;
        position3 = motor3.getCurrentPosition() - start3;

        servo0 = hardwareMap.get(CRServo.class, "servo0");
        //servo0.setDirection(Servo.Direction.FORWARD);
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        //servo1.setDirection(Servo.Direction.REVERSE);
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        arm0  = hardwareMap.get(DcMotor.class, "arm0");
        arm1  = hardwareMap.get(DcMotor.class, "arm1");

        analogInput = hardwareMap.get(AnalogInput.class, "servoEncoder");

        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward
        /*camera.runOpMode();
        camera.initTfod();*/


        initTfod();

        waitForStart();

        imu.resetYaw();

        start = arm0.getCurrentPosition();
        position = arm0.getCurrentPosition() - start;

        startClimb = climb0.getCurrentPosition();
        positionClimb = climb0.getCurrentPosition() - startClimb;


        startTime = System.nanoTime();
        updateTime();

        servoPosition = analogInput.getVoltage() / 3.3 * 360;


        startClaw = servoPosition;
        positionClaw = servoPosition - startClaw;

        targetPosition0 = 0;
        servo0.setPower(targetPosition0);
        targetPosition1 = 0.1;
        servo1.setPower(-targetPosition1);
        servo2.setPower(targetPosition1);

        start0 = motor0.getCurrentPosition();
        start1 = motor1.getCurrentPosition();
        start2 = motor2.getCurrentPosition();
        start3 = motor3.getCurrentPosition();



        boolean move1 = true;
        boolean move1First = true;
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


        position();
        while (opModeIsActive()) {
            //camera.telemetryTfod();

            telemetryTfod();


            if (move1) {
                if(move1First){
                    start0 -= 200;
                    move1First = false;
                }
                setPoint0 = 1100;
                setPoint1 = 1100;
                setPoint2 = 1100;
                setPoint3 = 1100;
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
                    move1 = false;
                    move2 = true;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }

            }

            //turn left
            else if (move2) {
                if(move2First){
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    move2First = false;
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
                    move2 = false;
                    move3 = true;
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }
            }
            //arm down
            else if (move3 ){
                if(move3First){
                    oldTime = runtime.seconds();
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();


                    move3First = false;
                }
                setPoint0 = -150;
                setPoint1 = -150;
                setPoint2 = -150;
                setPoint3 = -150;

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
                setPoint = 450;
                previousPosition = position;
                position = arm0.getCurrentPosition() - start;
                deltaPosition = position - previousPosition;
                error = setPoint - position;
                arm0.setPower((error * kp - (kd * velocity)));
                arm1.setPower((error * kp - (kd * velocity)));


                setPointClaw = 100;
                previousPositionClaw = positionClaw;
                positionClaw = servoPosition - startClaw;
                deltaPositionClaw = positionClaw - previousPositionClaw;
                errorClaw = setPointClaw - positionClaw;



                updateTime();

                velocity = deltaPosition / deltaTime;


                targetPosition0 = 0;
                servo0.setPower(errorClaw * kpClaw);
                targetPosition1 = 0.2;
                servo1.setPower(-targetPosition1);
                servo2.setPower(targetPosition1);

                if(newTime - oldTime >= 1.5){
                    move3 = false;
                    move4 = true;
                }
            }
            //arm open
            else if (move4){
                if(move4First){
                    oldTime = runtime.seconds();

                    move4First = false;
                }

                newTime = runtime.seconds();
                setPoint = 450;
                previousPosition = position;
                position = arm0.getCurrentPosition() - start;
                deltaPosition = position - previousPosition;
                error = setPoint - position;
                arm0.setPower((error * kp - (kd * velocity)));
                arm1.setPower((error * kp - (kd * velocity)));


                setPointClaw = 100;
                previousPositionClaw = positionClaw;
                positionClaw = servoPosition - startClaw;
                deltaPositionClaw = positionClaw - previousPositionClaw;
                errorClaw = setPointClaw - positionClaw;



                updateTime();

                velocity = deltaPosition / deltaTime;


                targetPosition0 = 0;
                servo0.setPower(errorClaw * kpClaw);
                targetPosition1 = 0.2;
                servo1.setPower(targetPosition1);
                servo2.setPower(targetPosition1);

                if(newTime - oldTime >= 1.0){
                    move4 = false;
                    move5 = true;
                }
            }
            //arm up
            else if (move5){
                if(move5First){
                    oldTime = runtime.seconds();

                    move5First = false;
                }

                newTime = runtime.seconds();
                setPoint = 450;
                previousPosition = position;
                position = arm0.getCurrentPosition() - start;
                deltaPosition = position - previousPosition;
                error = setPoint - position;
                arm0.setPower((error * kp - (kd * velocity)));
                arm1.setPower((error * kp - (kd * velocity)));

                setPointClaw = 200;
                previousPositionClaw = positionClaw;
                positionClaw = servoPosition - startClaw;
                deltaPositionClaw = positionClaw - previousPositionClaw;
                errorClaw = setPointClaw - positionClaw;



                updateTime();

                velocity = deltaPosition / deltaTime;


                targetPosition0 = 0;
                servo0.setPower(errorClaw * kpClaw);
                targetPosition1 = 0.2;
                servo1.setPower(-targetPosition1);
                servo2.setPower(targetPosition1);

                if(newTime - oldTime >= 1.0){
                    move5 = false;
                    move6 = true;
                }
            }

            else if (move6) {
                if(move6First){
                    start0 = motor0.getCurrentPosition();
                    start1 = motor1.getCurrentPosition();
                    start2 = motor2.getCurrentPosition();
                    start3 = motor3.getCurrentPosition();
                    move6First = false;
                }
                setPoint0 = 1250;
                setPoint1 = -1250;
                setPoint2 = -1250;
                setPoint3 = 1250;
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
                    move6 = false;
                }
            }

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            /*telemetry.addData("error0 ", error0);
            telemetry.addData("error1 ", error1);
            telemetry.addData("position0 ", position0);
            telemetry.addData("position1 ", position1);
            telemetry.addData("start0 ", start0);*/
            telemetry.addData("Yaw: ", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Time ", runtime.seconds());
            telemetry.addData("Position0 ", position0);
            telemetry.addData("Position1 ", position1);
            telemetry.addData("Position2 ", position2);
            telemetry.addData("Position3 ", position3);
            telemetry.addData("servoPosition ", servoPosition);
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

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        headingError = setYaw - orientation.getYaw(AngleUnit.DEGREES);
    }
    public void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

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


