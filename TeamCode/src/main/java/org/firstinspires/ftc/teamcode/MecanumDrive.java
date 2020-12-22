package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;

class MecanumDrive {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;

    DcMotor shooterFront;
    DcMotor shooterBack;
    DcMotor intake;
    DcMotor conveyor;

    Servo grabber;
    Servo wobbleArm;
    Servo lifter;
    Servo pusher;

    public static double GEAR_RATIO = 1.0; // for simulator - ours should be 0.5f;
    public static double WHEEL_RADIUS = 5.0;  // 5 cm
    public static double TICKS_PER_ROTATION = 1120.0;  // From NeveRest (for simulator)  GoBilda should be 383.6f
    public static double CM_PER_TICK = (2 * Math.PI * GEAR_RATIO * WHEEL_RADIUS) / TICKS_PER_ROTATION;

    private double maxSpeed = 1.0;


    private MatrixF conversion;
    private GeneralMatrixF encoderMatrix = new GeneralMatrixF(3, 1);

    private int frontLeftOffset;
    private int frontRightOffset;
    private int backRightOffset;
    private int backLeftOffset;

    private ElapsedTime runtime = new ElapsedTime();


    MecanumDrive() {
        float[] data = {1.0f, 1.0f, 1.0f,
                1.0f, -1.0f, -1.0f,
                1.0f, -1.0f, 1.0f};
        conversion = new GeneralMatrixF(3, 3, data);
        conversion = conversion.inverted();
    }

    void init(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class, "front_left_motor");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight = hwMap.get(DcMotor.class, "front_right_motor");
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft = hwMap.get(DcMotor.class, "back_left_motor");
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight = hwMap.get(DcMotor.class, "back_right_motor");
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //grabber = hwMap.get(Servo.class, "left_hand");
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
    }

    void initServo(HardwareMap hwMap){
        grabber = hwMap.get(Servo.class, "grabber");//0 - control hub
        wobbleArm = hwMap.get(Servo.class, "wobble_arm");//4 - control hub
        lifter = hwMap.get(Servo.class, "lifter");//5 - control hub
        pusher = hwMap.get(Servo.class, "pusher");//3 - ext hub

    }
    void initShooterMotors(HardwareMap hwMap){
        shooterFront = hwMap.get(DcMotor.class, "shooter_front");//
        shooterBack = hwMap.get(DcMotor.class, "shooter_back");
    }

    void initIntakeAndConveyor(HardwareMap hwMap){
        intake = hwMap.get(DcMotor.class, "intake");
        conveyor = hwMap.get(DcMotor.class, "conveyor");
    }


    void setAllMotorsToRunToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void setAllMotorsToRunUsingEncoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void setSpeeds(double flSpeed, double frSpeed, double blSpeed, double brSpeed) {
        double largest = maxSpeed;
        largest = Math.max(largest, Math.abs(flSpeed));
        largest = Math.max(largest, Math.abs(frSpeed));
        largest = Math.max(largest, Math.abs(blSpeed));
        largest = Math.max(largest, Math.abs(brSpeed));

        frontLeft.setPower(flSpeed / largest);
        frontRight.setPower(frSpeed / largest);
        backLeft.setPower(blSpeed / largest);
        backRight.setPower(brSpeed / largest);
    }

    void driveMecanum(double forward, double strafe, double rotate) {
        //watch following site to get more details on different options on how to move motors
        //https://stemrobotics.cs.pdx.edu/node/4746
        double frontLeftSpeed = forward + strafe + rotate;
        double frontRightSpeed = forward - strafe - rotate;
        double backLeftSpeed = forward - strafe + rotate;
        double backRightSpeed = forward + strafe - rotate;

        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    double[] getDistanceCm() {
        double[] distances = {0.0, 0.0};

        encoderMatrix.put(0, 0, (float) ((frontLeft.getCurrentPosition() - frontLeftOffset) * CM_PER_TICK));
        encoderMatrix.put(1, 0, (float) ((frontRight.getCurrentPosition() - frontRightOffset) * CM_PER_TICK));
        encoderMatrix.put(2, 0, (float) ((backLeft.getCurrentPosition() - backLeftOffset) * CM_PER_TICK));

        MatrixF distanceMatrix = conversion.multiplied(encoderMatrix);
        distances[0] = distanceMatrix.get(0, 0);
        distances[1] = distanceMatrix.get(1, 0);

        return distances;
    }

    void setMaxSpeed(double speed) {
        maxSpeed = Math.min(speed, 1.0);
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setEncoderOffsets() {
        frontRightOffset = frontRight.getCurrentPosition();
        frontLeftOffset = frontLeft.getCurrentPosition();
        backLeftOffset = backLeft.getCurrentPosition();
        backRightOffset = backRight.getCurrentPosition();
    }

    private void setAllWheelsToTargetPosition(int distance) {
        frontLeft.setTargetPosition(distance);
        frontRight.setTargetPosition(distance);
        backLeft.setTargetPosition(distance);
        backRight.setTargetPosition(distance);
    }

    public void moveForward(int distanceInInches, boolean isOpModeActive, int timeoutS){
        setAllWheelsToTargetPosition(distanceInInches);
        setAllMotorsToRunToPosition();
        runtime.reset();
        driveMecanum(1.0,0.0,0.0);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() || frontRight.isBusy())) {
            //wait or print something in telemetry
        }
        setSpeeds(0.0, 0.0,0.0, 0.0);
        setAllMotorsToRunUsingEncoder();

    }

    public void moveBackward(int distanceInInches, boolean isOpModeActive, int timeoutS){
        setAllWheelsToTargetPosition(-distanceInInches);
        setAllMotorsToRunToPosition();
        runtime.reset();
        driveMecanum(1.0,0.0,0.0);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() || frontRight.isBusy())) {
            //wait or print something in telemetry
        }
        setSpeeds(0.0, 0.0,0.0, 0.0);
        setAllMotorsToRunUsingEncoder();
    }

    public void strafeLeft(int distanceInInches, boolean isOpModeActive, int timeoutS){
        setAllWheelsToTargetPosition(distanceInInches);
        setAllMotorsToRunToPosition();
        runtime.reset();
        driveMecanum(0.0,1.0,0.0);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() || frontRight.isBusy())) {
            //wait or print something in telemetry
        }
        setSpeeds(0.0, 0.0,0.0, 0.0);
        setAllMotorsToRunUsingEncoder();

    }

    public void strafeRight(int distanceInInches, boolean isOpModeActive, int timeoutS){
        setAllWheelsToTargetPosition(distanceInInches);
        setAllMotorsToRunToPosition();
        runtime.reset();
        driveMecanum(0.0,-1.0,0.0);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() || frontRight.isBusy())) {
            //wait or print something in telemetry
        }
        setSpeeds(0.0, 0.0,0.0, 0.0);
        setAllMotorsToRunUsingEncoder();

    }

    public void rotateLeft(int distanceInInches, boolean isOpModeActive, int timeoutS){
        setAllWheelsToTargetPosition(distanceInInches);
        setAllMotorsToRunToPosition();
        runtime.reset();
        driveMecanum(0.0,0.0,1.0);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() || frontRight.isBusy())) {
            //wait or print something in telemetry
        }
        setSpeeds(0.0, 0.0,0.0, 0.0);
        setAllMotorsToRunUsingEncoder();

    }

    public void rotateRight(int distanceInInches, boolean isOpModeActive, int timeoutS){
        setAllWheelsToTargetPosition(distanceInInches);
        setAllMotorsToRunToPosition();
        runtime.reset();
        driveMecanum(1.0,0.0,-1.0);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() || frontRight.isBusy())) {
            //wait or print something in telemetry
        }
        setSpeeds(0.0, 0.0,0.0, 0.0);
        setAllMotorsToRunUsingEncoder();

    }


    public void moveForward(double forward){
        driveMecanum(forward, 0.0, 0.0);
    }

    public void moveBackward(double backward){
        driveMecanum(-backward, 0.0, 0.0);
    }

    public void strafeLeft(double left){
        driveMecanum(0.0, left, 0.0);
    }

    public void strafeRight(double right){
        driveMecanum(0.0, -right, 0.0);
    }

    public void rotateLeft(double left){
        driveMecanum(0.0, 0.0, left);
    }

    public void rotateRight(double right){
        driveMecanum(0.0, 0.0, -right);
    }

    public void moveBasedOnTotalRings(int totalRings) {
        if(totalRings == 0){
            //Strafe right
            strafeRight(12, true, 5);
            //Move forward to A
            moveForward(12, true, 5);

        }else if(totalRings == 1){
            //Strafe right
            strafeRight(10, true, 5);
            //Move forward to A
            moveForward(18, true, 5);
            //Strafe left to B
            strafeLeft(10, true, 5);

        }else if(totalRings == 4){
            //Strafe right
            strafeRight(12, true, 5);
            //Move forward to A
            moveForward(24, true, 5);

        }
    }

    public void grabWobble(){
        grabber.setPosition(0);
    }

    public void releaseWobble(){
        grabber.setPosition(1.0);
    }

    public void runIntake(double intakePower){
        intake.setPower(intakePower);
    }

    public void runConveyor(double conveyorPower){
        conveyor.setPower(conveyorPower);
    }

    public void moveWobbleArmUp() {
        wobbleArm.setPosition(0);
    }
    public void moveWobbleArmDown() {
        wobbleArm.setPosition(1.0);
    }
    public void moveLifterUp() {
        lifter.setPosition(0);
    }
    public void moveLifterDown() {
        lifter.setPosition(1.0);
    }
    public void pushForward() {
        pusher.setPosition(0);
    }
    public void pushBackward() {
        pusher.setPosition(1.0);
    }

}