package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Drive Via Gamepad", group = "ftc16671")
public class MecanumDriveOpMode extends OpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private double[] distances;
    double pusherPosition, lifterPosition, grabberPosition, armPosition;
    double  MIN_POSITION = 0, MAX_POSITION = 1;
    double SERVO_OFFSET = 0.005;
    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        mecanumDrive.init(hardwareMap);
        mecanumDrive.initServo(hardwareMap);
        mecanumDrive.initIntakeAndConveyor(hardwareMap);
        mecanumDrive.initShooterMotors(hardwareMap);
        mecanumDrive.lifter.setPosition(1.0);
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        //setup gamepads
        //getting gamepad1 controls

        double forward = gamepad1.left_stick_y * 1; //The y direction on the gamepad is reversed idk why
        double strafe = gamepad1.left_stick_x * -1;
        double rotate = gamepad1.right_stick_x * -1;
        /*
        double gamepadrt = gamepad1.right_trigger;
        double gamepadlt = gamepad1.left_trigger;
        boolean gamepadLeftBumper = gamepad1.left_bumper;
        boolean gamepadRightBumper = gamepad1.right_bumper;
        //getting gamepad 2 controls
        boolean gamepad2X = gamepad2.x;
        boolean gamepad2Y = gamepad2.y;
        boolean gamepad2A = gamepad2.a;
        boolean gamepad2B = gamepad2.b;
        double gamepad2rt = gamepad2.right_trigger;
        double gamepad2lt = gamepad2.left_trigger;
        boolean gamepad2LeftBumper = gamepad2.left_bumper;
        boolean gamepad2RightBumper = gamepad2.right_bumper;

         */
        //when motor starts moving, lifter position should be at bottom level
        if(mecanumDrive.frontLeft.isBusy() && mecanumDrive.backRight.isBusy()){
            mecanumDrive.lifter.setPosition(1.0);
        }

        if (gamepad1.right_trigger > 0){
            mecanumDrive.runIntake(-1.0);//take it in
            mecanumDrive.runConveyor(-1.0);//take it in
        }

        if (gamepad1.left_trigger > 0){
            mecanumDrive.runIntake(1.0);//take it out
            mecanumDrive.runConveyor(1.0);//take it out
        }

        if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0){
            mecanumDrive.runIntake(0.0);//take it out
            mecanumDrive.runConveyor(0.0);//take it out
        }


        // if gamepad 2 right trigger or left trigger are pushed even slightly,
        // both shooters will run full speed
        if(gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0){
            mecanumDrive.runShooterFront(1.0);
            mecanumDrive.runShooterBack(1.0);
        }

        if(gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0) {
            mecanumDrive.runShooterFront(0.0);
            mecanumDrive.runShooterBack(0.0);
        }

        //use gamepad2 left bumper to grab the wobber and move the arm up as well
        if(gamepad2.left_bumper || gamepad2.dpad_left){
            grabberPosition = 0.4;

            mecanumDrive.grabber.setPosition(Range.clip(grabberPosition, MIN_POSITION, MAX_POSITION));
            //mecanumDrive.wobbleArm.setPosition(Range.clip(armPosition, MIN_POSITION, MAX_POSITION));
            telemetry.addData("grabber servo", "position=" + grabberPosition + "  actual="
                    + mecanumDrive.grabber.getPosition());
            //telemetry.addData("arm servo", "position=" + armPosition + "  actual="
            //        + mecanumDrive.wobbleArm.getPosition());
        }
        if(gamepad2.right_bumper || gamepad2.dpad_right){
            grabberPosition = MIN_POSITION;

            mecanumDrive.grabber.setPosition(Range.clip(grabberPosition, MIN_POSITION, MAX_POSITION));
            //mecanumDrive.wobbleArm.setPosition(Range.clip(armPosition, MIN_POSITION, MAX_POSITION));
            telemetry.addData("grabber servo", "position=" + grabberPosition + "  actual="
                    + mecanumDrive.grabber.getPosition());
            //telemetry.addData("arm servo", "position=" + armPosition + "  actual="
            //      + mecanumDrive.wobbleArm.getPosition());
        }

        if(gamepad2.dpad_up){
            armPosition = 0.4;
            mecanumDrive.wobbleArm.setPosition(Range.clip(armPosition, MIN_POSITION, MAX_POSITION));
            telemetry.addData("arm servo", "position=" + armPosition + "  actual="
                    + mecanumDrive.wobbleArm.getPosition());
        }
        if(gamepad2.dpad_down){
            armPosition = 0.0;
            mecanumDrive.wobbleArm.setPosition(Range.clip(armPosition, MIN_POSITION, MAX_POSITION));
            telemetry.addData("arm servo", "position=" + armPosition + "  actual="
                    + mecanumDrive.wobbleArm.getPosition());
        }

        /*
        //use gamepad2 right bumper to move the arm down and release the wobble.
        if(gamepad2.right_bumper){
            if(armPosition < MAX_POSITION){
                armPosition = MAX_POSITION;
            }
            if(armPosition == MAX_POSITION){
                armPosition = MIN_POSITION;
            }
        }
        mecanumDrive.grabber.setPosition(Range.clip(grabberPosition, MIN_POSITION, MAX_POSITION));
        telemetry.addData("wobber_arm servo", "position=" + grabberPosition + "  actual="
                + mecanumDrive.grabber.getPosition());

        */
        //Setting gamepad2B***************************

        if (gamepad2.b) {
            if (pusherPosition > MIN_POSITION) {
                pusherPosition = MIN_POSITION;
            }
            mecanumDrive.pusher.setPosition(Range.clip(pusherPosition, MIN_POSITION, MAX_POSITION));
            //wait
            MecanumDrive.sleep(500);
            //
            if(mecanumDrive.pusher.getPosition() == MIN_POSITION ){
                pusherPosition =  MAX_POSITION;
            }

            mecanumDrive.pusher.setPosition(Range.clip(pusherPosition, MIN_POSITION, MAX_POSITION));
            telemetry.addData("pusher servo", "position=" + pusherPosition + "  actual="
                    + mecanumDrive.pusher.getPosition());
        }





        //Setting gamepad2Y***************************
        if(gamepad2.y){
            if(lifterPosition > 0.2) {
                lifterPosition -= SERVO_OFFSET;
            }
            mecanumDrive.lifter.setPosition(Range.clip(lifterPosition, MIN_POSITION, MAX_POSITION));
            telemetry.addData("lifter servo", "position=" + lifterPosition + "  actual="
                    + mecanumDrive.lifter.getPosition());
        }



        //Setting gamepad2A***************************
        if(gamepad2.a){
            if(lifterPosition < 0.8){
                lifterPosition += SERVO_OFFSET;
            }
            mecanumDrive.lifter.setPosition(Range.clip(lifterPosition, MIN_POSITION, MAX_POSITION));
            telemetry.addData("lifter servo", "position=" + lifterPosition + "  actual="
                    + mecanumDrive.lifter.getPosition());

        }


        //supply gamepad values to run motors, servo and other parts of robots
        mecanumDrive.driveMecanum(forward, strafe, rotate);
        distances = mecanumDrive.getDistanceCm();
        telemetry.addData("distance fwd", distances[0]);
        telemetry.addData("distance right", distances[1]);
        telemetry.update();



    }

    /**
     *
     * @param miliseconds
     */
    private void justWait(int miliseconds){

        double currTime = getRuntime();
        double waitUntil = currTime + (double)(miliseconds/1000);
        while (getRuntime() < waitUntil){
            //do nothing
        }

    }


}