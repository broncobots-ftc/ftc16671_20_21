package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive Via Gamepad", group = "ftc16671")
public class MecanumDriveOpMode extends OpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private double[] distances;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        mecanumDrive.init(hardwareMap);
        mecanumDrive.initServo(hardwareMap);
        mecanumDrive.initIntakeAndConveyor(hardwareMap);
        mecanumDrive.initShooterMotors(hardwareMap);
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        //setup gamepads
        double forward = gamepad1.left_stick_y * 1; //The y direction on the gamepad is reversed idk why
        double strafe = gamepad1.left_stick_x * -1;
        double rotate = gamepad1.right_stick_x * -1;
        double gamepadrt = gamepad1.right_trigger;
        boolean gamepadB = gamepad1.b;
        boolean gamepadX = gamepad1.x;

        boolean gamepadLeftBumper = gamepad1.left_bumper;
        boolean gamepadRightBumper = gamepad1.right_bumper;

        boolean gamepad2A = gamepad2.a;
        boolean gamepad2B = gamepad2.b;

        if(gamepadB){
            mecanumDrive.grabWobble();
        }

        if(gamepadX){
            mecanumDrive.releaseWobble();
        }

        if(gamepadLeftBumper){
            mecanumDrive.moveWobbleArmDown();
        }

        if(gamepadRightBumper){
            mecanumDrive.moveWobbleArmUp();
        }
        if(gamepad2A){
            mecanumDrive.moveLifterDown();
        }

        if(gamepad2B){
            mecanumDrive.moveLifterUp();
        }



        mecanumDrive.runIntake(-gamepadrt);
        //supply gamepad values to run motors, servo and other parts of robots
        mecanumDrive.driveMecanum(forward, strafe, rotate);
        distances = mecanumDrive.getDistanceCm();
        telemetry.addData("distance fwd", distances[0]);
        telemetry.addData("distance right", distances[1]);

    }


}