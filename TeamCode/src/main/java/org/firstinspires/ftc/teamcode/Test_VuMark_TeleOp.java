package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Testing VuMark teleOP")

public class Test_VuMark_TeleOp extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private VuMark_Nav nav = new VuMark_Nav();
  private Wheels mecanum = new Wheels();

  @Override
  public void init(){
    nav.init(hardwareMap);
    mecanum.init(hardwareMap);
  }

  @Override
  public void init_loop(){}

  @Override
  public void start(){
    runtime.reset();
    nav.start();
  }

  @Override
  public void loop(){

    telemetry.addData("Status", "Initialized");
    nav.scan(telemetry);
    updateWheels();
  }

  @Override
  public void stop(){
    nav.stop();
  }

  public void updateWheels(){

    double drive = -gamepad1.left_stick_y;
    double strafe = 0.0;

    if(gamepad1.dpad_left == true){
      strafe = -0.6;
    } else if(gamepad1.dpad_right == true){
      strafe = 0.6;
    }

    double rotate = gamepad1.right_stick_x;

    mecanum.drive(drive, strafe, rotate, telemetry);
    telemetry.addData("Wheel","Wheel power: D: %.2f S: %.2f R: %.2f", drive, strafe, rotate);
  }
}
