package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Testing TFO teleOP")

public class Test_TFO_TeleOp extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private MineralDetect mineralDetect = new MineralDetect();
  private Wheels mecanum = new Wheels();
  private PhoneTilt phoneTilt = new PhoneTilt();

  @Override
  public void init(){
    mineralDetect.init(hardwareMap);
    mecanum.init(hardwareMap);
    phoneTilt.init(hardwareMap);
  }

  @Override
  public void init_loop(){}

  @Override
  public void start(){
    runtime.reset();
    mineralDetect.start(telemetry);
    phoneTilt.setPhoneTilt(PhoneTilt.PHONE_TILT_DEG_TENSOR_FLOW);
  }

  @Override
  public void loop(){

    telemetry.addData("Status", "Initialized");
    mineralDetect.detect(telemetry);
    updateWheels();
    phoneTilt.update(telemetry);
  }

  @Override
  public void stop(){
    mineralDetect.stop();
    mecanum.stop();
    phoneTilt.stop();
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
