import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.Wheels;

@TeleOp(name="Testing teleOP")

public class myTeleOp extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  //private DcMotor leftDrive = null;
  private Wheels mecanum = new Wheels();


  @Override
  public void init(){
    mecanum.init(hardwareMap);
  }

  @Override
  public void init_loop(){

  }



  @Override
  public void start(){
    runtime.reset();
  }



  @Override
  public void loop(){

    telemetry.addData("Status", "Initialized");
    updateWheels();

    }

  @Override
  public void stop(){
    mecanum.stop();
  }


  public void updateWheels(){


    double drive = -gamepad1.left_stick_y;
    double strafe = 0.0;

    if(gamepad1.dpad_left == true){
      strafe = -1.0;
    } else if(gamepad1.dpad_right == true){
      strafe = 1.0;
    }

    double rotate = gamepad1.right_stick_x;

    mecanum.drive(drive, strafe, rotate, telemetry);
    telemetry.addData("Wheel","Wheel power: D: %.2f S: %.2f R: %.2f", drive, strafe, rotate);
  }

}
