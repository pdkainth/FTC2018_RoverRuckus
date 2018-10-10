import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Wheels;

@TeleOp(name="Testing teleOP")

public class myTeleOp extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  //private DcMotor leftDrive = null;
  private Wheels mecanum = new Wheels();
  private Intake intake = new Intake();


  @Override
  public void init(){
    mecanum.init(hardwareMap);
    intake.init(hardwareMap);
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
    updateIntake();

    }

  @Override
  public void stop(){
    mecanum.stop();
    intake.stop();
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

  public void updateIntake(){

    boolean on = gamepad1.right_bumper;
    boolean off = gamepad1.left_bumper;

    if(on){
      intake.turnOn();
    } else if(off){
      intake.turnOff();
    }

  }
}
