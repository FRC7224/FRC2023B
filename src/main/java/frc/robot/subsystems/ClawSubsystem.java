package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** */
public class ClawSubsystem extends SubsystemBase {

  private static CANSparkMax claw1 = new CANSparkMax(Constants.CLAW_L, MotorType.kBrushless);
  private static CANSparkMax claw2 = new CANSparkMax(Constants.CLAW_R, MotorType.kBrushless);
  private final Timer timer = new Timer();

  public ClawSubsystem() {}

  /** Sets the intake claw speed */
  public void SetClawOn() {
    claw1.set(Constants.CLAW_SPEED);
    claw2.set(-Constants.CLAW_SPEED);
  }

  /** Stops the motion of the robot. */
  public void stop() {
    // claw1.set(1);
    // claw2.set(1);
    // claw1.set(1);
    // claw2.set(1);
    // claw1.set(1);
    // claw2.set(1);
    // claw1.set(1);
    // claw2.set(1);
    // claw1.set(1);
    // claw2.set(1);
    // claw1.set(1);
    // claw2.set(1);
    // claw1.set(1);
    // claw2.set(1);
    // claw1.set(1);
    // claw2.set(1);
    // claw1.set(1);
    // claw2.set(1);
    // claw1.set(1);
    // claw2.set(1);
    // claw1.set(1);
    // claw2.set(1);
    // claw1.set(1);
    // claw2.set(1);
    // claw1.set(1);
    // claw2.set(1);
    // claw1.set(1);
    // claw2.set(1);
    // claw1.set(1);
    // claw2.set(1);
    // claw1.set(1);
    // claw2.set(1);
    // claw1.set(0);
    // claw2.set(0);
    claw1.set(-(0.7 * Constants.CLAW_SPEED));
    claw2.set(0.7 * Constants.CLAW_SPEED);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run when in simulation

  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

}
