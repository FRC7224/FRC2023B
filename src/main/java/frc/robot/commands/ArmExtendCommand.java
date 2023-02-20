package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive based on the specified
 * values from the controller(s). This command is designed to be the default command for the
 * drivetrain subsystem.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: never
 *
 * <p>At End: stops the drivetrain
 */
public class ArmExtendCommand extends CommandBase {

  private final ArmSubsystem armsubsystem;
  private final DoubleSupplier translationXSupplier;
  private final JoystickButton extendoverideButton;
  private final JoystickButton medgoalButton;
  private final JoystickButton highgoalButton;

  double extendamount = 0;
  double targetPositionRotations = 0;
  /** Used to create string thoughout loop */
  StringBuilder _sb = new StringBuilder();

  int _loops = 0;

  /**
   * Create a new ArmExtendCommand command object.
   *
   * @param armsubsystem the drivetrain subsystem instructed by this command
   * @param translationXSupplier the supplier of the translation x value as a percentage of the
   *     maximum velocity as defined by the standard field or robot coordinate system
   * @return
   */
  public ArmExtendCommand(
      ArmSubsystem armsubsystem,
      JoystickButton extendoverideButton,
      JoystickButton medgoalButton,
      JoystickButton highgoalButton,
      DoubleSupplier translationXSupplier) {
    this.armsubsystem = armsubsystem;
    this.extendoverideButton = extendoverideButton;
    this.medgoalButton = medgoalButton;
    this.highgoalButton = highgoalButton;
    this.translationXSupplier = translationXSupplier;

    addRequirements(armsubsystem);
  }

  public void execute() {
    /* Gamepad processing */

    double extendcontrol = modifyAxis(translationXSupplier.getAsDouble());
    SmartDashboard.putNumber("extendcontrol", extendcontrol);

    double motorOutput;

    /* Get Talon/Victor's current output percentage */
    motorOutput = ArmSubsystem.GetMotorOutputPercent();

    /* Prepare line to print */
    _sb.append("\tout:");
    /* Cast to int to remove decimal places */
    _sb.append((int) (motorOutput * 100));
    _sb.append("%"); // Percent

    _sb.append("\tpos:");
    _sb.append(ArmSubsystem.GetSelectedSensorPosition());
    _sb.append("u"); // Native units

    /**
     * When button 1 is pressed, perform Position Closed Loop to selected position, indicated by
     * Joystick position x10, [-10, 10] rotations
     */
    if (extendoverideButton.getAsBoolean()) {
      /* When button is held, just straight drive */
      /* Percent Output */
      armsubsystem.SetPercentOutput(extendcontrol * 0.1);
    } else if (medgoalButton.getAsBoolean()) {
      targetPositionRotations = Constants.MED_ARM_PRESET;
      armsubsystem.SetTargetPositionRotations(targetPositionRotations);
    } else if (highgoalButton.getAsBoolean()) {
      targetPositionRotations = Constants.HIGH_ARM_PRESET;
      armsubsystem.SetTargetPositionRotations(targetPositionRotations);
    } else {
      /* Position Closed Loop */
      /* 7.5 Rotations * 4096 u/rev in either direction */
      extendamount = extendcontrol;
      if (extendcontrol <= 0.0) {
        extendamount = 0;
      }
      targetPositionRotations = extendamount * 7.5 * 4096;
      armsubsystem.SetTargetPositionRotations(targetPositionRotations);
    }

    /* If Talon is in position closed-loop, print some more info */
    if (armsubsystem.GetControlMode() == ControlMode.Position) {
      /* ppend more signals to print when in speed mode. */
      _sb.append("\terr:");
      _sb.append(armsubsystem.GetClosedLoopError());
      _sb.append("u"); // Native Units

      _sb.append("\ttrg:");
      _sb.append(targetPositionRotations);
      _sb.append("u"); // / Native Units
    }

    /** Print every ten loops, printing too much too fast is generally bad for performance. */
    if (++_loops >= 10) {
      _loops = 0;
      System.out.println(_sb.toString());
    }

    /* Reset built string for next loop */
    _sb.setLength(0);
  }

  @Override
  public void end(boolean interrupted) {
    this.armsubsystem.stop();

    super.end(interrupted);

    Logger.getInstance().recordOutput("ActiveCommands/TeleopSwerve", false);
  }

  /**
   * Squares the specified value, while preserving the sign. This method is used on all joystick
   * inputs. This is useful as a non-linear range is more natural for the driver.
   *
   * @param value
   * @return
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, Constants.DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}
