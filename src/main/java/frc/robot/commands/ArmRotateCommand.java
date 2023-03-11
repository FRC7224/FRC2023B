package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.ArmRotateSubsystem;
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
public class ArmRotateCommand extends CommandBase {

  private final ArmRotateSubsystem armrotatesubsystem;
  private final DoubleSupplier translationRSupplier;
  private final JoystickButton rotateoverideButton;
  private final JoystickButton medrotateButton;
  private final JoystickButton highrotateButton;
  private final JoystickButton drvrotateButton;

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
  public ArmRotateCommand(
      ArmRotateSubsystem armrotatesubsystem,
      JoystickButton rotateoverideButton,
      JoystickButton medrotateButton,
      JoystickButton highrotateButton,
      JoystickButton drvrotateButton,
      DoubleSupplier translationRSupplier) {
    this.armrotatesubsystem = armrotatesubsystem;
    this.rotateoverideButton = rotateoverideButton;
    this.medrotateButton = medrotateButton;
    this.highrotateButton = highrotateButton;
    this.drvrotateButton = drvrotateButton;
    this.translationRSupplier = translationRSupplier;

    addRequirements(armrotatesubsystem);
  }

  public void execute() {
    /* Gamepad processing */

    double rotatecontrol = modifyAxis(translationRSupplier.getAsDouble());

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
    if (rotateoverideButton.getAsBoolean()) {
      /* When button is held, just straight drive */
      /* Percent Output */
      armrotatesubsystem.SetPercentOutputR1(rotatecontrol * Constants.OV_ROT_ARM);
      armrotatesubsystem.SetPercentOutputR2(rotatecontrol * -Constants.OV_ROT_ARM);
    } else if (drvrotateButton.getAsBoolean()) {
      if (targetPositionRotations >= 0) { // check to see if arm is rotated backwards
        targetPositionRotations = Constants.DRV_ROT_PRESET;
      } else {
        targetPositionRotations = -Constants.DRV_ROT_PRESET;
      }
      armrotatesubsystem.SetTargetPositionRotationsR1(targetPositionRotations);
      armrotatesubsystem.SetTargetPositionRotationsR2(-targetPositionRotations);
    } else if (medrotateButton.getAsBoolean()) {
      if (targetPositionRotations >= 0) { // check to see if arm is rotated backwards
        targetPositionRotations = Constants.MED_ROT_PRESET;
      } else {
        targetPositionRotations = -Constants.MED_ROT_PRESET;
      }
      armrotatesubsystem.SetTargetPositionRotationsR1(targetPositionRotations);
      armrotatesubsystem.SetTargetPositionRotationsR2(-targetPositionRotations);
    } else if (highrotateButton.getAsBoolean()) {
      if (targetPositionRotations >= 0) { // check to see if arm is rotated backwards
        targetPositionRotations = Constants.HIGH_ROT_PRESET;
      } else {
        targetPositionRotations = -Constants.HIGH_ROT_PRESET;
      }
      armrotatesubsystem.SetTargetPositionRotationsR1(targetPositionRotations);
      armrotatesubsystem.SetTargetPositionRotationsR2(-targetPositionRotations);
    } else {
      /* Position Closed Loop */
      /* x *  Rotations * 4096 u/rev in either direction */

      targetPositionRotations = rotatecontrol * Constants.ROT_MAX;
      armrotatesubsystem.SetTargetPositionRotationsR1(targetPositionRotations);
      armrotatesubsystem.SetTargetPositionRotationsR2(-targetPositionRotations);
    }

    /* If Talon is in position closed-loop, print some more info */
    if (armrotatesubsystem.GetControlModeR1() == ControlMode.Position) {
      /* ppend more signals to print when in speed mode. */
      _sb.append("\terr R1:");
      _sb.append(armrotatesubsystem.GetClosedLoopErrorR1());
      _sb.append("u"); // Native Units

      _sb.append("\terr R2:");
      _sb.append(armrotatesubsystem.GetClosedLoopErrorR2());
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
    this.armrotatesubsystem.stop();

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
