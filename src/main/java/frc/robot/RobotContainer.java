// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOAhrs;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIO;
import frc.lib.team3061.swerve.SwerveModuleIOSim;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFX;
import frc.lib.team3061.vision.Vision;
import frc.lib.team3061.vision.VisionConstants;
import frc.lib.team3061.vision.VisionIO;
import frc.lib.team3061.vision.VisionIOPhotonVision;
import frc.lib.team3061.vision.VisionIOSim;
import frc.robot.Constants.Mode;
import frc.robot.commands.ArmExtendCommand;
import frc.robot.commands.ArmRotateCommand;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.FollowPath;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ArmRotateSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // private OperatorInterface oi = new OperatorInterface() {};

  final Joystick drivejoystick = new Joystick(0);
  JoystickButton ClawOn = new JoystickButton(drivejoystick, 1),
      XStanceButton = new JoystickButton(drivejoystick, 2),
      ResetGyroButton = new JoystickButton(drivejoystick, 3),
      FieldRelativeButton = new JoystickButton(drivejoystick, 4),
      Highgoal = new JoystickButton(drivejoystick, 5),
      Medgoal = new JoystickButton(drivejoystick, 6),
      Drvgoal = new JoystickButton(drivejoystick, 7),
      ExtendOveride = new JoystickButton(drivejoystick, 8),
      button9 = new JoystickButton(drivejoystick, 9),
      RotateoverideButton = new JoystickButton(drivejoystick, 10);
  // button11 = new JoystickButton(drivejoystick, 11);
  // button12 = new JoystickButton(drivejoystick, 12);

  private Drivetrain drivetrain;
  private ArmSubsystem armcontrol;
  private ArmRotateSubsystem armrotatecontrol;
  private ClawSubsystem clawsubsystem;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to
  // ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();

  /** Create the container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // create real, simulated, or replay subsystems based on the mode and robot
    // specified
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2023_SEASON:
          {
            // GyroIO gyro = new Nax X2
            // The important thing about how you configure your gyroscope is that rotating
            // the robot counter-clockwise should
            // cause the angle reading to increase until it wraps back over to zero.
            GyroIO gyro =
                new GyroIOAhrs(); // NavX connected over MXP    SmartDashboard.putNumber("pitch io",
            // gyro.getPitch());

            SwerveModule flModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        0,
                        FRONT_LEFT_MODULE_DRIVE_MOTOR,
                        FRONT_LEFT_MODULE_STEER_MOTOR,
                        FRONT_LEFT_MODULE_STEER_ENCODER,
                        FRONT_LEFT_MODULE_STEER_OFFSET),
                    0,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule frModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        1,
                        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_ENCODER,
                        FRONT_RIGHT_MODULE_STEER_OFFSET),
                    1,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule blModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        2,
                        BACK_LEFT_MODULE_DRIVE_MOTOR,
                        BACK_LEFT_MODULE_STEER_MOTOR,
                        BACK_LEFT_MODULE_STEER_ENCODER,
                        BACK_LEFT_MODULE_STEER_OFFSET),
                    2,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule brModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        3,
                        BACK_RIGHT_MODULE_DRIVE_MOTOR,
                        BACK_RIGHT_MODULE_STEER_MOTOR,
                        BACK_RIGHT_MODULE_STEER_ENCODER,
                        BACK_RIGHT_MODULE_STEER_OFFSET),
                    3,
                    MAX_VELOCITY_METERS_PER_SECOND);

            drivetrain = new Drivetrain(gyro, flModule, frModule, blModule, brModule);
            armcontrol = new ArmSubsystem();
            armrotatecontrol = new ArmRotateSubsystem();
            clawsubsystem = new ClawSubsystem();
            new Vision(new VisionIOPhotonVision(CAMERA_NAME));
            break;
          }
        case ROBOT_SIMBOT:
          {
            SwerveModule flModule =
                new SwerveModule(new SwerveModuleIOSim(), 0, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule frModule =
                new SwerveModule(new SwerveModuleIOSim(), 1, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule blModule =
                new SwerveModule(new SwerveModuleIOSim(), 2, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule brModule =
                new SwerveModule(new SwerveModuleIOSim(), 3, MAX_VELOCITY_METERS_PER_SECOND);
            drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
            armcontrol = new ArmSubsystem();
            armrotatecontrol = new ArmRotateSubsystem();
            clawsubsystem = new ClawSubsystem();
            AprilTagFieldLayout layout;
            try {
              layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
            } catch (IOException e) {
              layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
            }
            new Vision(
                new VisionIOSim(layout, drivetrain::getPose, VisionConstants.ROBOT_TO_CAMERA));

            break;
          }
        default:
          break;
      }

    } else {
      SwerveModule flModule =
          new SwerveModule(new SwerveModuleIO() {}, 0, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule frModule =
          new SwerveModule(new SwerveModuleIO() {}, 1, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule blModule =
          new SwerveModule(new SwerveModuleIO() {}, 2, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule brModule =
          new SwerveModule(new SwerveModuleIO() {}, 3, MAX_VELOCITY_METERS_PER_SECOND);
      drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
      armcontrol = new ArmSubsystem();
      armrotatecontrol = new ArmRotateSubsystem();
      clawsubsystem = new ClawSubsystem();
      new Vision(new VisionIO() {});
    }

    // disable all telemetry in the LiveWindow to reduce the processing during each
    // iteration
    LiveWindow.disableAllTelemetry();

    updateOI();

    configureAutoCommands();
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {

    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    /*
     * Set up the default command for the drivetrain. The joysticks' values map to
     * percentage of the
     * maximum velocities. The velocities may be specified from either the robot's
     * frame of
     * reference or the field's frame of reference. In the robot's frame of
     * reference, the positive
     * x direction is forward; the positive y direction, left; position rotation,
     * CCW. In the field
     * frame of reference, the origin of the field to the lower left corner (i.e.,
     * the corner of the
     * field to the driver's right). Zero degrees is away from the driver and
     * increases in the CCW
     * direction. This is why the left joystick's y axis specifies the velocity in
     * the x direction
     * and the left joystick's x axis specifies the velocity in the y direction.
     */
    // drivetrain.setDefaultCommand(
    // new TeleopSwerve(drivetrain,
    // -drivejoystick.getRawAxis(1),-drivejoystick.getRawAxis(0),
    // -drivejoystick.getRawAxis(3));
    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain,
            () -> -drivejoystick.getRawAxis(1),
            () -> -drivejoystick.getRawAxis(0),
            () -> -drivejoystick.getRawAxis(3))); // field vs robot drive

    armcontrol.setDefaultCommand(
        new ArmExtendCommand( // use same button for preset rotate and extend
            armcontrol,
            ExtendOveride,
            Medgoal,
            Highgoal,
            Drvgoal,
            () -> drivejoystick.getRawAxis(4)));

    armrotatecontrol.setDefaultCommand(
        new ArmRotateCommand(
            armrotatecontrol,
            RotateoverideButton,
            Medgoal,
            Highgoal,
            Drvgoal,
            () -> -drivejoystick.getRawAxis(2)));

    configureButtonBindings();
  }

  /**
   * Factory method to create the singleton robot container object.
   *
   * @return the singleton robot container object
   */
  public static RobotContainer getInstance() {
    return robotContainer;
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // field-relative toggle

    //   SmartDashboard.putBoolean("Field Button input", FieldRelativeButton.getAsBoolean());
    FieldRelativeButton.toggleOnTrue(
        Commands.either(
            Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
            Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
            drivetrain::getFieldRelative));

    // reset close ope claw
    ClawOn.onFalse(Commands.runOnce(clawsubsystem::SetClawOn, clawsubsystem));
    ClawOn.onTrue(Commands.runOnce(clawsubsystem::stop, clawsubsystem));

    // x-stance
    XStanceButton.onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    XStanceButton.onTrue(Commands.runOnce(drivetrain::setBrakeOn, drivetrain));

    XStanceButton.onTrue(Commands.runOnce(drivetrain::disableXstance, drivetrain));
    XStanceButton.onTrue(Commands.runOnce(drivetrain::setBrakeOff, drivetrain));
    // arm extend
    // Extendarm.onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    // Extendarm.onTrue(Commands.runOnce(armcontrol::setExtendArm, armcontrol));
    ResetGyroButton.onTrue(Commands.runOnce(drivetrain::autoGyroscope, drivetrain));
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    AUTO_EVENT_MAP.put("event1", Commands.print("passed marker 1"));
    AUTO_EVENT_MAP.put("event2", Commands.print("passed marker 2"));

    // build auto path commands

    List<PathPlannerTrajectory> Left1 =
        PathPlanner.loadPathGroup(
            "Left1",
            new PathConstraints(
                AUTO_MAX_SPEED_METERS_PER_SECOND, AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

    List<PathPlannerTrajectory> Left2 =
        PathPlanner.loadPathGroup(
            "Left2",
            new PathConstraints(
                AUTO_MAX_SPEED_METERS_PER_SECOND, AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

    List<PathPlannerTrajectory> Left3 =
        PathPlanner.loadPathGroup(
            "Left3",
            new PathConstraints(
                AUTO_MAX_SPEED_METERS_PER_SECOND, AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

    List<PathPlannerTrajectory> Left3long =
        PathPlanner.loadPathGroup(
            "Left3long",
            new PathConstraints(
                AUTO_MAX_SPEED_METERS_PER_SECOND, AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

    List<PathPlannerTrajectory> Right1 =
        PathPlanner.loadPathGroup(
            "Right1",
            new PathConstraints(
                AUTO_MAX_SPEED_METERS_PER_SECOND, AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

    List<PathPlannerTrajectory> Right2 =
        PathPlanner.loadPathGroup(
            "Right2",
            new PathConstraints(
                AUTO_MAX_SPEED_METERS_PER_SECOND, AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

    List<PathPlannerTrajectory> Right3 =
        PathPlanner.loadPathGroup(
            "Right3",
            new PathConstraints(
                AUTO_MAX_SPEED_METERS_PER_SECOND, AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

    List<PathPlannerTrajectory> Right3long =
        PathPlanner.loadPathGroup(
            "Right3long",
            new PathConstraints(
                AUTO_MAX_SPEED_METERS_PER_SECOND, AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

    Command autoLeft1 =
        Commands.sequence(
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.runOnce(armrotatecontrol::SetTargetRotStart, armrotatecontrol),
            Commands.runOnce(clawsubsystem::SetClawOn, clawsubsystem),
            Commands.waitSeconds(0.5),
            Commands.runOnce(armcontrol::SetTargetArmHigh, armcontrol),
            Commands.runOnce(armrotatecontrol::SetTargetRotHIGH, armrotatecontrol),
            Commands.waitSeconds(2.0),
            Commands.runOnce(clawsubsystem::stop, clawsubsystem),
            Commands.waitSeconds(0.5),
            Commands.runOnce(armcontrol::SetTargetArmZero, armcontrol),
            Commands.runOnce(armrotatecontrol::SetTargetRotZero, armrotatecontrol),
            new FollowPathWithEvents(
                new FollowPath(Left1.get(0), drivetrain, true),
                Left1.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            Commands.runOnce(clawsubsystem::SetClawOn, clawsubsystem),
            Commands.waitSeconds(1.0),
            Commands.runOnce(drivetrain::autoBalance, drivetrain));
    //      Commands.runOnce(drivetrain::enableXstance, drivetrain),
    //      Commands.runOnce(drivetrain::setBrakeOn, drivetrain),
    //      Commands.runOnce(drivetrain::autoGyroscope, drivetrain),
    //      Commands.waitSeconds(2.0),
    //      Commands.runOnce(drivetrain::disableXstance, drivetrain),
    //      Commands.runOnce(drivetrain::setBrakeOff, drivetrain));

    Command autoLeft2 =
        Commands.sequence(
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.runOnce(armrotatecontrol::SetTargetRotStart, armrotatecontrol),
            Commands.runOnce(clawsubsystem::SetClawOn, clawsubsystem),
            Commands.waitSeconds(0.5),
            Commands.runOnce(armcontrol::SetTargetArmHigh, armcontrol),
            Commands.runOnce(armrotatecontrol::SetTargetRotHIGH, armrotatecontrol),
            Commands.waitSeconds(2.0),
            Commands.runOnce(clawsubsystem::stop, clawsubsystem),
            Commands.waitSeconds(0.5),
            Commands.runOnce(armcontrol::SetTargetArmZero, armcontrol),
            Commands.runOnce(armrotatecontrol::SetTargetRotZero, armrotatecontrol),
            new FollowPathWithEvents(
                new FollowPath(Left2.get(0), drivetrain, true),
                Left2.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            Commands.runOnce(clawsubsystem::SetClawOn, clawsubsystem),
            Commands.waitSeconds(1.0),
            Commands.runOnce(drivetrain::autoBalance, drivetrain));
    //       Commands.runOnce(drivetrain::autoGyroscope, drivetrain),
    //       Commands.runOnce(drivetrain::enableXstance, drivetrain),
    //       Commands.runOnce(drivetrain::setBrakeOn, drivetrain),
    //       Commands.waitSeconds(2.0),
    //       Commands.runOnce(drivetrain::disableXstance, drivetrain),
    //       Commands.runOnce(drivetrain::setBrakeOff, drivetrain));

    Command autoLeft3 =
        Commands.sequence(
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.runOnce(armrotatecontrol::SetTargetRotStart, armrotatecontrol),
            Commands.runOnce(clawsubsystem::SetClawOn, clawsubsystem),
            Commands.waitSeconds(0.5),
            Commands.runOnce(armcontrol::SetTargetArmHigh, armcontrol),
            Commands.runOnce(armrotatecontrol::SetTargetRotHIGH, armrotatecontrol),
            Commands.waitSeconds(2.0),
            Commands.runOnce(clawsubsystem::stop, clawsubsystem),
            Commands.waitSeconds(0.5),
            Commands.runOnce(armcontrol::SetTargetArmZero, armcontrol),
            Commands.runOnce(armrotatecontrol::SetTargetRotZero, armrotatecontrol),
            new FollowPathWithEvents(
                new FollowPath(Left3.get(0), drivetrain, true),
                Left3.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            Commands.runOnce(clawsubsystem::SetClawOn, clawsubsystem),
            Commands.waitSeconds(0.75),
            Commands.runOnce(drivetrain::autoBalance, drivetrain));
    //          Commands.runOnce(drivetrain::autoGyroscope, drivetrain),
    //         Commands.runOnce(drivetrain::enableXstance, drivetrain),
    //         Commands.runOnce(drivetrain::setBrakeOn, drivetrain),
    //         Commands.waitSeconds(2.0),
    //         Commands.runOnce(drivetrain::disableXstance, drivetrain),
    //         Commands.runOnce(drivetrain::setBrakeOff, drivetrain));

    Command autoLeft3long =
        Commands.sequence(
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.runOnce(armrotatecontrol::SetTargetRotStart, armrotatecontrol),
            Commands.runOnce(clawsubsystem::SetClawOn, clawsubsystem),
            Commands.waitSeconds(0.5),
            Commands.runOnce(armcontrol::SetTargetArmHigh, armcontrol),
            Commands.runOnce(armrotatecontrol::SetTargetRotHIGH, armrotatecontrol),
            Commands.waitSeconds(2.0),
            Commands.runOnce(clawsubsystem::stop, clawsubsystem),
            Commands.waitSeconds(0.5),
            Commands.runOnce(armcontrol::SetTargetArmZero, armcontrol),
            Commands.runOnce(armrotatecontrol::SetTargetRotZero, armrotatecontrol),
            new FollowPathWithEvents(
                new FollowPath(Left3long.get(0), drivetrain, true),
                Left3long.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            Commands.runOnce(clawsubsystem::SetClawOn, clawsubsystem),
            Commands.runOnce(drivetrain::autoGyroscope, drivetrain),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.runOnce(drivetrain::setBrakeOn, drivetrain),
            Commands.waitSeconds(3.0),
            Commands.runOnce(drivetrain::disableXstance, drivetrain),
            Commands.runOnce(drivetrain::setBrakeOff, drivetrain));

    Command autoRight1 =
        Commands.sequence(
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.runOnce(armrotatecontrol::SetTargetRotStart, armrotatecontrol),
            Commands.runOnce(clawsubsystem::SetClawOn, clawsubsystem),
            Commands.waitSeconds(0.5),
            Commands.runOnce(armcontrol::SetTargetArmHigh, armcontrol),
            Commands.runOnce(armrotatecontrol::SetTargetRotHIGH, armrotatecontrol),
            Commands.waitSeconds(2.0),
            Commands.runOnce(clawsubsystem::stop, clawsubsystem),
            Commands.waitSeconds(0.5),
            Commands.runOnce(armcontrol::SetTargetArmZero, armcontrol),
            Commands.runOnce(armrotatecontrol::SetTargetRotZero, armrotatecontrol),
            new FollowPathWithEvents(
                new FollowPath(Right1.get(0), drivetrain, true),
                Right1.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            Commands.runOnce(clawsubsystem::SetClawOn, clawsubsystem),
            Commands.waitSeconds(1.0),
            Commands.runOnce(drivetrain::autoBalance, drivetrain));
    //      Commands.runOnce(drivetrain::autoGyroscope, drivetrain),
    //      Commands.runOnce(drivetrain::enableXstance, drivetrain),
    //      Commands.runOnce(drivetrain::setBrakeOn, drivetrain),
    //      Commands.waitSeconds(2.0),
    //      Commands.runOnce(drivetrain::disableXstance, drivetrain),
    //      Commands.runOnce(drivetrain::setBrakeOff, drivetrain));

    Command autoRight2 =
        Commands.sequence(
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.runOnce(armrotatecontrol::SetTargetRotStart, armrotatecontrol),
            Commands.runOnce(clawsubsystem::SetClawOn, clawsubsystem),
            Commands.waitSeconds(0.5),
            Commands.runOnce(armcontrol::SetTargetArmHigh, armcontrol),
            Commands.runOnce(armrotatecontrol::SetTargetRotHIGH, armrotatecontrol),
            Commands.waitSeconds(2.0),
            Commands.runOnce(clawsubsystem::stop, clawsubsystem),
            Commands.waitSeconds(0.5),
            Commands.runOnce(armcontrol::SetTargetArmZero, armcontrol),
            Commands.runOnce(armrotatecontrol::SetTargetRotZero, armrotatecontrol),
            new FollowPathWithEvents(
                new FollowPath(Right2.get(0), drivetrain, true),
                Right2.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            Commands.runOnce(clawsubsystem::SetClawOn, clawsubsystem),
            Commands.waitSeconds(1.0),
            Commands.runOnce(drivetrain::autoBalance, drivetrain));
    //       Commands.runOnce(drivetrain::autoGyroscope, drivetrain),
    //       Commands.runOnce(drivetrain::enableXstance, drivetrain),
    //       Commands.runOnce(drivetrain::setBrakeOn, drivetrain),
    //       Commands.waitSeconds(2.0),
    //       Commands.runOnce(drivetrain::disableXstance, drivetrain),
    //        Commands.runOnce(drivetrain::setBrakeOff, drivetrain));

    Command autoRight3 =
        Commands.sequence(
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.runOnce(armrotatecontrol::SetTargetRotStart, armrotatecontrol),
            Commands.runOnce(clawsubsystem::SetClawOn, clawsubsystem),
            Commands.waitSeconds(0.5),
            Commands.runOnce(armcontrol::SetTargetArmHigh, armcontrol),
            Commands.runOnce(armrotatecontrol::SetTargetRotHIGH, armrotatecontrol),
            Commands.waitSeconds(2.0),
            Commands.runOnce(clawsubsystem::stop, clawsubsystem),
            Commands.waitSeconds(0.5),
            Commands.runOnce(armcontrol::SetTargetArmZero, armcontrol),
            Commands.runOnce(armrotatecontrol::SetTargetRotZero, armrotatecontrol),
            new FollowPathWithEvents(
                new FollowPath(Right3.get(0), drivetrain, true),
                Right3.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            Commands.runOnce(clawsubsystem::SetClawOn, clawsubsystem),
            Commands.waitSeconds(0.75),
            Commands.runOnce(drivetrain::autoBalance, drivetrain));

    Command autoRight3long =
        Commands.sequence(
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.runOnce(armrotatecontrol::SetTargetRotStart, armrotatecontrol),
            Commands.runOnce(clawsubsystem::SetClawOn, clawsubsystem),
            Commands.waitSeconds(0.5),
            Commands.runOnce(armcontrol::SetTargetArmHigh, armcontrol),
            Commands.runOnce(armrotatecontrol::SetTargetRotHIGH, armrotatecontrol),
            Commands.waitSeconds(2.0),
            Commands.runOnce(clawsubsystem::stop, clawsubsystem),
            Commands.waitSeconds(0.5),
            Commands.runOnce(armcontrol::SetTargetArmZero, armcontrol),
            Commands.runOnce(armrotatecontrol::SetTargetRotZero, armrotatecontrol),
            new FollowPathWithEvents(
                new FollowPath(Right3long.get(0), drivetrain, true),
                Right3long.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            Commands.runOnce(clawsubsystem::SetClawOn, clawsubsystem),
            Commands.runOnce(drivetrain::autoGyroscope, drivetrain),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.runOnce(drivetrain::setBrakeOn, drivetrain),
            Commands.waitSeconds(3.0),
            Commands.runOnce(drivetrain::disableXstance, drivetrain),
            Commands.runOnce(drivetrain::setBrakeOff, drivetrain));

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    // Auto Mode Left1
    autoChooser.addOption("Auto Left 1", autoLeft1);

    // Auto Mode Left2
    autoChooser.addOption("Auto Left 2", autoLeft2);

    // Auto Mode Left3
    autoChooser.addOption("Auto Left 3", autoLeft3);

    // Auto Mode Left3 Long
    autoChooser.addOption("Auto Left 3 Long ", autoLeft3long);

    // Auto Mode Right1
    autoChooser.addOption("Auto Right 1", autoRight1);

    // Auto Mode Right2
    autoChooser.addOption("Auto Right 2", autoRight2);

    // Auto Mode Right3
    autoChooser.addOption("Auto Right 3", autoRight3);

    // Auto Mode Right3 Long
    autoChooser.addOption("Auto Right 3 Long", autoRight3long);

    // "auto" command for tuning the drive velocity PID
    autoChooser.addOption(
        "Drive Velocity Tuning",
        Commands.sequence(
            Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
            Commands.deadline(
                Commands.waitSeconds(5.0),
                Commands.run(() -> drivetrain.drive(1.5, 0.0, 0.0), drivetrain))));

    // "auto" command for characterizing the drivetrain
    autoChooser.addOption(
        "Drive Characterization",
        new FeedForwardCharacterization(
            drivetrain,
            true,
            new FeedForwardCharacterizationData("drive"),
            drivetrain::runCharacterizationVolts,
            drivetrain::getCharacterizationVelocity));

    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
