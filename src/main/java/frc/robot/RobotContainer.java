package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import frc.robot.commands.SwerveCmd;
import frc.robot.commands.ToggleSolenoidCmd;
import frc.robot.commands.SetDriveBrakeCmd;
import frc.robot.commands.SetDriveCoastCmd;
import frc.robot.commands.SetPoseCmd;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  private final CommandPS4Controller controller = new CommandPS4Controller(ControllerConstants.CONTROLLER_PORT);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final PneumaticsSubsystem pneumaticSubsystem = new PneumaticsSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  private final ToggleSolenoidCmd toggleSolenoid = new ToggleSolenoidCmd(pneumaticSubsystem);
  private final SwerveCmd joystickSwerve = new SwerveCmd(
    swerveSubsystem, 
    () -> controller.getLeftX(), 
    () -> controller.getLeftY(), 
    () -> -controller.getRightX(),
    controller.L1(),
    controller.L2());
  private final SetPoseCmd resetPose = new SetPoseCmd(swerveSubsystem, DriveConstants.ZERO_POSE);
  private final SetDriveBrakeCmd setDriveBrake = new SetDriveBrakeCmd(swerveSubsystem);
  private final SetDriveCoastCmd setDriveCoast = new SetDriveCoastCmd(swerveSubsystem);

  public static ShuffleboardTab robotStatus = Shuffleboard.getTab("Robot");
  
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();

    swerveSubsystem.setDefaultCommand(joystickSwerve);

    // autoChooser.setDefaultOption("Toggle Solenoid", toggleSolenoid);
    // autoChooser.addOption("Reset Pose", resetPose);

    autoChooser = AutoBuilder.buildAutoChooser(); 

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    controller.L2().onTrue(toggleSolenoid);

    controller.R1().onTrue(resetPose);

    controller.button(0).onTrue(setDriveBrake);
    controller.button(1).onTrue(setDriveCoast);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
