package frc4146.robot;

import common.drivers.Gyroscope;
import common.robot.input.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc4146.robot.commands.drivetrain.DriveCommand;
import frc4146.robot.commands.subsystems.AlignWithFiducial;
import frc4146.robot.commands.subsystems.ArmCommand;
import frc4146.robot.commands.subsystems.ClawCommand;
import frc4146.robot.subsystems.*;

public class RobotContainer {

  private PowerDistribution pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

  private final XboxController primaryController =
      new XboxController(Constants.PRIMARY_CONTROLLER_PORT);

  private final XboxController secondaryController =
      new XboxController(Constants.SECONDARY_CONTROLLER_PORT);

  private final Pigeon gyroscope = new Pigeon(Constants.DriveConstants.PIGEON_PORT);
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(gyroscope);

  private final Limelight limelight = new Limelight();

  private final Arm arm = new Arm();
  private final Claw claw = new Claw();

  public RobotContainer() {
    pdh.setSwitchableChannel(true);

    CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
    CommandScheduler.getInstance().registerSubsystem(arm);
    CommandScheduler.getInstance().registerSubsystem(claw);

    CommandScheduler.getInstance()
        .setDefaultCommand(
            drivetrainSubsystem,
            new DriveCommand(
                drivetrainSubsystem,
                primaryController.getLeftYAxis(),
                primaryController.getLeftXAxis(),
                primaryController.getRightXAxis()));

    CommandScheduler.getInstance()
        .setDefaultCommand(
            arm,
            new ArmCommand(
                arm,
                secondaryController.getLeftTriggerAxis(),
                secondaryController.getRightTriggerAxis(),
                secondaryController.getRightYAxis()));
    CommandScheduler.getInstance()
        .setDefaultCommand(claw, new ClawCommand(claw, secondaryController.getLeftXAxis()));

    // enables drive using controller

    CameraServer.startAutomaticCapture();

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // TODO: Configure Button Bindings
    primaryController.getStartButton().onTrue(Commands.runOnce(gyroscope::calibrate));
    primaryController
        .getYButton()
        .onTrue(Commands.runOnce(drivetrainSubsystem::toggleFieldOriented));
    primaryController.getXButton().onTrue(Commands.runOnce(drivetrainSubsystem::toggleDriveFlag));

    // primaryController.getBButton().toggleOnTrue(new BalanceRobot(drivetrainSubsystem,
    // gyroscope));

    primaryController.getBButton().onTrue(new AlignWithFiducial(drivetrainSubsystem, limelight));
    // secondaryController.getBButton().toggleOnTrue(new ArmRotate(arm));

    // secondaryController
    //    .getBButton().onTrue(new InstantCommand(() -> arm.rotateArm()));
    // .toggleOnTrue(new InstantCommand(() -> arm.extendArm(arm.getPos())));

    // secondaryControlelr.get
  }

  public Arm getArmSubsystem() {
    return arm;
  }

  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return drivetrainSubsystem;
  }

  public Gyroscope getGyroscope() {
    return gyroscope;
  }
}
