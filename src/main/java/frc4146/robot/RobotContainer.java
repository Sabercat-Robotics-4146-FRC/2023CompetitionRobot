package frc4146.robot;

import static frc4146.robot.Constants.Setpoints.*;

import common.drivers.Gyroscope;
import common.robot.input.DPadButton.Direction;
import common.robot.input.XboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc4146.robot.commands.drivetrain.DriveCommand;
import frc4146.robot.commands.subsystems.AlignWithFiducial;
import frc4146.robot.commands.subsystems.ArmCommand;
import frc4146.robot.commands.subsystems.ClawCommand;
import frc4146.robot.commands.subsystems.SetArmPosition;
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
                secondaryController.getRightYAxis(),
                secondaryController.getLeftBumperButton(),
                secondaryController.getRightBumperButton(),
                secondaryController.getDPadButton(Direction.CENTER),
                secondaryController.getAButton(),
                secondaryController.getBButton(),
                secondaryController.getYButton()));

    CommandScheduler.getInstance()
        .setDefaultCommand(claw, new ClawCommand(claw, secondaryController.getLeftXAxis()));

    // enables drive using controller

    CameraServer.startAutomaticCapture();

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    primaryController.getStartButton().onTrue(Commands.runOnce(gyroscope::calibrate));
    primaryController.getStartButton().onTrue(Commands.runOnce(drivetrainSubsystem::zeroWheels));
    primaryController
        .getYButton()
        .onTrue(Commands.runOnce(drivetrainSubsystem::toggleFieldOriented));
    primaryController.getXButton().onTrue(Commands.runOnce(drivetrainSubsystem::toggleDriveFlag));

    // primaryController.getBButton().onTrue(Commands.runOnce(drivetrainSubsystem::lockWheels));

    // primaryController.getBButton().toggleOnTrue(new BalanceRobot(drivetrainSubsystem,
    // gyroscope));

    primaryController.getBButton().onTrue(new AlignWithFiducial(drivetrainSubsystem, limelight));

    // secondaryController.getAButton().onTrue(Commands.runOnce(arm::toggleExtensionMode));
    // secondaryController.getBButton().onTrue(Commands.runOnce(arm::toggleRotationMode));
    // secondaryController.getBButton().toggleOnTrue(new ArmRotate(arm));

    // secondaryController
    //    .getBButton().onTrue(new InstantCommand(() -> arm.rotateArm()));
    // .toggleOnTrue(new InstantCommand(() -> arm.extendArm(arm.getPos())));

    secondaryController
        .getLeftBumperButton()
        .or(secondaryController.getRightBumperButton())
        .and(secondaryController.getDPadButton(Direction.CENTER))
        .onTrue(new SetArmPosition(arm, intake_pos[0], intake_pos[1]));

    secondaryController
        .getLeftBumperButton()
        .and(secondaryController.getAButton())
        .onTrue(new SetArmPosition(arm, cone_low[0], cone_low[1]));

    secondaryController
        .getLeftBumperButton()
        .and(secondaryController.getBButton())
        .onTrue(new SetArmPosition(arm, cone_mid[0], cone_mid[1]));

    secondaryController
        .getLeftBumperButton()
        .and(secondaryController.getYButton())
        .onTrue(new SetArmPosition(arm, cone_high[0], cone_high[1]));

    secondaryController
        .getRightBumperButton()
        .and(secondaryController.getAButton())
        .onTrue(new SetArmPosition(arm, cube_low[0], cube_low[1]));

    secondaryController
        .getRightBumperButton()
        .and(secondaryController.getBButton())
        .onTrue(new SetArmPosition(arm, cube_mid[0], cube_mid[1]));

    secondaryController
        .getRightBumperButton()
        .and(secondaryController.getYButton())
        .onTrue(new SetArmPosition(arm, cube_high[0], cube_high[1]));
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
