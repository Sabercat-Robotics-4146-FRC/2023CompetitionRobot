package frc4146.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4146.robot.subsystems.DrivetrainSubsystem;


/* Work in progress.
My plan is for this to look at how much of the 9.8m/s/s g is in the z axis of the pigeon's accelerometer,
in addition to it direction in the axis pointing down the ramp(when the ramp is level). 
This would say whether the robot is balanced(when the magnitude in the z axis is close to 1.0g), 
or how unbalanced it is.
*/
public class BalanceRobot extends CommandBase {
    private final DrivetrainSubsystem drivetrain;

    private final Timer timerElapsed;
    private final Timer timer;
    
    public BalanceRobot(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        timer = new Timer();
        timerElapsed = new Timer();
    }




    
}
