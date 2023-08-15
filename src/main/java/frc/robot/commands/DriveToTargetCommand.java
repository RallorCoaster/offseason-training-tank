package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightInterface;
import frc.robot.subsystems.Drive;

public class DriveToTargetCommand extends CommandBase {
    private double targetLimelightTY;
    private Drive drive;
    private PIDController driveController;

    private double minSpeed = 0.05;

    public DriveToTargetCommand(double targetLimelightTY, Drive drive) {
        this.targetLimelightTY = targetLimelightTY;
        this.drive = drive;

        driveController = new PIDController(0.01, 0, 0);

        driveController.setTolerance(0.3); //max tolerance for error in degrees (or radians??)

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        driveController.reset(); //resets any values within angleController
        driveController.setSetpoint(targetLimelightTY); //assumingly sets the point which it is currently at as 0 degrees.
    }

    @Override
    public void execute() {
        double output = driveController.calculate(LimelightInterface.getTY()); //ask if access lli from static or non static
        output = Math.copySign(minSpeed, output) + output; /*minspeed defines the value that will be returned. if output is positive,
        then minspeed will be returned as positive minspeed, if output is negative, then minspeed will be returned as negative minspeed*/

        drive.driveTank(output, output);
    }

    @Override
    public boolean isFinished() {
        return driveController.atSetpoint(); //returns whether the gyro senses that the robot is at 0 degrees (?)
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.stop(); //stop doesn't exist?
    }
}