package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightInterface;
import frc.robot.subsystems.Drive;

public class AimToTargetCommand extends CommandBase {
    private Drive drive;
    private PIDController angleController;

    public double minSpeed = 0.05;
    
    public AimToTargetCommand(Drive drive) {
        this.drive = drive;

        angleController = new PIDController(0.18, 0, 0); //Proportional, Integral, Derivative.

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(Units.degreesToRadians(0.2)); //Allows only 0.2 degrees of error (though this is usually not possible to achieve)

        addRequirements(drive);


    }

    @Override
    public void initialize() {
        angleController.reset(); //resets any values within angleController
        angleController.setSetpoint(0); //assumingly sets the point which it is currently at as 0 degrees.
    }

    @Override
    public void execute() {
        double output = angleController.calculate(Units.degreesToRadians(LimelightInterface.getTX())); //ask if access lli from static or non static
        output = Math.copySign(minSpeed, output) + output; /*minspeed defines the value that will be returned. if output is positive,
        then minspeed will be returned as positive minspeed, if output is negative, then minspeed will be returned as negative minspeed*/

        drive.driveTank(-output, output); //replace drivePercent() with driveTank()
    }

    @Override
    public boolean isFinished() {
        return angleController.atSetpoint(); //returns whether the gyro senses that the robot is at 0 degrees (?)
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.stop(); //stop doesn't exist?
    }
}
