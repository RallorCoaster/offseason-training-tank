package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.LimelightInterface;
import edu.wpi.first.math.util.Units;

public class DriveAndAimTargetCommand extends CommandBase{
    private double targetLimelightTY; 
    private Drive drive;
    private PIDController driveController; 
    private LimelightInterface limelightInterface; 
    private PIDController angleController; 
    private double minSpeed = 0.05;
    
    public DriveAndAimTargetCommand(double targetLimelightTY, Drive drive, LimelightInterface limelightInterface){
        this.targetLimelightTY = targetLimelightTY; 
        this.drive = drive;
        this.limelightInterface = limelightInterface; 

        angleController = new PIDController(.001, 0, 0);
        driveController = new PIDController(0.001, 0, 0);

        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(Units.degreesToRadians(0.2));
        driveController.setTolerance(0.3);

        addRequirements(drive);
    }
    
    @Override
    public void initialize(){
        angleController.reset();
        driveController.reset();
        angleController.setSetpoint(0);
        driveController.setSetpoint(targetLimelightTY);
    }

    @Override
    public void execute() {
        double output = angleController.calculate(Units.degreesToRadians(limelightInterface.getTX()));
        output = Math.copySign(minSpeed, output) + output; 
        
        double ooutput = driveController.calculate(limelightInterface.getTY());
        ooutput = Math.copySign(minSpeed, ooutput) + ooutput; 

        drive.DriveTank(ooutput - output, ooutput + output);
        
    }

    @Override 
    public boolean isFinished() {
        return driveController.atSetpoint() && angleController.atSetpoint();
        
    }
    
    @Override
    public void end(boolean interrupted){
        drive.stop();
    }


}
