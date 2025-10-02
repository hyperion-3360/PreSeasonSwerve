package frc.robot.subsystems.swerve;

import java.util.HashMap;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {
   
    @AutoLog
    public static class SwerveIOInputs {
    
        
    }
    /**
     * translates the swerves forward with the input voltage
     * @param translationVoltageInput the input voltage to use so that we drive forward
     */
    public void driveSwerve(double translationVoltageInput);

    /**
     * rotates the swerve with a speed corresponding to the rotation rate
     * @param rotationRateInput rotation rate in radians per seconds
     */
    public void rotateSwerve(double rotationRateInput);
}
