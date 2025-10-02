package frc.robot.subsystems.swerve;


import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.subsystems.swerve.SwerveIO.SwerveIOInputs;

public class SwerveDrive extends SubsystemBase{
   
    private final SwerveIO io;
    private final SwerveIOInputs inputs = new SwerveIOInputs();
    private boolean isSwerveinit = false;
    private SysIdRoutine sysIdTranslation;
    private SysIdRoutine sysIdRotation;
    private SysIdRoutine sysIdSteer;
    
    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private SwerveDrive(SwerveIO io) {
        this.io = io;

            sysIdTranslation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(7),
                null,
                // TODO see if signalLogger is better than Logger
                (state) -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
            output -> runSwervesTranslation(m_translationCharacterization.withVolts(output).VoltsToApply),
            null,
            this)
            );

             sysIdRotation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // default ramprate is 1 V/s
                null, // default dynamic voltage is 7 V
                null, // default timeOut is 10s
                // TODO see if signalLogger is better than Logger
                (state) -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
            output -> runSwervesRotation(m_rotationCharacterization.withRotationalRate(output.in(Volts)).RotationalRate),
            null,
            this)
            );
            
            sysIdSteer =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // default ramprate is 1 V/s
                null, // default dynamic voltage is 7 V
                null, // default timeOut is 10s
                // TODO see if signalLogger is better than Logger
                (state) -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
            output -> runSwervesTranslation(m_translationCharacterization.withVolts(output).VoltsToApply),
            null,
            this)
            );
             
                    }
                
     /**
     * Basic lock to prevent multiple swerve initialisations
     * @return the class's constructor
     */
    public SwerveDrive initSwerves() {
        if (!isSwerveinit) {
           isSwerveinit = true; 
           return this;
        }
        else {
            return null;
        }
    }

    public void runSwervesTranslation(double voltage) {
        io.driveSwerve(voltage);
    }
    public void runSwervesRotation(double rotationRadianPerSec) {
        io.rotateSwerve(rotationRadianPerSec);
    }
}
