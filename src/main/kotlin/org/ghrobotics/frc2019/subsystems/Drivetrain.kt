package org.ghrobotics.frc2019.subsystems

import com.ctre.phoenix.sensors.PigeonIMU
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import org.ghrobotics.frc2019.commands.TeleopDriveCommand
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.SlopeNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.physics.MotorCharacterization
import org.ghrobotics.lib.subsystems.drive.FalconWestCoastDrivetrain
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.asSource
import sun.security.util.Length

object Drivetrain : FalconWestCoastDrivetrain() {

    // Constants are at the top of each subsystem.
    // These must be private.
    private const val kLeftMasterId = 1
    private const val kLeftSlave1Id = 2
    private const val kRightMasterId = 3
    private const val kRightSlave1Id = 4
    private val kDriveNativeUnitModel = SlopeNativeUnitModel(138.inch, 10000.nativeUnits)


    private const val kPigeonId = 17

    private const val kBeta = 2.0
    private const val kZeta = 0.7

    // Private member variables
    private val pigeon = PigeonIMU(kPigeonId)

    // Overriden variables

    // Motors
    override val leftMotor: FalconMotor<Meter> = configureGearbox(kLeftMasterId, kLeftSlave1Id, setInverted = false)
    override val rightMotor: FalconMotor<Meter> = configureGearbox(kLeftMasterId, kLeftSlave1Id, setInverted = true)
    private val leftSlave1: FalconMotor<Meter> = TODO()
    private val rightSlave1: FalconMotor<Meter> = TODO()

    // Motor characterization
    override val leftCharacterization: MotorCharacterization<Meter> = TODO()
    override val rightCharacterization: MotorCharacterization<Meter> = TODO()

    // Gyro
    override val gyro: Source<Rotation2d> = pigeon.asSource()

    // Kinematics
    override val kinematics: DifferentialDriveKinematics = TODO()

    // Odometry
    override val odometry = DifferentialDriveOdometry(kinematics)

    // Trajectory tracker
    override val trajectoryTracker = RamseteTracker(kBeta, kZeta)

    // Constructor
    init {
        defaultCommand = TeleopDriveCommand()
    }

    // Emergency management.
    override fun activateEmergency() {
        TODO()
    }

    override fun recoverFromEmergency() {
        TODO()
    }


    private fun configureGearbox(masterId: Int, slaveId: Int, setInverted: Boolean): FalconMotor<Meter> {
        val masterMotor = FalconSRX(masterId, kDriveNativeUnitModel)
        val slaveMotor = FalconSRX(slaveId, kDriveNativeUnitModel)
        return masterMotor
    }



}