
/*
    Copyright (c) 2022 Atomic Robotics (https://atomicrobotics3805.org)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see https://www.gnu.org/licenses/.
*/




package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import org.atomicrobotics3805.cflib.Command
import org.atomicrobotics3805.cflib.CommandScheduler
import org.atomicrobotics3805.cflib.Constants
import org.atomicrobotics3805.cflib.driving.DriverControlled
import org.atomicrobotics3805.cflib.driving.MecanumDriveConstants
import org.atomicrobotics3805.cflib.driving.drivers.Driver
import org.atomicrobotics3805.cflib.driving.localizers.SubsystemLocalizer
import org.atomicrobotics3805.cflib.roadrunner.BNO055IMUUtil
import org.atomicrobotics3805.cflib.roadrunner.LynxModuleUtil
import org.atomicrobotics3805.cflib.roadrunner.MyLynxModuleUtil
import java.lang.Thread.sleep
import java.util.*

/**
 * This object controls the movement of a mecanum drivetrain. It's pretty complicated, so it might
 * be tough to understand everything at first. The normal wrapping & spacing rules aren't always
 * followed because they can make the file more difficult to understand.
 *
 * If you want to have multiple Mecanum Drives in the same project, you don't need to make another
 * copy of this object. You just need to make another object that inherits DriveConstants and set
 * that as the one used in the Constants file.
 */
@Suppress("unused")
class MyMecanumDrive(val mecanumDriveConstants: MecanumDriveConstants,
                   localizer: SubsystemLocalizer,
                   startPose: () -> Pose2d = { Pose2d() }
) : Driver(mecanumDriveConstants, localizer, startPose) {

    // this constraint is used when building trajectories to determine how fast the robot will go
    override val velConstraint: MinVelocityConstraint
        get() = MinVelocityConstraint(listOf(
            AngularVelocityConstraint(constants.MAX_ANG_VEL),
            MecanumVelocityConstraint(constants.MAX_VEL, constants.TRACK_WIDTH)
        ))

    // drive motors, battery voltage sensor, IMU, and the OpMode's hardwareMap
    private lateinit var leftFront: DcMotorEx
    private lateinit var leftBack: DcMotorEx
    private lateinit var rightBack: DcMotorEx
    private lateinit var rightFront: DcMotorEx
    private lateinit var motors: List<DcMotorEx>

    override val rawExternalHeading: Double
        get() = imu.angularOrientation.firstAngle.toDouble()
    // To work around an SDK bug, use -zRotationRate in place of xRotationRate
    // and -xRotationRate in place of zRotationRate (yRotationRate behaves as
    // expected). This bug does NOT affect orientation.
    //
    // See https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251 for details.
    override val externalHeadingVelocity: Double
        get() = -imu.angularVelocity.xRotationRate.toDouble()

    /**
     * Allows the drivers to control the drivetrain using a gamepad
     * @param gamepad the gamepad that controls the drivetrain
     */
    override fun driverControlled(gamepad: Gamepad): Command = DriverControlled(gamepad, listOf(this), true, mecanumDriveConstants.POV, mecanumDriveConstants.REVERSE_STRAFE, mecanumDriveConstants.REVERSE_STRAIGHT, mecanumDriveConstants.REVERSE_TURN)

    /**
     * Initializes the drivetrain. This includes initializing the IMU, motor, and the battery
     * voltage sensor.
     */
    override fun initialize() {

        Constants.opMode.telemetry.addLine("3.1")
        Constants.opMode.telemetry.update()
        hardwareMap = Constants.opMode.hardwareMap
        Constants.opMode.telemetry.addLine("3.2")
        Constants.opMode.telemetry.update()
        // initializes the imu
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)
        Constants.opMode.telemetry.addLine("3.3")
        Constants.opMode.telemetry.update()
        // if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        //BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        BNO055IMUUtil.remapZAxis(imu, constants.VERTICAL_AXIS)
        Constants.opMode.telemetry.addLine("3.4")
        Constants.opMode.telemetry.update()
        // initializes the batteryVoltageSensor
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        Constants.opMode.telemetry.addLine("3.5")
        Constants.opMode.telemetry.update()
        // makes sure the turnController knows to go from 0 to 2pi (a full rotation in radians)
        turnController.setInputBounds(0.0, 2 * Math.PI)
        Constants.opMode.telemetry.addLine("3.6")
        Constants.opMode.telemetry.update()
        MyLynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)
        Constants.opMode.telemetry.addLine("3.7")
        Constants.opMode.telemetry.update()
        sleep(1000)
        // honestly I don't know what this does
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
        Constants.opMode.telemetry.addLine("3.8")
        Constants.opMode.telemetry.update()
        sleep(1000)
        CommandScheduler.registerSubsystems(localizer)
        localizer.poseEstimate = Pose2d()
        Constants.opMode.telemetry.addLine("3.9")
        Constants.opMode.telemetry.update()
        sleep(1000)

        Constants.opMode.telemetry.addLine("2.1")
        Constants.opMode.telemetry.update()
        //super.initialize()
        Constants.opMode.telemetry.addLine("2.2")
        Constants.opMode.telemetry.update()
        // initializes the motors
        constants as MecanumDriveConstants
        leftFront = hardwareMap.get(DcMotorEx::class.java, (constants as MecanumDriveConstants).LEFT_FRONT_NAME)
        leftBack = hardwareMap.get(DcMotorEx::class.java, (constants as MecanumDriveConstants).LEFT_BACK_NAME)
        rightBack = hardwareMap.get(DcMotorEx::class.java, (constants as MecanumDriveConstants).RIGHT_BACK_NAME)
        rightFront = hardwareMap.get(DcMotorEx::class.java, (constants as MecanumDriveConstants).RIGHT_FRONT_NAME)
        Constants.opMode.telemetry.addLine("2.3")
        Constants.opMode.telemetry.update()
        motors = listOf(leftFront, leftBack, rightBack, rightFront)
        // sets the achieveableMaxRPMFraction for each motor to 1.0
        for (motor in motors) {
            val motorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }
        Constants.opMode.telemetry.addLine("2.4")
        Constants.opMode.telemetry.update()
        // sets the RunMode for each motor
        if (constants.IS_RUN_USING_ENCODER) {
            for (motor in motors) {
                motor.mode = RunMode.STOP_AND_RESET_ENCODER
                motor.mode = RunMode.RUN_USING_ENCODER
            }
            // sets the motors' PIDFCoefficients
            setPIDFCoefficients(constants.MOTOR_VEL_PID)
        }
        else {
            for (motor in motors) {
                motor.mode = RunMode.RUN_WITHOUT_ENCODER
            }
        }
        Constants.opMode.telemetry.addLine("2.5")
        Constants.opMode.telemetry.update()
        // sets the zero power behavior for each motor
        for (motor in motors) {
            motor.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        }
        Constants.opMode.telemetry.addLine("2.6")
        Constants.opMode.telemetry.update()
        // reverses motors if necessary
        leftBack.direction = (constants as MecanumDriveConstants).LEFT_BACK_DIRECTION
        leftFront.direction = (constants as MecanumDriveConstants).LEFT_FRONT_DIRECTION
        rightBack.direction = (constants as MecanumDriveConstants).RIGHT_BACK_DIRECTION
        rightFront.direction = (constants as MecanumDriveConstants).RIGHT_FRONT_DIRECTION
        Constants.opMode.telemetry.addLine("2.7")
        Constants.opMode.telemetry.update()
    }

    /**
     * Returns a list of how far each wheel has turned in inches
     * @return how far each wheel has turned in inches
     */
    fun getWheelPositions(): List<Double> {
        val wheelPositions: MutableList<Double> = ArrayList()
        for (motor in motors) {
            wheelPositions.add(constants.encoderTicksToInches(motor.currentPosition.toDouble()))
        }
        return wheelPositions
    }

    /**
     * Returns a list of how fast each wheel is turning in inches per second
     * @return how fast each wheel is turning in inches per second
     */
    fun getWheelVelocities(): List<Double> {
        val wheelVelocities: MutableList<Double> = ArrayList()
        for (motor in motors) {
            wheelVelocities.add(constants.encoderTicksToInches(motor.velocity))
        }
        return wheelVelocities
    }

    /**
     * Sets power to each of the motors
     * @param frontLeft the power for the front left motor
     * @param backLeft the power for the back left motor
     * @param backRight the power for the back right motor
     * @param frontRight the power for the front right motor
     */
    private fun setMotorPowers(frontLeft: Double, backLeft: Double, backRight: Double, frontRight: Double) {
        leftFront.power = frontLeft
        leftBack.power = backLeft
        rightBack.power = backRight
        rightFront.power = frontRight
    }

    /**
     * Sets the robot's drive signal. This drive signal includes a target velocity & acceleration.
     * @param driveSignal the target velocity and acceleration of the robot
     */
    override fun setDriveSignal(driveSignal: DriveSignal) {
        val velocities = MecanumKinematics.robotToWheelVelocities(
            driveSignal.vel,
            constants.TRACK_WIDTH,
            constants.TRACK_WIDTH,
            (constants as MecanumDriveConstants).LATERAL_MULTIPLIER
        )
        val accelerations = MecanumKinematics.robotToWheelAccelerations(
            driveSignal.accel,
            constants.TRACK_WIDTH,
            constants.TRACK_WIDTH,
            (constants as MecanumDriveConstants).LATERAL_MULTIPLIER
        )
        val powers = Kinematics.calculateMotorFeedforward(
            velocities,
            accelerations,
            constants.kV,
            constants.kA,
            constants.kStatic
        )
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
    }

    /**
     * Sets the robot's drive power as a Pose2d, which has x, y, and heading components.
     * @param drivePower the target forwards/backwards, left/right, and turn speeds for the robot
     */
    override fun setDrivePower(drivePower: Pose2d) {
        val powers = MecanumKinematics.robotToWheelVelocities(
            drivePower,
            1.0,
            1.0,
            (constants as MecanumDriveConstants).LATERAL_MULTIPLIER
        )
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
    }

    /**
     * Sets the PIDF coefficients for the built-in velocity control on the motors. Adjusts the f
     * value based on the battery voltage.
     * @param coefficients the PIDF coefficients to set the motors to
     */
    fun setPIDFCoefficients(coefficients: PIDFCoefficients) {
        val compensatedCoefficients = PIDFCoefficients(
            coefficients.p, coefficients.i, coefficients.d,
            coefficients.f * 12 / batteryVoltageSensor.voltage
        )
        for (motor in motors) {
            motor.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, compensatedCoefficients)
        }
    }

    fun setMode(mode: RunMode) {
        for (motor in motors) {
            motor.mode = mode
        }
    }
}