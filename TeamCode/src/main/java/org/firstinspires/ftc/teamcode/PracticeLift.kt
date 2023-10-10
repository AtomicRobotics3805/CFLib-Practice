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

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import org.atomicrobotics3805.cflib.Constants.opMode
import org.atomicrobotics3805.cflib.Command
import org.atomicrobotics3805.cflib.hardware.MotorEx
import org.atomicrobotics3805.cflib.hardware.MotorExGroup
import org.atomicrobotics3805.cflib.subsystems.PowerMotor
import org.atomicrobotics3805.cflib.subsystems.Subsystem
import org.atomicrobotics3805.cflib.subsystems.MotorToPosition
import kotlin.math.PI

/**
 * This class is an example of a lift controlled by a single motor. Unlike the Intake example object, it can use
 * encoders to go to a set position. Its first two commands, toLow and toHigh, do just that. The start command turns
 * the motor on and lets it spin freely, and the reverse command does the same but in the opposite direction. The stop
 * command stops the motor. These last three are meant for use during the TeleOp period to control the lift manually.
 * To use this class, copy it into the proper package and change the first eight constants (COUNTS_PER_INCH is fine as
 * is).
 */
@Config
@Suppress("Unused", "MemberVisibilityCanBePrivate")
object PracticeLift : Subsystem {

    var NAME = "lift"
    var SPEED = 0.5
    var DIRECTION = DcMotorSimple.Direction.FORWARD
    val liftMotor: MotorEx = MotorEx(NAME,MotorEx.MotorType.ANDYMARK_NEVEREST, 3.7, DIRECTION)
    var HIGH_POSITION = 30.0
    var LOW_POSITION = 15.0
    private const val PULLY_WIDTH = 0.7
    private const val COUNTS_PER_REV = 28 * 3.7
    private const val DRIVE_GEAR_REDUCTION = 1.0
    private const val COUNTS_PER_INCH = COUNTS_PER_REV * DRIVE_GEAR_REDUCTION / (PULLY_WIDTH * PI)
    val start: Command
        get() = PowerMotor(liftMotor, SPEED)
    val reverse: Command
        get() = PowerMotor(liftMotor, -SPEED)
    val stop: Command
        get() = PowerMotor(liftMotor, 0.0)
    override fun initialize() {
        liftMotor.initialize()
        liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }
    val toBottom: Command
        get() = MotorToPosition(liftMotor, (0.5 * COUNTS_PER_INCH).toInt(), SPEED)
    val toLow: Command
        get() = MotorToPosition(liftMotor, (LOW_POSITION * COUNTS_PER_INCH).toInt(), SPEED)
    val toHigh: Command
        get() = MotorToPosition(liftMotor, (HIGH_POSITION * COUNTS_PER_INCH).toInt(), SPEED)
}