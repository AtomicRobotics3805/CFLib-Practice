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
package com.atomicrobotics.cflib.example.localizers

import com.atomicrobotics.cflib.driving.localizers.TwoWheelOdometryConstants

@Suppress("PropertyName")
class ExampleOdometryConstants : TwoWheelOdometryConstants {
    @JvmField
    var _PARALLEL_X = 0.0 // in; forward offset of the parallel wheel
    @JvmField
    var _PARALLEL_Y = 0.0 // in; left offset of the parallel wheel
    @JvmField
    var _PERPENDICULAR_X = 0.0 // in; forward offset of the perpendicular wheel
    @JvmField
    var _PERPENDICULAR_Y = 0.0 // in; left offset of the perpendicular wheel
    @JvmField
    var _PARALLEL_NAME = "LF" // the name of the wheel that the parallel deadwheel encoder is plugged into
    @JvmField
    var _PERPENDICULAR_NAME = "LB" // the name of the wheel that the perpendicular deadwheel encoder is plugged into
    @JvmField
    var _TICKS_PER_REV = 8192.0 // REV through bore encoder
    @JvmField
    var _WHEEL_RADIUS = 0.688975 // rotacaster wheels // in
    @JvmField
    var _GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed
    @JvmField
    var _PARALLEL_REVERSED = false
    @JvmField
    var _PERPENDICULAR_REVERSED = false
    @JvmField
    var _X_MULTIPLIER = 1.0
    @JvmField
    var _Y_MULTIPLIER = 1.0
    @JvmField
    var _CORRECTED_VELOCITY = true

    override val PARALLEL_X: Double
        get() = _PARALLEL_X
    override val PARALLEL_Y: Double
        get() = _PARALLEL_Y
    override val PERPENDICULAR_X: Double
        get() = _PERPENDICULAR_X
    override val PERPENDICULAR_Y: Double
        get() = _PERPENDICULAR_Y
    override val PARALLEL_NAME: String
        get() = _PARALLEL_NAME
    override val PERPENDICULAR_NAME: String
        get() = _PERPENDICULAR_NAME
    override val TICKS_PER_REV: Double
        get() = _TICKS_PER_REV
    override val WHEEL_RADIUS: Double
        get() = _WHEEL_RADIUS
    override val GEAR_RATIO: Double
        get() = _GEAR_RATIO
    override val PARALLEL_REVERSED: Boolean
        get() = _PARALLEL_REVERSED
    override val PERPENDICULAR_REVERSED: Boolean
        get() = _PERPENDICULAR_REVERSED
    override val X_MULTIPLIER: Double
        get() = _X_MULTIPLIER
    override val Y_MULTIPLIER: Double
        get() = _Y_MULTIPLIER
    override val CORRECTED_VELOCITY: Boolean
        get() = _CORRECTED_VELOCITY
}