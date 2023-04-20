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
package com.atomicrobotics.cflib.trajectories

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import com.atomicrobotics.cflib.Constants.Color.BLUE
import com.atomicrobotics.cflib.Constants.color
//https://play.kotlinlang.org/#eyJ2ZXJzaW9uIjoiMS44LjEwIiwicGxhdGZvcm0iOiJqYXZhIiwiYXJncyI6IiIsIm5vbmVNYXJrZXJzIjp0cnVlLCJ0aGVtZSI6ImlkZWEiLCJjb2RlIjoiaW1wb3J0IGtvdGxpbi5tYXRoLipcblxuLyoqXG4gKiBZb3UgY2FuIGVkaXQsIHJ1biwgYW5kIHNoYXJlIHRoaXMgY29kZS5cbiAqIHBsYXkua290bGlubGFuZy5vcmdcbiAqL1xuZnVuIG1haW4oKSB7XG4gICAgdmFsIGRyaXZlUG93ZXI6IFBvc2UyZFxuICAgIHZhbCBhbmdsZTogRG91YmxlID0gaWYgKGdhbWVwYWQubGVmdF9zdGlja194ICE9IDAuMGYpXG4gICAgICAgICAgICAgICAgYXRhbihnYW1lcGFkLmxlZnRfc3RpY2tfeSAvIGdhbWVwYWQubGVmdF9zdGlja194KS50b0RvdWJsZSgpXG4gICAgICAgICAgICBlbHNlIDkwLjAudG9SYWRpYW5zXG5cbiAgICAgICAgICAgIHZhciBhZGp1c3RlZEFuZ2xlID0gYW5nbGUgLSBkcml2ZS5wb3NlRXN0aW1hdGUuaGVhZGluZ1xuICAgICAgICAgICAgdmFsIHRvdGFsUG93ZXIgPSBzcXJ0KGdhbWVwYWQubGVmdF9zdGlja195LnBvdygyKSArIGdhbWVwYWQubGVmdF9zdGlja194LnBvdygyKSlcbiAgICAgICAgICAgIGRyaXZlUG93ZXIgPSBQb3NlMmQoXG4gICAgICAgICAgICAgICAgdG90YWxQb3dlciAqIHNpbihhZGp1c3RlZEFuZ2xlKSAqIGlmIChnYW1lcGFkLmxlZnRfc3RpY2tfeCAhPSAwLjBmKVxuICAgICAgICAgICAgICAgICAgICAgICAgc2lnbihnYW1lcGFkLmxlZnRfc3RpY2tfeCkgZWxzZSAxLjBmICogaWYgKGdhbWVwYWQubGVmdF9zdGlja195ICE9IDAuMGYpXG4gICAgICAgICAgICAgICAgICAgICAgICBzaWduKGdhbWVwYWQubGVmdF9zdGlja195KSBlbHNlIDEuMGYsXG4gICAgICAgICAgICAgICAgdG90YWxQb3dlciAqIGNvcyhhZGp1c3RlZEFuZ2xlKSxcbiAgICAgICAgICAgICAgICAoZ2FtZXBhZC5yaWdodF9zdGlja194KS50b0RvdWJsZSgpXG4gICAgICAgICAgICApXG4gICAgIHByaW50bG4oXCJYOiAkeyBkcml2ZVBvd2VyLnggfSwgWTogJHsgZHJpdmVQb3dlci55IH0sIEhlYWRpbmc6ICR7IGRyaXZlUG93ZXIuaGVhZGluZyB9XCIpXG59XG5cbm9iamVjdCBnYW1lcGFkIHtcbiAgICB2YWwgbGVmdF9zdGlja194ID0gMS4wZlxuICAgIHZhbCBsZWZ0X3N0aWNrX3kgPSAwLjBmXG4gICAgdmFsIHJpZ2h0X3N0aWNrX3ggPSAxLjBmXG59XG5cbm9iamVjdCBkcml2ZSB7XG4gICAgdmFsIHBvc2VFc3RpbWF0ZSA9IFBvc2UyZCgwLjAsIDAuMCwgTWF0aC50b1JhZGlhbnMoOTAuMCkpXG59XG5cbmNsYXNzIFBvc2UyZCh2YWwgeDogRG91YmxlLCB2YWwgeTogRG91YmxlLCB2YWwgaGVhZGluZzogRG91YmxlKVxuXG52YWwgRG91YmxlLnRvUmFkaWFucyBnZXQoKSA9IChNYXRoLnRvUmFkaWFucyh0aGlzKSkifQ==
val Double.inchesToMm get() = this * 25.4
val Double.mmToInches get() = this / 25.4
val Double.toRadians get() = (Math.toRadians(this))
val Double.switchColorAngle get () = (if (color == BLUE) this else 360 - this)
val Double.switchApproachTangentAngle get () = (if (color == BLUE) this else this - 180)
val Double.switchColor get () = (if (color == BLUE) this else this * -1)
val Double.flipAlongX36 get() = (if (color == BLUE) this else 72 - this)
fun Pose2d(matrix: OpenGLMatrix) = Pose2d(
    matrix.translation.get(0).toDouble().mmToInches,
    // TODO("Figure out where this is matrix.get(1) (y) or matrix.get(2) (z)")
    matrix.translation.get(1).toDouble().mmToInches,
    Orientation.getOrientation(matrix, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES)
        .thirdAngle.toDouble().toRadians
)