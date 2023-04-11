
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




package org.firstinspires.ftc.teamcode;

/**
 * IMU axes signs in the order XYZ (after remapping).
 */
public enum MyAxesSigns {
    PPP(0b000),
    PPN(0b001),
    PNP(0b010),
    PNN(0b011),
    NPP(0b100),
    NPN(0b101),
    NNP(0b110),
    NNN(0b111);

    public final int bVal;

    MyAxesSigns(int bVal) {
        this.bVal = bVal;
    }

    public static MyAxesSigns fromBinaryValue(int bVal) {
        int maskedVal = bVal & 0x07;
        switch (maskedVal) {
            case 0b000:
                return MyAxesSigns.PPP;
            case 0b001:
                return MyAxesSigns.PPN;
            case 0b010:
                return MyAxesSigns.PNP;
            case 0b011:
                return MyAxesSigns.PNN;
            case 0b100:
                return MyAxesSigns.NPP;
            case 0b101:
                return MyAxesSigns.NPN;
            case 0b110:
                return MyAxesSigns.NNP;
            case 0b111:
                return MyAxesSigns.NNN;
            default:
                throw new IllegalStateException("Unexpected value for maskedVal: " + maskedVal);
        }
    }
}