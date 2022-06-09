#!/usr/bin/python3
#
# This file is part of IvPID.
# Copyright (C) 2015 Ivmech Mechatronics Ltd. <bilgi@ivmech.com>
#
# IvPID is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# IvPID is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# title           : test_pid.py
# description     : python pid controller test
# author          : Caner Durmusoglu
# date            : 20151218
# version         : 0.1
# notes           :
# python_version  : 3
# dependencies    : matplotlib, numpy, scipy
# ==============================================================================

import PID
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import make_interp_spline
from IPython import embed

class GetPIDPlot():
    def __init__(self) -> None:
        self.line_printed = False

    def test_pid(self, P = 0.2, I = 0.0, D= 0.0, L=100):
        """Self-test PID class

        .. note::
            ...
            for i in range(1, END):
                pid.update(feedback)
                output = pid.output
                if pid.target > 0:
                    feedback += (output - (1/i))
                if i>9:
                    pid.target = 1
                time.sleep(0.02)
            ---
        """
        pid = PID.PID(P, I, D)
        pid.set_target(0.0)
        pid.set_sample_time(0.01)

        END = L
        feedback = 0

        feedback_list = []
        time_list = []
        target_list = []

        for i in range(1, END):
            output = pid.update(feedback)
            if pid.target > 0:
                feedback += (output - (1/i))
            if i > 9:
                pid.set_target(1)
            time.sleep(0.02)

            feedback_list.append(feedback)
            target_list.append(pid.target)
            time_list.append(i)

        time_sm = np.array(time_list)
        time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)
        helper_x3 = make_interp_spline(time_list, feedback_list)
        feedback_smooth = helper_x3(time_smooth)
        if not self.line_printed:
            plt.plot(time_list, target_list)
            self.line_printed = True
        return time_smooth, feedback_smooth, feedback_list


    def plot(self, p, i, d):
        L = 50
        time_smooth, feedback_smooth, feedback_list = self.test_pid(p, i, d, L=L)
        plt.plot(time_smooth, feedback_smooth)
        plt.xlim((0, L))
        plt.ylim((min(feedback_list)-0.5, max(feedback_list)+0.5))


if __name__ == "__main__":
    get_plot = GetPIDPlot()
    get_plot.plot(1.2, 1, 0.001)
    get_plot.plot(1.2, 1.5, 0.001)
    plt.xlabel('time (s)')
    plt.ylabel('PID (PV)')
    plt.title('TEST PID')
    plt.ylim((1-0.5, 1+0.5))
    plt.grid(True)
    plt.legend(['label1', 'label2', 'label3'])
    plt.show()
