# -*- coding: utf-8 -*-
# Mechanicaly conforms a moving gantry to the bed with 4 Z steppers
#
# Copyright (C) 2018  Maks Zolin <mzolin@vorondesign.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# 기존 버전 수정
# git rev-parse --verify HEAD
# 수정당시 revision - 72161d04053e168baf025957ec75008d0fd4f60f

import logging
import probe, z_tilt
import traceback
import json
from decimal import Decimal
import math
import os

def list_to_decimal(v):
    return [Decimal(str(p)) for p in v]


def list_plus(v1, v2):
    return [a + b for a, b in zip(v1, v2)]


def list_minus(v1, v2):
    return [a - b for a, b in zip(v1, v2)]


def list_cross_product(v1, v2):
    return [v1[1] * v2[2] - v1[2] * v2[1],
            v1[2] * v2[0] - v1[0] * v2[2],
            v1[0] * v2[1] - v1[1] * v2[0]]


def get_estimated_z_value(normal_v, base_point_v, xy_for_estimate):
    x = xy_for_estimate[0]
    y = xy_for_estimate[1]
    return base_point_v[2] - ((((x - base_point_v[0]) * normal_v[0]) + ((y - base_point_v[1]) * normal_v[1])) / normal_v[2])


class QuadGantryLevelMyMod:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.retry_helper = z_tilt.RetryHelper(config,
            "Possibly Z motor numbering is wrong")
        self.max_adjust = config.getfloat("max_adjust", 4, above=0)
        self.horizontal_move_z = config.getfloat("horizontal_move_z", 5.0)
        self.probe_helper = probe.ProbePointsHelper(config, self.probe_finalize)
        if len(self.probe_helper.probe_points) != 4:
            raise config.error(
                "Need exactly 4 probe points for quad_gantry_level")
        self.load_default_adjust()
        gantry_corners = config.get('gantry_corners').split('\n')
        try:
            gantry_corners = [line.split(',', 1)
                              for line in gantry_corners if line.strip()]
            self.gantry_corners = [(Decimal(zp[0].strip()), Decimal(zp[1].strip()))
                                   for zp in gantry_corners]
        except:
            raise config.error("Unable to parse gantry_corners in %s" % (
                config.get_name()))
        if len(self.gantry_corners) < 2:
            raise config.error(
                "quad_gantry_level requires at least two gantry_corners")
        # Register QUAD_GANTRY_LEVEL command
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'QUAD_GANTRY_LEVEL', self.cmd_QUAD_GANTRY_LEVEL,
            desc=self.cmd_QUAD_GANTRY_LEVEL_help)
        self.gcode.register_command(
            'QUAD_GANTRY_MANUAL', self.cmd_QUAD_GANTRY_MANUAL,
            desc=self.cmd_QUAD_GANTRY_MANUAL_help)
        self.gcode.register_command(
            'QUAD_GANTRY_ADJUST', self.cmd_QUAD_GANTRY_ADJUST,
            desc=self.cmd_QUAD_GANTRY_ADJUST_help)

    cmd_QUAD_GANTRY_LEVEL_help = (
        "Conform a moving, twistable gantry to the shape of a stationary bed")
    cmd_QUAD_GANTRY_MANUAL_help = (
        "TEMP : manual adjustment")
    cmd_QUAD_GANTRY_ADJUST_help = (
        "TEMP : default adjustment")

    def cmd_QUAD_GANTRY_LEVEL(self, params):
        self.retry_helper.start(params)
        self.probe_helper.start_probe(params)

    def cmd_QUAD_GANTRY_MANUAL(self, params):
        try:
            z_adjusts = []
            self.gcode.respond_info("QUAD_GANTRY_MANUAL----------------")
            self.gcode.respond_info(json.dumps(params))
            for idx in range(4):
                k = 'Z' + str(idx)
                val = 0
                if k in params:
                    val = Decimal(params[k])
                z_adjusts.append(val)

            adjust_max = max([abs(v) for v in z_adjusts])
            if adjust_max > self.max_adjust:
                self.gcode.respond_error(
                    "Aborting quad_gantry_level " +
                    "required adjustment %0.6f " % (adjust_max) +
                    "is greater than max_adjust %0.6f" % (self.max_adjust))
                return
            self.adjust_z_steppers(z_adjusts)
        except:
            self.gcode.respond_info(traceback.format_exc())
    
    def cmd_QUAD_GANTRY_ADJUST(self, params):
        try:
            z_adjusts = []
            self.gcode.respond_info("QUAD_GANTRY_ADJUST----------------")
            self.gcode.respond_info(json.dumps(params))
            for idx in range(4):
                k = 'Z' + str(idx)
                val = 0
                if k in params:
                    val = Decimal(params[k])
                z_adjusts.append(val)

            adjust_max = max([abs(v) for v in z_adjusts])
            if adjust_max > self.max_adjust:
                self.gcode.respond_error(
                    "Aborting quad_gantry_level " +
                    "required adjustment %0.6f " % (adjust_max) +
                    "is greater than max_adjust %0.6f" % (self.max_adjust))
                return
            self.default_adjust = z_adjusts
            self.save_default_adjust()
        except:
            self.gcode.respond_info(traceback.format_exc())
    
    
    default_adjust_file_name = '/home/pi/quad_gantry_default_adjustment.json'
    def load_default_adjust(self):
        self.default_adjust = [Decimal(0),Decimal(0),Decimal(0),Decimal(0)]
        if os.path.exists(self.default_adjust_file_name):
            with open(self.default_adjust_file_name, 'r') as f:
                for idx, v in enumerate(f.readlines()):
                    self.default_adjust[idx] = Decimal(v)   
 
    def save_default_adjust(self):
        with open(self.default_adjust_file_name, 'w') as f:
            for val in self.default_adjust:
                f.write(str(val))
                f.write('\n')

    def probe_finalize(self, offsets, positions):
        # 빌드플레이트와 gantry는 평행해야하기때문에 굳이 뒤집어서 계산하지말자. 심플하게.
        # offsets -> 프로브와 노즐의 offset. 내꺼는 0, 25, 0
        # positions -> 4개의 프로브 결과값.

        # 각 4개의 포인트에서 normal vector를 구한다음에 한번 비교해보자.
        # 그리고, 오차를 최소화하기위해 모든 계산은 Decimal로 하며 결과를 float으로 바꿈.

        # x,y,z의 표현으로 포인트들을 만들자.
        probe_points = []
        for z_idx in range(4):
            ps = list_to_decimal(positions[z_idx])
            os = list_to_decimal(offsets)
            probe_points.append(list_plus(ps, os))

        points_message = "probe points:\n%s\n" % (
            " ".join(["%s: (%s)" % (z_id, ", ".join(['{:.6f}'.format(val) for val in probe_points[z_id]]))
                      for z_id in range(len(probe_points))]))
        self.gcode.respond_info(points_message)

        # normal vector는 베드로부터 윗쪽 방향으로. 오른손의 법칙 적용.
        # 0 : (v3 - v0) x (v1 - v0)
        # 1 : (v0 - v1) x (v2 - v1)
        # 2 : (v1 - v2) x (v3 - v2)
        # 3 : (v2 - v3) x (v0 - v3)
        normal_vectors = []
        for idx in range(4):
            cur_point = probe_points[idx]
            prev_point = probe_points[idx - 1]
            next_point = probe_points[(idx + 1) % 4]
            f1 = list_minus(prev_point, cur_point)
            f2 = list_minus(next_point, cur_point)
            normal_vectors.append(list_cross_product(f1, f2))

        points_message = "normal_vectors of each point:\n%s\n" % (
            " ".join(["%s: (%s)" % (z_id, ", ".join(['{:.6f}'.format(val) for val in normal_vectors[z_id]]))
                      for z_id in range(len(normal_vectors))]))
        self.gcode.respond_info(points_message)

        # 벨트의 위치
        belt_min_x = self.gantry_corners[0][0]
        belt_max_x = self.gantry_corners[1][0]
        belt_min_y = self.gantry_corners[0][1]
        belt_max_y = self.gantry_corners[1][1]
        belt_points = [[belt_min_x, belt_min_y],
                       [belt_min_x, belt_max_y],
                       [belt_max_x, belt_max_y],
                       [belt_max_x, belt_min_y]]
        z_corner_heights = [get_estimated_z_value(normal_vectors[idx], probe_points[idx], bt)
                            for idx, bt in enumerate(belt_points)]
        points_message = "corner_heights of each point:\n%s\n" % (
            " ".join(["%s: %.6f" % (z_id, z_corner_heights[z_id])
                      for z_id in range(len(z_corner_heights))]))
        self.gcode.respond_info(points_message)
		
        r_z_adjust = list_plus(z_corner_heights, self.default_adjust) 
        points_message = "r_z_adjust:\n%s\n" % (
            " ".join(["%s: %.6f" % (z_id, r_z_adjust[z_id])
                      for z_id in range(len(r_z_adjust))]))
        self.gcode.respond_info(points_message)

        # 평균 값을 중앙값으로하여 변환값을 정한다.
        z_ave = sum(r_z_adjust) / len(r_z_adjust)
        # 더 높다는 것은 더 적게 내려왔다는 뜻이므로 올려줘야한다.
        z_adjust = [z - z_ave for z in r_z_adjust]

        points_message = "z_adjust:\n%s\n" % (
            " ".join(["%s: %.6f" % (z_id, z_adjust[z_id])
                      for z_id in range(len(z_adjust))]))
        self.gcode.respond_info(points_message)

        adjust_max = max([abs(v) for v in z_adjust])
        if adjust_max > self.max_adjust:
            self.gcode.respond_error(
                "Aborting quad_gantry_level " +
                "required adjustment %0.6f " % (adjust_max) +
                "is greater than max_adjust %0.6f" % (self.max_adjust))
            return

        self.adjust_z_steppers(z_adjust)
        return self.retry_helper.check_retry([p[2] for p in probe_points])

    def adjust_z_steppers(self, adjust_heights):
        tool_head = self.printer.lookup_object('toolhead')
        kin = tool_head.get_kinematics()
        z_steppers = kin.get_steppers('Z')
        current_position = tool_head.get_position()
        # Report on movements
        step_strs = ["%s = %.6f" % (s.get_name(), float(a))
                     for s, a in zip(z_steppers, adjust_heights)]
        msg = "Making the following Z adjustments:\n%s" % ("\n".join(step_strs))
        self.gcode.respond_info(msg)

        # Move each z stepper (sorted from lowest to highest) until they match
        positions = [(float(a), s) for a, s in zip(adjust_heights, z_steppers)]
        speed = self.probe_helper.get_lift_speed()

        try:
            for v in positions:
                tool_head.flush_step_generation()
                for s in z_steppers:
                    s.set_trapq(None)
                stepper_offset, stepper = v
                stepper.set_trapq(tool_head.get_trapq())
                new_pos = current_position
                new_pos[2] = new_pos[2] + stepper_offset
                tool_head.move(new_pos, speed)
                tool_head.set_position(current_position)
        except Exception as e:
            self.gcode.respond_info(str(e))
            self.gcode.respond_info(traceback.format_exc())
            logging.exception("ZAdjustHelper adjust_steppers")
            raise
        finally:
            tool_head.flush_step_generation()
            for s in z_steppers:
                s.set_trapq(tool_head.get_trapq())
            tool_head.set_position(current_position)
            self.gcode.reset_last_position()
'''
    def probe_finalize(self, offsets, positions):
        # Mirror our perspective so the adjustments make sense
        # from the perspective of the gantry
        z_positions = [self.horizontal_move_z - p[2] for p in positions]
        points_message = "Gantry-relative probe points:\n%s\n" % (
            " ".join(["%s: %.6f" % (z_id, z_positions[z_id])
                for z_id in range(len(z_positions))]))
        self.gcode.respond_info(points_message)
        # x의 위치와 그 위치에서의 z값, z값은 위에서 말한것처럼
        # gantry가 lift 되어서 내려올때까지의 거리 그공간을 뒤집어서 위로 올라간다생각하면됨.
        p1 = [positions[0][0] + offsets[0],z_positions[0]]
        p2 = [positions[1][0] + offsets[0],z_positions[1]]
        p3 = [positions[2][0] + offsets[0],z_positions[2]]
        p4 = [positions[3][0] + offsets[0],z_positions[3]]
        # 좌우로 라인을 그어서 직선의 방정식을 만들어 mx+b=y의 모양을 만든다. m값과 b값을 구함.
        f1 = self.linefit(p1,p4)
        f2 = self.linefit(p2,p3)
        logging.info("quad_gantry_level f1: %s, f2: %s" % (f1,f2))
        # 이제 세로의 기울기 구하기 앞에가 1 뒤에가 2.
        # y값과 그 위치에서의 z값
        a1 = [positions[0][1] + offsets[1],
              self.plot(f1,self.gantry_corners[0][0])]
        a2 = [positions[1][1] + offsets[1],
              self.plot(f2,self.gantry_corners[0][0])]
        b1 = [positions[0][1] + offsets[1],
              self.plot(f1,self.gantry_corners[1][0])]
        b2 = [positions[1][1] + offsets[1],
              self.plot(f2,self.gantry_corners[1][0])]
        af = self.linefit(a1,a2)
        bf = self.linefit(b1,b2)
        logging.info("quad_gantry_level af: %s, bf: %s" % (af,bf))
        z_height = [0,0,0,0]
        z_height[0] = self.plot(af,self.gantry_corners[0][1])
        z_height[1] = self.plot(af,self.gantry_corners[1][1])
        z_height[2] = self.plot(bf,self.gantry_corners[1][1])
        z_height[3] = self.plot(bf,self.gantry_corners[0][1])

        ainfo = zip(["z","z1","z2","z3"], z_height[0:4])
        apos = " ".join(["%s: %06f" % (x) for x in ainfo])
        self.gcode.respond_info("Actuator Positions:\n" + apos)

        z_ave = sum(z_height) / len(z_height)
        self.gcode.respond_info("Average: %0.6f" % z_ave)
        z_adjust = []
        for z in z_height:
            z_adjust.append(z_ave - z)

        adjust_max = max(z_adjust)
        if adjust_max > self.max_adjust:
            self.gcode.respond_error(
                "Aborting quad_gantry_level " +
                "required adjustment %0.6f " % ( adjust_max ) +
                "is greater than max_adjust %0.6f" % (self.max_adjust))
            return

        speed = self.probe_helper.get_lift_speed()
        self.z_helper.adjust_steppers(z_adjust, speed)
        return self.retry_helper.check_retry(z_positions)
    def linefit(self,p1,p2):
        if p1[1] == p2[1]:
            # Straight line
            return 0,p1[1]
        m = (p2[1] - p1[1])/(p2[0] - p1[0])
        b = p1[1] - m * p1[0]
        return m,b
    def plot(self,f,x):
        return f[0]*x + f[1]
'''


def load_config(config):
    return QuadGantryLevelMyMod(config)
