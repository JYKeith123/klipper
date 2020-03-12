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
import probe, z_tilt, bed_mesh
import traceback
import json
from decimal import Decimal
import math
import os
import collections


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
        self.printer.register_event_handler("klippy:ready",
                                            self.handle_ready)
        self.retry_helper = z_tilt.RetryHelper(config,
            "Possibly Z motor numbering is wrong")
        self.max_adjust = config.getfloat("max_adjust", 4, above=0)
        self.horizontal_move_z = config.getfloat("horizontal_move_z", 5.0)
        self.probe_helper = probe.ProbePointsHelper(config, self.probe_finalize)
        if len(self.probe_helper.probe_points) != 4:
            raise config.error(
                "Need exactly 4 probe points for quad_gantry_level")
        self.default_adjust = [Decimal(0), Decimal(0), Decimal(0), Decimal(0)]
        self.z_mesh_matrix = None
        self.last_position = [0., 0., 0., 0.]
        self.load_custom_data()
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
        self.gcode.register_command(
            'QUAD_BED_MESH_ADJUST', self.cmd_QUAD_BED_MESH_ADJUST,
            desc=self.cmd_cmd_QUAD_BED_MESH_ADJUST_help)
        self.gcode.register_command(
            'QUAD_BED_MESH_CLEAR', self.cmd_QUAD_BED_MESH_CLEAR,
            desc=self.cmd_QUAD_BED_MESH_CLEAR_help)

        self.tool_head = None
        self.z_mesh = None
        self.use_additional_z_mesh = config.getboolean("use_additional_z_mesh", False)
        if self.use_additional_z_mesh:
            self.mesh_params = collections.OrderedDict()
            self.mesh_params['algo'] = 'direct'
            self.points = self._generate_points(config)
            self.fade_start = config.getfloat('fade_start', 1.)
            self.fade_end = config.getfloat('fade_end', 0.)
            self.fade_dist = self.fade_end - self.fade_start
            if self.fade_dist <= 0.:
                self.fade_start = self.fade_end = self.FADE_DISABLE

            self.log_fade_complete = False
            self.base_fade_target = config.getfloat('fade_target', None)
            self.fade_target = 0.
            self.splitter = bed_mesh.MoveSplitter(config, self.gcode)
            self.gcode.set_move_transform(self)

    def handle_ready(self):
        self.tool_head = self.printer.lookup_object('toolhead')
        if self.use_additional_z_mesh:
            self.apply_mesh()

    def _generate_points(self, config):
        # rectangular
        x_cnt, y_cnt = bed_mesh.parse_pair(
            config, ('probe_count', '3'), check=False, cast=int, minval=3)
        min_x, min_y = bed_mesh.parse_pair(config, ('mesh_min',))
        max_x, max_y = bed_mesh.parse_pair(config, ('mesh_max',))
        #pps = bed_mesh.parse_pair(config, ('mesh_pps', '2'), check=False, cast=int, minval=0)
        pps = [0, 0]
        if max_x <= min_x or max_y <= min_y:
            raise config.error('bed_mesh: invalid min/max points')

        self.mesh_params['x_count'] = x_cnt
        self.mesh_params['y_count'] = y_cnt
        self.mesh_params['min_x'] = min_x
        self.mesh_params['max_x'] = max_x
        self.mesh_params['min_y'] = min_y
        self.mesh_params['max_y'] = max_y
        self.mesh_params['mesh_x_pps'] = pps[0]
        self.mesh_params['mesh_y_pps'] = pps[1]

        x_dist = (max_x - min_x) / (x_cnt - 1)
        y_dist = (max_y - min_y) / (y_cnt - 1)
        # floor distances down to next hundredth
        x_dist = math.floor(x_dist * 100) / 100
        y_dist = math.floor(y_dist * 100) / 100
        if x_dist <= 1. or y_dist <= 1.:
            raise config.error("bed_mesh: min/max points too close together")

        # rectangular bed, only re-calc max_x
        max_x = min_x + x_dist * (x_cnt - 1)
        pos_y = min_y
        points = []
        for i in range(y_cnt):
            for j in range(x_cnt):
                if not i % 2:
                    # move in positive directon
                    pos_x = min_x + j * x_dist
                else:
                    # move in negative direction
                    pos_x = max_x - j * x_dist
                points.append((pos_x, pos_y))
            pos_y += y_dist
        return points

    cmd_QUAD_GANTRY_LEVEL_help = (
        "Conform a moving, twistable gantry to the shape of a stationary bed")
    cmd_QUAD_GANTRY_MANUAL_help = (
        "TEMP : manual adjustment")
    cmd_QUAD_GANTRY_ADJUST_help = (
        "TEMP : default adjustment")
    cmd_cmd_QUAD_BED_MESH_ADJUST_help = (
        "cmd_cmd_QUAD_BED_MESH_ADJUST_help")
    cmd_QUAD_BED_MESH_CLEAR_help = (
        "cmd_QUAD_BED_MESH_CLEAR_help")

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
            self.save_custom_data()
        except:
            self.gcode.respond_info(traceback.format_exc())

    def cmd_QUAD_BED_MESH_ADJUST(self, params):
        try:
            # mesh_pps 는 0으로 강제했기에 x와 y값만 곱함
            x_cnt = self.mesh_params['x_count']
            y_cnt = self.mesh_params['y_count']
            total_point_count = x_cnt * y_cnt
            z_matrix = []

            self.gcode.respond_info("QUAD_BED_MESH_ADJUST----------------")
            self.gcode.respond_info(json.dumps(params))

            row = []
            for idx in range(total_point_count):
                # 좌측 하단부터 오른쪽으로 순서대로.
                # 원래 BedMesh에서는 ㄹ처럼 지그재그로 프로빙을한다.
                # 나중에 finalize 해서 받은 Z 값에 대해서는 X가 작은데서 큰데로 올라가는 순서로
                # 좌측하단부터 위로 올라가는 순서로 파라미터를 넘긴다.
                # 나는 좌측 상단부터 내려가는것을 원하기때문에 뒤집어준다.
                k = 'Z' + str(idx)
                val = 0
                if k in params:
                    val = float(params[k])
                row.append(val)

                if len(row) == x_cnt:
                    z_matrix.insert(0, row)
                    row = []
            self.z_mesh_matrix = z_matrix
            self.apply_mesh()
            self.save_custom_data()
        except:
            self.gcode.respond_info(traceback.format_exc())

    def apply_mesh(self):
        if self.z_mesh_matrix is not None:
            mesh = bed_mesh.ZMesh(self.mesh_params)
            try:
                mesh.build_mesh(self.z_mesh_matrix)
            except bed_mesh.BedMeshError as e:
                raise self.gcode.error(e.message)
            self.set_mesh(mesh)

    def cmd_QUAD_BED_MESH_CLEAR(self, params):
        self.set_mesh(None)

    custom_data_file_name = '/home/pi/quad_gantry_custom_data.json'

    def load_custom_data(self):
        if os.path.exists(self.custom_data_file_name):
            with open(self.custom_data_file_name, 'r') as f:
                json_data = f.read()
            scheme = json.loads(json_data)
            self.default_adjust = [Decimal(v) for v in scheme['default_adjust']]
            self.z_mesh_matrix = scheme['z_mesh_matrix']

    def save_custom_data(self):
        scheme = dict()
        scheme['default_adjust'] = [str(v) for v in self.default_adjust]
        scheme['z_mesh_matrix'] = self.z_mesh_matrix
        json_string = json.dumps(scheme, indent=4, sort_keys=False)
        with open(self.custom_data_file_name, 'w') as f:
            f.write(json_string)

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

        try:
            adjust_max = max([abs(v) for v in z_adjust])
            if adjust_max > self.max_adjust:
                self.gcode.respond_error(
                    "Aborting quad_gantry_level " +
                    "required adjustment %0.6f " % (adjust_max) +
                    "is greater than max_adjust %0.6f" % (self.max_adjust))
                return

            self.adjust_z_steppers(z_adjust)
            return self.retry_helper.check_retry([p[2] for p in probe_points])
        except Exception as e:
            self.gcode.respond_info(str(e))
            self.gcode.respond_info(traceback.format_exc())
            raise e

    def adjust_z_steppers(self, adjust_heights):
        kin = self.tool_head.get_kinematics()
        z_steppers = [s for s in kin.get_steppers() if s.is_active_axis('z')]
        current_position = self.tool_head.get_position()
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
                self.tool_head.flush_step_generation()
                for s in z_steppers:
                    s.set_trapq(None)
                stepper_offset, stepper = v
                stepper.set_trapq(self.tool_head.get_trapq())
                new_pos = current_position
                new_pos[2] = new_pos[2] + stepper_offset
                self.tool_head.move(new_pos, speed)
                self.tool_head.set_position(current_position)
        except Exception as e:
            self.gcode.respond_info(str(e))
            self.gcode.respond_info(traceback.format_exc())
            logging.exception("ZAdjustHelper adjust_steppers")
            raise
        finally:
            self.tool_head.flush_step_generation()
            for s in z_steppers:
                s.set_trapq(self.tool_head.get_trapq())
            self.tool_head.set_position(current_position)
            self.gcode.reset_last_position()

    FADE_DISABLE = 0x7FFFFFFF

    def set_mesh(self, mesh):
        if mesh is not None and self.fade_end != self.FADE_DISABLE:
            self.log_fade_complete = True
            if self.base_fade_target is None:
                self.fade_target = mesh.avg_z
            else:
                self.fade_target = self.base_fade_target
                min_z, max_z = mesh.get_z_range()
                if (not min_z <= self.fade_target <= max_z and
                        self.fade_target != 0.):
                    # fade target is non-zero, out of mesh range
                    err_target = self.fade_target
                    self.z_mesh = None
                    self.fade_target = 0.
                    raise self.gcode.error(
                        "bed_mesh: ERROR, fade_target lies outside of mesh z "
                        "range\nmin: %.4f, max: %.4f, fade_target: %.4f"
                        % (min_z, max_z, err_target))
            if self.fade_target:
                mesh.offset_mesh(self.fade_target)
            min_z, max_z = mesh.get_z_range()
            if self.fade_dist <= max(abs(min_z), abs(max_z)):
                self.z_mesh = None
                self.fade_target = 0.
                raise self.gcode.error(
                    "bed_mesh:  Mesh extends outside of the fade range, "
                    "please see the fade_start and fade_end options in"
                    "example-extras.cfg. fade distance: %.2f mesh min: %.4f"
                    "mesh max: %.4f" % (self.fade_dist, min_z, max_z))
        else:
            self.fade_target = 0.
        self.z_mesh = mesh
        self.splitter.initialize(mesh)
        # cache the current position before a transform takes place
        self.gcode.reset_last_position()

    def get_z_factor(self, z_pos):
        if z_pos >= self.fade_end:
            return 0.
        elif z_pos >= self.fade_start:
            return (self.fade_end - z_pos) / self.fade_dist
        else:
            return 1.

    def get_position(self):
        # Return last, non-transformed position
        if self.z_mesh is None:
            # No mesh calibrated, so send toolhead position
            self.last_position[:] = self.tool_head.get_position()
            self.last_position[2] -= self.fade_target
        else:
            # return current position minus the current z-adjustment
            x, y, z, e = self.tool_head.get_position()
            z_adj = self.z_mesh.calc_z(x, y)
            factor = 1.
            max_adj = z_adj + self.fade_target
            if min(z, (z - max_adj)) >= self.fade_end:
                # Fade out is complete, no factor
                factor = 0.
            elif max(z, (z - max_adj)) >= self.fade_start:
                # Likely in the process of fading out adjustment.
                # Because we don't yet know the gcode z position, use
                # algebra to calculate the factor from the toolhead pos
                factor = ((self.fade_end + self.fade_target - z) /
                          (self.fade_dist - z_adj))
                factor = bed_mesh.constrain(factor, 0., 1.)
            final_z_adj = factor * z_adj + self.fade_target
            self.last_position[:] = [x, y, z - final_z_adj, e]
        return list(self.last_position)

    def move(self, newpos, speed):
        factor = self.get_z_factor(newpos[2])
        if self.z_mesh is None or not factor:
            # No mesh calibrated, or mesh leveling phased out.
            x, y, z, e = newpos
            if self.log_fade_complete:
                self.log_fade_complete = False
                logging.info(
                    "bed_mesh fade complete: Current Z: %.4f fade_target: %.4f "
                    % (z, self.fade_target))
            self.tool_head.move([x, y, z + self.fade_target, e], speed)
        else:
            self.splitter.build_move(self.last_position, newpos, factor)
            while not self.splitter.traverse_complete:
                split_move = self.splitter.split()
                if split_move:
                    self.tool_head.move(split_move, speed)
                else:
                    raise self.gcode.error(
                        "Mesh Leveling: Error splitting move ")
        self.last_position[:] = newpos


def load_config(config):
    return QuadGantryLevelMyMod(config)
