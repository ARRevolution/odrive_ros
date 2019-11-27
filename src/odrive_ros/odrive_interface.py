import serial
from serial.serialutil import SerialException

import sys
import time
import logging
import traceback

import odrive
from odrive.enums import *
from odrive.utils import dump_errors

import fibre

default_logger = logging.getLogger(__name__)
default_logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

default_logger.addHandler(ch)

class ODriveFailure(Exception):
    pass

class ODriveInterfaceAPI(object):
    encoder_cpr_a0 = 0
    encoder_cpr_a1 = 0
    _prerolled = True #False
    #engaged = False
    
    def __init__(self, logger=None):
        self.id = None
        # self. engaged = False
        self.axis0 = None
        self.axis1 = None
        self.connected = False
        self._index_searched = False
        self.driver = None
        self.logger = logger if logger else default_logger
                
    def __del__(self):
        self.disconnect()
                    
    def connect(self, port=None, timeout=30, odrive_id=None):
        """
        Connect by serial numbers

        serial numbers:
            207C37823548

        params:
            port
            timeout
            odrive_id - string id unique to each odroid
        """
        self.logger.info("Find ID = 0x" + odrive_id)
        self.id = odrive_id
        if self.driver:
            self.logger.info("Already connected. Disconnecting and reconnecting.")
        try:
            #self.driver = odrive.find_any(timeout=timeout, logger=self.logger)
            self.driver = odrive.find_any(serial_number=odrive_id, timeout=timeout, logger=self.logger)
            self.axes = (self.driver.axis0, self.driver.axis1)
        except:
            self.logger.error("No ODrive found. Is device powered?")
            return False
            
        # save some parameters for easy access
        self.axis0 = self.driver.axis0
        self.axis1 = self.driver.axis1

        self.encoder_cpr_a0 = self.driver.axis0.encoder.config.cpr
        self.encoder_cpr_a0 = self.driver.axis1.encoder.config.cpr
        
        self.connected = True
        self.logger.info("Connected to ODrive. SN 0x%x, Hardware v%d.%d-%d, firmware v%d.%d.%d%s" % (
                        self.driver.serial_number,
                        self.driver.hw_version_major, self.driver.hw_version_minor, self.driver.hw_version_variant,
                        self.driver.fw_version_major, self.driver.fw_version_minor, self.driver.fw_version_revision,
                        "-dev" if self.driver.fw_version_unreleased else ""
                        ))
        return True
        
    def disconnect(self):
        self.connected = False
        self.axis0 = None
        self.axis1 = None
        
        self._prerolled = False
        #self.engaged = False
        
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        
        try:
            self.driver.release()
        except:
            self.logger.error("Error in timer: " + traceback.format_exc())
            return False
        finally:
            self.driver = None
        return True

    def calibrate(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        
        self.logger.info("Vbus %.2fV" % self.driver.vbus_voltage)
        
        for i, axis in enumerate(self.axes):
            self.logger.info("Calibrating axis %d..." % i)
            axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            time.sleep(1)
            while axis.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
            if axis.error != 0:
                self.logger.error("Failed calibration with axis error 0x%x, motor error 0x%x" % (axis.error, axis.motor.error))
                return False
                
        return True
        
    def preroll(self, wait=True):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
            
        if self._prerolled: # must be prerolling or already prerolled
            return False
            
        #self.logger.info("Vbus %.2fV" % self.driver.vbus_voltage)

        for i, axis in enumerate(self.axes):
            self.logger.info("Index search preroll axis %d..." % i)
            axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        
        if wait:
            for i, axis in enumerate(self.axes):
                while axis.current_state != AXIS_STATE_IDLE:
                    time.sleep(0.1)
            for i, axis in enumerate(self.axes):
                if axis.error != 0:
                    self.logger.error("Failed preroll with axis error 0x%x, motor error 0x%x" % (axis.error, axis.motor.error))
                    return False
        self._prerolled = True
        return True
        
    def prerolling(self):
        return self.axes[0].current_state == AXIS_STATE_ENCODER_INDEX_SEARCH or self.axes[1].current_state == AXIS_STATE_ENCODER_INDEX_SEARCH
    
    def prerolled(self): #
        return self._prerolled and not self.prerolling()
    
    def engaged(self):
        return self.axes[0].current_state == AXIS_STATE_CLOSED_LOOP_CONTROL or self.axes[1].current_state == AXIS_STATE_CLOSED_LOOP_CONTROL
    
    def idle(self):
        return self.axes[0].current_state == AXIS_STATE_IDLE and self.axes[1].current_state == AXIS_STATE_IDLE
        
    def engage(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False

        ### Original Differential robot
        #self.logger.debug("Setting drive mode.")
        #for axis in self.axes:
        #    axis.controller.vel_setpoint = 0
        #    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        #    axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        
        ### Steered Robot
        self.axes[0].controller.pos_setpoint = 0
        self.axes[0].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axes[0].controller.config.control_mode = CTRL_MODE_POSITION_CONTROL 

        self.axes[1].controller.vel_setpoint = 0
        self.axes[1].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axes[1].controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
		
        #self.engaged = True
        return True
        
    def release(self):
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        #self.logger.debug("Releasing.")
        for axis in self.axes: 
            axis.requested_state = AXIS_STATE_IDLE

        #self.engaged = False
        return True
    
    def drive_vel(self, ax0=None, ax1=None):
        if not self.driver:
            self.logger.error("Vel - Not connected.")
            return
        elif not self.engaged():
            self.logger.error("Vel - Not engaged")
            return
        try:
            if ax0 is not None:
                self.driver.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
                self.driver.axis0.controller.vel_setpoint = ax0
                self.logger.info("axis0 - Vel")      
            if ax1 is not None:
                self.driver.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
                self.driver.axis1.controller.vel_setpoint = ax1
                #self.logger.info("axis1 - Vel")
        except (fibre.protocol.ChannelBrokenException, AttributeError) as e:
           raise ODriveFailure(str(e))
        
    def drive_pos(self, ax0=None, ax1=None, trajectory=None):
        if not self.driver:
            self.logger.error("Pos - Not connected.")
            return
        elif not self.engaged():
            self.logger.error("Pos - Not engaged")
            return
        try:
            mode = CTRL_MODE_POSITION_CONTROL if trajectory is None else CTRL_MODE_TRAJECTORY_CONTROL
            if ax0 is not None:
                self.driver.axis0.controller.config.control_mode = mode
                if trajectory:
                    self.set_trajectory(self.driver.axis0.trap_traj.config, trajectory)
                    self.driver.axis0.controller.move_incremental(ax0, False)
                else:
                    #self.logger.info("Setting pos - %f" % ax0)
                    self.driver.axis0.controller.pos_setpoint = ax0

            if ax1 is not None:
                self.driver.axis1.controller.config.control_mode = mode
                if trajectory:
                    self.set_trajectory(self.driver.axis1.trap_traj.config, trajectory)
                    self.driver.axis1.controller.move_incremental(ax1, False)
                else:
                    self.driver.axis1.controller.pos_setpoint = ax1
        except (fibre.protocol.ChannelBrokenException, AttributeError) as e:
           raise ODriveFailure(str(e))

    def set_trajectory(self, traj_config, traj_values):
        """
        Trajectory control value have units related to counts
        """

        assert len(traj_values) == 4, "Trajectory values not 4 elements long"
        traj_config.vel_limit = traj_values[0]
        traj_config.accel_limit = traj_values[1]
        traj_config.decel_limit = traj_values[2]
        traj_config.A_per_css = traj_values[3]

    def drive_current(self, ax0=None, ax1=None):
        if not self.driver:
            self.logger.error("Not connected.")
            return
        elif not self.engaged():
            self.logger.error("Not engaged")
            return
        try:
            if ax0 is not None:
                self.driver.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
                self.driver.axis0.controller.current_setpoint += ax0
            if ax1 is not None:
                self.driver.axis1.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
                self.driver.axis1.controller.current_setpoint += ax1
        except (fibre.protocol.ChannelBrokenException, AttributeError) as e:
           raise ODriveFailure(str(e))

    def get_errors(self, clear=True):
        # TODO: add error parsing, see: https://github.com/madcowswe/ODrive/blob/master/tools/odrive/utils.py#L34
        if not self.driver:
            return None
            
        axis_error = self.axes[0].error or self.axes[1].error
        
        if clear:
            for axis in self.axes:
                axis.error = 0
                axis.motor.error = 0
                axis.encoder.error = 0
                axis.controller.error = 0
        
        if axis_error:
            return "error"
    
    def get_adc_pos(self):
        if not self.driver:
            return None
        
        return self.driver.get_adc_voltage(5)
        
        