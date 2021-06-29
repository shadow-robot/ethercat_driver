DESC =  '''
=============== Live Sensor Feed Viewer  =============== \n
Visualizes the data and the 3D rendering of the Adroit Model\n
DEPENDENCIES: \n
    1) pyQtGraph : conda install pyqtgraph \n
    2) mj_viz : follow the instructions here https://github.com/vikashplus/mj_viz \n
USAGE: \n
    1) Turn on the hardware \n
    2) python3 hand_sensor_viewer.py (--help)\n
# ============================================================n
'''

from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import sys
import hand_interface
from pathlib import Path
import click


# Handware constants -----------------------------------
N_RAW = 37          # length of raw sensor data
N_CALIB = 24        # length of calibrated sensor data
N_CMD = 20          # length of motor commands
N_TOUCH = 24        # length of touch data
N_ACCGYRO = 10      # length of accelerometer+gyroscope data
MJ_NQ = 24          # length of qpos in mujoco (required for mj_viz)
MJ_NV = 24          # length of qvel in mujoco (required for mj_viz)


# USER INPUTS ------------------------------------------
USE_MJ_VIZ = True           # enbale/ disable mujoco viz
USER_COMMANDS = [0]*N_CMD   # command data
USE_POSITION = False        # actuation mode
UPDATE_RATE = 10            # data update rate in Hz


if USE_MJ_VIZ:
    import mj_viz as viewer


# Bar class -----------------------------------
class Bar:
    def __init__(self, axis, yData, xData=None,
                 xRange=None, yRange=None,
                 title='', xLabel='', yLabel='',
                 width=0.8, brush='b'):

        if xData is None:
            n_ydata = len(yData)
            xData=np.linspace(0, n_ydata, n_ydata)
        self.bars = pg.BarGraphItem(x=xData, height=yData, width=width, brush=brush)
        axis.addItem(self.bars)
        axis.setLabel('bottom', xLabel, units='')
        axis.setLabel('left', yLabel, units='')
        if xRange is not None:
            axis.setXRange(xRange[0], xRange[1])
        if yRange is not None:
            axis.setYRange(yRange[0], yRange[1])

    def update(self, yData):
        self.bars.setOpts(height=yData)


# Config sensory plots ------------------------------------------
def config_sensorviewer(win):
    global raw_bars, calib_bars, cmd_bars, IFT_bars, MFT_bars, RFT_bars, LFT_bars, TFT_bars, ACCGYRO_bars

    # Raw joint values
    raw_plt = win.addPlot(title="raw", colspan=2)
    raw_bars = Bar(raw_plt, yData=np.zeros(N_RAW), xLabel='sensor id', yLabel='sensor value', yRange=(0, 4000), brush='b')

    # Calib joint values
    calib_plt = win.addPlot(title="calib", colspan=2)
    calib_bars = Bar(calib_plt, yData=np.zeros(N_CALIB), xLabel='sensor id', yLabel='sensor value', yRange=(-3.14, 3.14), brush='g')

    # Commands
    cmd_plt = win.addPlot(title="commands", colspan=2)
    cmd_bars = Bar(cmd_plt, yData=np.zeros(N_CMD), xLabel='actuator id', yLabel='cmd value', yRange=(-3.14, 3.14), brush='r')

    win.nextRow()

    # Index touch
    IFT_plt = win.addPlot(title="Touch FF", colspan=1)
    IFT_bars = Bar(IFT_plt, yData=np.zeros(N_TOUCH), xLabel='sessor id', yLabel='sessor value',\
                 yRange=(0, 4000), brush=pg.mkColor(200, 200, 210))

    # Middle touch
    MFT_plt = win.addPlot(title="Touch MF", colspan=1)
    MFT_bars = Bar(MFT_plt, yData=np.zeros(N_TOUCH), xLabel='sessor id', yLabel='sessor value',\
                 yRange=(0, 4000), brush=pg.mkColor(200, 200, 220))

    # Ring touch
    RFT_plt = win.addPlot(title="Touch RF", colspan=1)
    RFT_bars = Bar(RFT_plt, yData=np.zeros(N_TOUCH), xLabel='sessor id', yLabel='sessor value',\
                 yRange=(0, 4000), brush=pg.mkColor(200, 200, 230))
    # Little touch
    LFT_plt = win.addPlot(title="Touch LF", colspan=1)
    LFT_bars = Bar(LFT_plt, yData=np.zeros(N_TOUCH), xLabel='sessor id', yLabel='sessor value',\
                 yRange=(0, 4000), brush=pg.mkColor(200, 200, 240))

    # Thumb touch
    TFT_plt = win.addPlot(title="Touch TF", colspan=1)
    TFT_bars = Bar(TFT_plt, yData=np.zeros(N_TOUCH), xLabel='sessor id', yLabel='sessor value',\
                 yRange=(0, 4000), brush=pg.mkColor(200, 200, 250))

    # Accelerometer and Gyroscopes
    ACCGYRO_bars_plt = win.addPlot(title="Acc/ Gyro", colspan=1)
    ACCGYRO_bars = Bar(ACCGYRO_bars_plt, yData=np.zeros(N_ACCGYRO), xLabel='sessor id', yLabel='sessor value',\
                 yRange=(-3.14, 3.14), brush=pg.mkColor(200, 200, 220))


# Fake Sensor update ------------------------------------------
def random_update():
    global raw_bars, calib_bars, cmd_bars, IFT_bars, MFT_bars, RFT_bars, LFT_bars, TFT_bars, ACCGYRO_bars

    #Raw plot
    raw_data = np.random.uniform(low=0, high=4000, size=N_RAW)
    raw_bars.update(raw_data)

    #Calib plot
    calib_data = np.random.normal(size=N_CALIB)
    calib_bars.update(calib_data)

    #cmd plot
    cmd_data = np.random.normal(size=N_CMD)
    cmd_bars.update(cmd_data)

    #IFT plot
    IFT_data = np.random.normal(size=N_CALIB)
    IFT_bars.update(IFT_data)

    #MFT plot
    MFT_data = np.random.normal(size=N_CALIB)
    MFT_bars.update(MFT_data)

    #RFT plot
    RFT_data = np.random.normal(size=N_CALIB)
    RFT_bars.update(RFT_data)

    #LFT plot
    LFT_data = np.random.normal(size=N_CALIB)
    LFT_bars.update(LFT_data)

    #TFT plot
    TFT_data = np.random.normal(size=N_CALIB)
    TFT_bars.update(TFT_data)

    #TFT plot
    ACCGYRO_data = np.random.normal(size=N_ACCGYRO)
    ACCGYRO_bars.update(ACCGYRO_data)

    if USE_MJ_VIZ:
        qp_data = 0.1*np.random.normal(size=MJ_NQ)
        qv_data = 0*np.random.normal(size=MJ_NV)
        viewer.viz_update(float(0), qp_data, qv_data)


# Hardware Sensor update ------------------------------------------
def sensor_update():
    global robot, raw_bars, calib_bars, cmd_bars, IFT_bars, MFT_bars, RFT_bars, LFT_bars, TFT_bars, ACCGYRO_bars

    state = robot.run_loop(USER_COMMANDS, use_position=USE_POSITION, duration=1./UPDATE_RATE)
    # robot.print_state(state)

    #Raw plot
    raw_bars.update(yData=state.raw_position[:])

    #Calib plot
    qp_data = robot.get_mujoco_state(state)
    calib_bars.update(yData=qp_data)

    #cmd plot
    cmd_bars.update(yData=[hand_interface.to_radians(j) for j in USER_COMMANDS])

    #IFT plot
    IFT_bars.update(yData=state.biotac_data[0].electrodes[:])

    #MFT plot
    MFT_bars.update(yData=state.biotac_data[1].electrodes[:])

    #RFT plot
    RFT_bars.update(yData=state.biotac_data[2].electrodes[:])

    #LFT plot
    LFT_bars.update(yData=state.biotac_data[3].electrodes[:])

    #TFT plot
    TFT_bars.update(yData=state.biotac_data[4].electrodes[:])

    # 3D viewer
    if USE_MJ_VIZ:
        qv_data = 0*np.random.normal(size=MJ_NV)
        viewer.viz_update(float(0), qp_data, qv_data)


# MAIN =========================================================
@click.command(help=DESC)
@click.option('--user_home_dir', '-u', type=str, help='user\'s home directory path', default="//usr//local//google//home//vikashplus")
@click.option('--hardware', '-h', type=bool, help='use hardware?', default= True)
def main(user_home_dir, hardware):

    # Set up. ----------------------
    app = QtGui.QApplication([])
    win = pg.GraphicsWindow(title="Adroit data")
    win.resize(1000,600)
    win.setWindowTitle('Adroit')
    pg.setConfigOptions(antialias=True) # Enable antialiasing for prettier plots

    # open visualizer
    if USE_MJ_VIZ:
        viewer.viz_init(user_home_dir+"//dexterous//adept_models//adroit//Adroit_hand.xml",\
        viewer.viz_init(user_home_dir+"//Libraries//adept//adept_models//adroit//Adroit_hardware.xml",\
        user_home_dir+"//.mujoco//mjkey.txt")

    # config plots ----------------------
    config_sensorviewer(win)

    # set up hand
    if(hardware):
        global robot
        robot = hand_interface.ShadowHandRobot()
        robot.hand_init()

    # sensor update ----------------------
    timer = QtCore.QTimer()
    if(hardware):
        timer.timeout.connect(sensor_update)
    else:
        timer.timeout.connect(random_update)
    timer.start(1./30) # 30 Hz

    # start viewer ----------------------
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

    if USE_MJ_VIZ:
        viewer.viz_close()


if __name__ == '__main__':
    main()




