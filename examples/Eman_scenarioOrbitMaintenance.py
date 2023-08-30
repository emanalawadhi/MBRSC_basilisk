#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

r"""
Overview
--------

Demonstrates how to execute orbit maintenance using thrusters. 
The method used is a periodic burn of the same duration and DV per orbit. The burn occurs
at 240 degrees. All these parameters can be edited according to the user.
Relevant scripts:
- simIncludeThrusters @ Utilities
- thrForceMapping.py @fswAlgorithms
- thrFiringRemainder.py @fswAlgorithms
- ?

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioOrbitMaintenance.py


Illustration of Simulation Results
----------------------------------

::

    show_plots = True

The first plot illustrates that the :ref:`Inertial3D` module is able to achieve a stable inertial pointing.

.. image:: /_images/Scenarios/scenarioMtbMomentumManagement1.svg
   :align: center

The next plots illustrate the RW states.  The motor torque are initially large to stabilize the
spacecraft orientation.  After this they return to small values that are compensating for the
magnetic momentum dumping.  The RW spin rates converge to the desired values over time.

.. image:: /_images/Scenarios/scenarioMtbMomentumManagement3.svg
   :align: center

.. image:: /_images/Scenarios/scenarioMtbMomentumManagement4.svg
   :align: center

The following plots illustrate the sensed magnetic field as well as the TAM commanded dipoles.

.. image:: /_images/Scenarios/scenarioMtbMomentumManagement6.svg
   :align: center
.. image:: /_images/Scenarios/scenarioMtbMomentumManagement7.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft with RWs, TAMs and MTBs to perform RW momentum dumping.
# Author:   Henry Macanas and Hanspeter Schaub
# Creation Date:  June 22, 2021
#

import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (mrpFeedback, attTrackingError,
                                    inertial3D, thrFiringSchmitt, thrForceMapping, thrFiringRemainder)
from Basilisk.fswAlgorithms import thrForceMapping
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import simpleNav
from Basilisk.simulation import (simpleNav,
                                 magneticFieldWMM,
                                 spacecraft, thrusterDynamicEffector)
from Basilisk.utilities import (SimulationBaseClass, macros,
                                orbitalMotion, simIncludeGravBody, fswSetupThrusters,
                                simIncludeThruster, unitTestSupport, vizSupport)

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


# Plotting functions
# Plotting functions
def plot_attitude_error(timeData, dataSigmaBR):
    """Plot the attitude errors."""
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeData, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')
    plt.grid(True)

def plot_rate_error(timeData, dataOmegaBR):
    """Plot the body angular velocity rate tracking errors."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeData, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')
    plt.grid(True)
    
def plot_requested_torque(timeDataFSW, dataLr):
    """Plot the commanded attitude control torque."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeDataFSW, dataLr[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')

def plot_thrForce(timeDataFSW, dataMap, numTh):
    """Plot the Thruster force values."""
    plt.figure(4)
    for idx in range(numTh):
        plt.plot(timeDataFSW, dataMap[:, idx],
                 color=unitTestSupport.getLineColor(idx, numTh),
                 label='$thrForce,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Force requested [N]')

def plot_OnTimeRequest(timeDataFSW, dataSchm, numTh):
    """Plot the thruster on time requests."""
    plt.figure(5)
    for idx in range(numTh):
        plt.plot(timeDataFSW, dataSchm[:, idx],
                 color=unitTestSupport.getLineColor(idx, numTh),
                 label='$OnTimeRequest,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('OnTimeRequest [sec]')

    

    
def run(show_plots, useDVThrusters):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        useJitterSimple (bool): Specify if the RW simple jitter model should be included
        useRWVoltageIO (bool): Specify if the RW voltage interface should be simulated.

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    fswTaskName = "fswTask"
    fswProcessName = "fswProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(120.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)
    fswProcess = scSim.CreateNewProcess(fswProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(5400.0)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))
    fswTimeStep = macros.sec2nano(0.5)
    fswProcess.addTask(scSim.CreateNewTask(fswTaskName, fswTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "MBZSAT"
    # define the simulation inertia
    I = [517.2, 0., 0.,
         0., 523.6, 0.,
         0., 0., 300.2] #kg m^2
    scObject.hub.mHub = 707.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject, None, 1)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    #   set initial Spacecraft States
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0  # meters
    oe.e = 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B


    # setup extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    extFTObject.extTorquePntB_B = [[0.25], [-0.25], [0.1]]
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    # create magnetic field module
    magModule = magneticFieldWMM.MagneticFieldWMM()
    magModule.ModelTag = "WMM"
    magModule.dataPath = bskPath + '/supportData/MagneticField/'
    epochMsg = unitTestSupport.timeStringToGregorianUTCMsg('2019 June 27, 10:23:0.0 (UTC)')
    magModule.epochInMsg.subscribeTo(epochMsg)
    magModule.addSpacecraftToModel(scObject.scStateOutMsg)  # this command can be repeated if multiple
    scSim.AddModelToTask(simTaskName, magModule)

    location = [[0.0002,-0.223,-0.0345],
                [-0.0002,0.223,-0.0345]]

    direction = [[0.0, 0.0, 1.0],
                [0.0, 0.0, 1.0]]

    # create the set of thruster in the dynamics task
    thrusterSet = thrusterDynamicEffector.ThrusterDynamicEffector()
    scSim.AddModelToTask(simTaskName, thrusterSet)

    # Make a fresh thruster factory instance, this is critical to run multiple times
    thFactory = simIncludeThruster.thrusterFactory()

    # create the thruster devices by specifying the thruster type and its location and direction
    for pos_B, dir_B in zip(location, direction):

        if useDVThrusters:
            thFactory.create('MBZ_Thruster', pos_B, dir_B)
        else:
            thFactory.create('MBZ_Thruster', pos_B, dir_B)

    # get number of thruster devices
    numTh = thFactory.getNumOfDevices()

    # create thruster object container and tie to spacecraft object
    thrModelTag = "ACSThrusterDynamics"
    thFactory.addToSpacecraft(thrModelTag, thrusterSet, scObject)


    #
    #   setup the FSW algorithm tasks
    #

    # setup inertial3D guidance module
    inertial3DConfig = inertial3D.inertial3DConfig()
    inertial3DWrap = scSim.setModelDataWrap(inertial3DConfig)
    inertial3DWrap.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DWrap, inertial3DConfig)
    inertial3DConfig.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation

    # setup the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attErrorWrap, attErrorConfig)

    # setup the MRP Feedback control module
    mrpControlConfig = mrpFeedback.mrpFeedbackConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControlWrap, mrpControlConfig)
    
    mrpControlConfig.Ki = -1  # make value negative to turn off integral feedback
    mrpControlConfig.K = 0.0001
    mrpControlConfig.P = 0.002
    mrpControlConfig.integralLimit = 2. / mrpControlConfig.Ki * 0.1

    # setup the thruster force mapping module
    thrForceMappingConfig = thrForceMapping.thrForceMappingConfig()
    thrForceMappingWrap = scSim.setModelDataWrap(thrForceMappingConfig)
    thrForceMappingWrap.ModelTag = "thrForceMapping"
    scSim.AddModelToTask(fswTaskName, thrForceMappingWrap, thrForceMappingConfig)
    if useDVThrusters:
        controlAxes_B = [1, 0, 0,
                         0, 1, 0]
        thrForceMappingConfig.thrForceSign = -1
    else:
        controlAxes_B = [1, 0, 0,
                         0, 1, 0,
                         0, 0, 1]
        thrForceMappingConfig.thrForceSign = +1
    thrForceMappingConfig.controlAxes_B = controlAxes_B

    # # setup the thruster force mapping module
    # thrFiringRemainderConfig = thrFiringRemainder.thrFiringRemainderConfig()
    # thrFiringRemainderWrap = scSim.setModelDataWrap(thrFiringRemainderConfig)
    # thrFiringRemainderWrap.ModelTag = "thrFiringRemainder"
    # scSim.AddModelToTask(fswTaskName, thrFiringRemainderWrap, thrFiringRemainderConfig)

    # setup the Schmitt trigger thruster firing logic module
    thrFiringSchmittConfig = thrFiringSchmitt.thrFiringSchmittConfig()
    thrFiringSchmittWrap = scSim.setModelDataWrap(thrFiringSchmittConfig)
    thrFiringSchmittWrap.ModelTag = "thrFiringSchmitt"
    scSim.AddModelToTask(fswTaskName, thrFiringSchmittWrap, thrFiringSchmittConfig)
    thrFiringSchmittConfig.thrMinFireTime = 0.002
    thrFiringSchmittConfig.level_on = .75
    thrFiringSchmittConfig.level_off = .25
    if useDVThrusters:
        thrFiringSchmittConfig.baseThrustState = 1


#
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    mrpLog = mrpControlConfig.cmdTorqueOutMsg.recorder(samplingTime)
    attErrLog = attErrorConfig.attGuidOutMsg.recorder(samplingTime)
    thrMapLog = thrForceMappingConfig.thrForceCmdOutMsg.recorder(samplingTime)
    thrTrigLog = thrFiringSchmittConfig.onTimeOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(fswTaskName, mrpLog)
    scSim.AddModelToTask(fswTaskName, attErrLog)
    scSim.AddModelToTask(fswTaskName, thrMapLog)
    scSim.AddModelToTask(fswTaskName, thrTrigLog)

    #
    # create FSW simulation messages
    #

    # create the FSW vehicle configuration message

    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # create the FSW Thruster configuration message
    if useDVThrusters:
        maxThrust = 22
    else:
        maxThrust = 1

    # A `clearSetup()` should be called first to clear out any pre-existing devices from an
    # earlier simulation run.  Next, the `maxThrust` value should be specified and used in the macro `create()`,
    # together with the locations and directions, and looped through a for cycle to consider all the thrusters.
    # The support macro `writeConfigMessage()` creates the required thrusters flight configuration message.
    fswSetupThrusters.clearSetup()
    for pos_B, dir_B in zip(location, direction):
        fswSetupThrusters.create(pos_B, dir_B, maxThrust)
    fswThrConfigMsg = fswSetupThrusters.writeConfigMessage()

    # connect messages
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attErrorConfig.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attErrorConfig.attRefInMsg.subscribeTo(inertial3DConfig.attRefOutMsg)
    mrpControlConfig.guidInMsg.subscribeTo(attErrorConfig.attGuidOutMsg)
    mrpControlConfig.vehConfigInMsg.subscribeTo(vcMsg)
    thrForceMappingConfig.cmdTorqueInMsg.subscribeTo(mrpControlConfig.cmdTorqueOutMsg)
    thrForceMappingConfig.thrConfigInMsg.subscribeTo(fswThrConfigMsg)
    thrForceMappingConfig.vehConfigInMsg.subscribeTo(vcMsg)
    thrFiringSchmittConfig.thrConfInMsg.subscribeTo(fswThrConfigMsg)
    thrFiringSchmittConfig.thrForceInMsg.subscribeTo(thrForceMappingConfig.thrForceCmdOutMsg)
    thrFiringRemainderConfig.thrConfInMsg.subscribeTo(fswThrConfigMsg)
    thrFiringRemainderConfig.thrForceInMsg.subscribeTo(thrForceMappingConfig.thrForceCmdOutMsg)
    thrusterSet.cmdsInMsg.subscribeTo(thrFiringSchmittConfig.onTimeOutMsg)

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName,  scObject
                                              # , saveFile=fileName
                                              , thrEffectorList=thrusterSet
                                              , thrColors=vizSupport.toRGBA255("red")
                                              )
    vizSupport.setActuatorGuiSetting(viz, showThrusterLabels=True)

     #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    dataLr = mrpLog.torqueRequestBody
    dataSigmaBR = attErrLog.sigma_BR
    dataOmegaBR = attErrLog.omega_BR_B
    dataMap = thrMapLog.thrForce
    dataSchm = thrTrigLog.OnTimeRequest
    np.set_printoptions(precision=16)

        #
    #   plot the results
    #
    timeDataFSW = attErrLog.times() * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs

    plot_requested_torque(timeDataFSW, dataLr)
    figureList = {}
    pltName = fileName + "1" + str(int(useDVThrusters))
    figureList[pltName] = plt.figure(1)

    plot_rate_error(timeDataFSW, dataOmegaBR)
    pltName = fileName + "2" + str(int(useDVThrusters))
    figureList[pltName] = plt.figure(2)

    plot_attitude_error(timeDataFSW, dataSigmaBR)
    pltName = fileName + "3" + str(int(useDVThrusters))
    figureList[pltName] = plt.figure(3)

    plot_thrForce(timeDataFSW, dataMap, numTh)
    pltName = fileName + "4" + str(int(useDVThrusters))
    figureList[pltName] = plt.figure(4)

    plot_OnTimeRequest(timeDataFSW, dataSchm, numTh)
    pltName = fileName + "5" + str(int(useDVThrusters))
    figureList[pltName] = plt.figure(5)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots
        False,  # useDVThrusters
    )
