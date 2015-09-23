#!/usr/bin/env python
import math, sys
from robot import ROSRobot, PoseUpdater

class Sunflower(ROSRobot):
    """ Concrete implementation of the Robot interface for the UH Sunflower model of robots """
    _imageFormats = ['BMP', 'EPS', 'GIF', 'IM', 'JPEG', 'PCD', 'PCX', 'PDF', 'PNG', 'PPM', 'TIFF', 'XBM', 'XPM']

    def __init__(self, name, rosMaster):
        from rosHelper import ROS
        ROS.configureROS(rosMaster=rosMaster)
	print rosMaster
        super(Sunflower, self).__init__(name, ActionLib, 'sf_controller', '')

    def setComponentState(self, name, value, blocking=True):
        # check if the component has been initialised, and init if it hasn't
        if name == 'base':
            self._robInt.initComponent(name)
        if name == 'torso':
            name = 'roomba'
            if value == 'back':
                value = 'clean'
            elif value == 'front':
                value = 'spot'

        if name == 'roomba':
            return self._roomba.setComponentState(value, None)

        return super(Sunflower, self).setComponentState(name, value, blocking)
            
    def getComponentState(self, componentName, dontResolveName=False):
        """ Return the a tuple containing the state (named and raw) of the named component.  """
        """ Name resolution can be skipped by setting dontResolveName=True. """
        """ ('Name or Empty', {'name': '...', 'positions': [..., ...], 'goals': [..., ...], 'joints': [..., ...] })"""
        topic = '/%(name)s_controller/state' % { 'name': componentName }
        state = self._ros.getSingleMessage(topic)
        
        try:
            ret = {'name': componentName, 'positions': (state.current_pos, ), 'goals': (state.goal_pos, ), 'joints': (state.name, ) }
        except Exception as e:
            print "Error retrieving joint state for %s" % componentName
            print e
            ret = {'name': componentName, 'positions': (), 'goals': (), 'joints': () }
            
        if dontResolveName:
            return ('', ret)
        else:
            return self.resolveComponentState(componentName, ret)

    def play(self, fileName, blocking=True):
        self.setComponentState('play', fileName)

    @property
    def _transform(self):
        """ Transform between the robot base frame and the map frame, used in getLocation() """
        if self._tf == None:
            try:
                import rosHelper
                self._tf = rosHelper.Transform(rosHelper=self._rs, toTopic='/base_link', fromTopic='/map')
            except Exception as e:
                print >> sys.stderr, "Error occured while calling transform: %s" % repr(e)
        return self._tf

class ActionLib(object):
        
    def __init__(self):
        import rosHelper
        self._ros = rosHelper.ROS()
        self._ros.configureROS(packageName='sf_controller')
        self._ros.configureROS(packageName='sf_lights')
        
        import actionlib, sf_controller_msgs.msg, sf_lights_msgs.msg
        self._sfMsgs = sf_controller_msgs.msg
        self._sfLights = sf_lights_msgs.msg
        
        self._ros.initROS()
        self._sfClient = actionlib.SimpleActionClient('/sf_controller', self._sfMsgs.SunflowerAction)
        print "Waiting for sf_controller..."
        self._sfClient.wait_for_server()
        print "Connected to sf_controller"

        self._sfLight = actionlib.SimpleActionClient('/lights', self._sfLights.LightsAction)
        print "Waiting for sf_lights..."
        self._sfLight.wait_for_server()
        print "Connected to sf_lights"
        
    def runFunction(self, funcName, kwargs):
        return 5
        
    def initComponent(self, name):
        if name == 'base':
            goal = self._sfMsgs.SunflowerGoal(
                                                   action='init',
                                                   component=name)
            client = self._sfClient
            return client.send_goal_and_wait(goal)
        else:
            return 3
    
    def runComponent(self, name, value, mode=None, blocking=True):
        if name == 'light':
	    if value == 'red':
                value = [1, 0, 0]
	    elif value == 'green':
                value = [0, 1, 0]
	    elif value == 'yellow':
                value = [1, 1, 0]
	    elif value == 'blue':
                value = [0, 0, 1]
	    elif value == 'white':
                value = [1, 1, 1]
            elif value == 'black':
                value = [0, 0, 0]
            goal = self._sfLights.LightsGoal(rgb=value)
            client = self._sfLight
        elif name == 'play':
            goal = self._sfMsgs.SunflowerGoal(
                                                   action='play',
                                                   component=value,
                                                   namedPosition='',
                                                   jointPositions=[])
            client = self._sfClient
        else:
            (namedPosition, joints) = (value, []) if str == type(value) else ('', value)
            
            goal = self._sfMsgs.SunflowerGoal(
                                                   action='move',
                                                   component=name,
                                                   namedPosition=namedPosition,
                                                   jointPositions=joints)
            client = self._sfClient

        if(blocking):
            status = client.send_goal_and_wait(goal)
        else:
            client.send_goal(goal)
            status = 1
            
        return status
    

class PoseUpdater(PoseUpdater):
    """ Concrete implementation of pose updates for Sunflower robots """
    
    def __init__(self, robot):
        #super(PoseUpdater, self).__init__(robot)
        self._rs = None
        
    @property
    def _ros(self):
        if self._rs == None:
            # Wait to configure/initROS ROS till it's actually needed
            self._rs = rosHelper.ROS()
        return self._rs
    
    def checkUpdatePose(self, robot):
        states = {}
        states.update(self.getTrayStates(robot))
        states.update(self.getHeadStates(robot))
        self.updateStates(states)
        
    def getHeadStates(self, robot):
        """ Returns a dictionary with values for sensor 'eyePosition' to the current angle of the camera """
        return {
                   'eyePosition': ([0,], 'front'),
               }

    def getComponentPosition(self, robot, componentName):
        (stateName, state) = robot.getComponentState(componentName)
        if stateName == None or len(state['positions']) == 0:
            if len(state['positions']) == 0:
                #Error while retreiving state
                return (None, None)

        if stateName == '':
            #print "No named component state for: %s." % (componentName)
            stateName = 'Unknown'
        
        p = []
        for position in state['positions']:
            p.append(round(position, 3))
        
        return (p, stateName)

    def getTrayStates(self, robot):
        """ Returns a dictionary containing the values for sensor variables 'trayStatus' and 'trayIs' """
        return {
                   'trayStatus': self.getComponentPosition(robot, 'tray'),
                   'trayIs': ([0,], 'empty') 
               }
            
if __name__ == '__main__':
    """ Run pose and location updates for the currentRobot """
    from robotFactory import Factory
    robot = Factory.getCurrentRobot()

    import locations
    from history import SensorLog
    l = locations.RobotLocationProcessor(robot)
    rp = PoseUpdater(robot)
    sr = SensorLog(rp.channels, rp.robot.name)

    rp.start()
    sr.start()
    
    l.start()
    
    while True:
        try:
            sys.stdin.read()
        except KeyboardInterrupt:
            break
    l.stop()

    sr.stop()
    rp.stop()
    
