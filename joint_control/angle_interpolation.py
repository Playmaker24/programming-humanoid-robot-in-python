'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = self.perception.time

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        #current running time in reference to perceptron time
        run_time = perception.time - self.start_time
        
        for i, joint in enumerate(keyframes[0]):
            for t in range (len(keyframes[1][i]) -1):
                #print('time_0:', keyframes[1][i][t])
                #print('run time:', run_time)
                #print('time_3:', keyframes[1][i][t+1])
                if keyframes[1][i][t] < run_time < keyframes[1][i][t+1]:
                    #value(angle) of the points(p_0,...,p_3)
                    y_0 = keyframes[2][i][t][0] # initial point
                    y_1 = y_0 + keyframes[2][i][t][2][2] # control point after initial point
                    y_3 = keyframes[2][i][t+1][0] # end point
                    y_2 = y_3 - keyframes[2][i][t+1][1][2] # control point before end point
                    t = (run_time-keyframes[1][i][t])/(keyframes[1][i][t+1] - keyframes[1][i][t]) 
                    y_t = (1-t)**3 * y_0 + 3*(1-t)**2 * t * y_1 + 3*(1-t)* t**2 * y_2 + t**3 * y_3
                    target_joints.update({joint:y_t})
                else:
                    continue     
        
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
