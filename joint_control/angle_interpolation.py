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
from keyframes import hello, leftBackToStand, leftBellyToStand


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.stime = None

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}

        if self.stime == None:
            self.stime = perception.time

        (names, times, keys) = keyframes
        stime = perception.time - self.stime

        for i in range(len(names)):
            joint = names[i]
            time = times[i]
            key = keys[i]

            if joint in self.joint_names:
                for j in range(len(time) - 1):
                    if stime < time[0] and j == 0:
                        t0 = 0.0
                        t3 = time[0]
                        p0 = perception.joint[joint]
                        p3 = key[0][0]
                    elif time[j] < stime < time[j+1]:
                        t0 = time[j]
                        t3 = time[j+1]
                        p0 = key[j][0]
                        p3 = key[j+1][0]
                    else:
                        continue

                    p1 = key[j][1][1] + p0
                    p2 = key[j][2][1] + p3
                    t1 = key[j][1][2] + t0
                    t2 = key[j][2][2] + t3

                    t = (stime - t0) / (t3 - t0)

                    target_joints[joint] = self.bezier_algorithm(p0, p1, p2, p3, t)
        return target_joints

    @staticmethod
    def bezier_algorithm(p0, p1, p2, p3, t):
        fact0 = (1 - t)**3
        fact1 = 3 * (1 - t)**2 * t
        fact2 = 3 * (1 - t) * t**2
        fact3 = t**3

        return (fact0 * p0) + (fact1 * p1) + (fact2 * p2) + (fact3 * p3)

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBellyToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
