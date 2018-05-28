'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello, leftBackToStand, rightBackToStand
import pickle
import numpy as np

features = [
	'LHipYawPitch', 
	'LHipRoll', 
	'LHipPitch', 
	'LKneePitch', 
	'RHipYawPitch', 
	'RHipRoll', 
	'RHipPitch', 
	'RKneePitch', 
	'AngleX', 
	'AngleY'
]
robot_pose = {
	0: 'Back',
	1: 'Belly',
	2: 'Crouch',
	3: 'HeadBack',
	4: 'Knee',
	5: 'Left',
	6: 'Right',
	7: 'Sit',
	8: 'Stand',
	9: 'StandInit'
}

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
	self.features = features
	self.robot_pose = robot_pose
        self.posture = 'unknown'
	self.posture_classifier = pickle.load(open("robot_pose.pkl"))


    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        # YOUR CODE HERE
	data = []
	for feature in self.features:
		if feature == 'AngleX':
			data.append(perception.imu[0])
		elif feature == 'AngleY':
			data.append(perception.imu[1])
		else:
			data.append(perception.joint[feature])
	
	predict = self.posture_classifier.predict(np.array(data).reshape(1, -1))
	posture = self.robot_pose[predict[0]]

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = rightBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
