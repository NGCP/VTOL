'''structures related to vehicle flight'''

class Attitude:
    '''current attitude target for vehicle'''
    def __init__(self, pitch=0, yaw=0, roll=0, thrust=.5):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll
        self.thrust = thrust

    def set_pitch(self, pitch):
        '''sets pitch'''
        self.pitch = pitch

    def set_yaw(self, yaw):
        '''sets yaw'''
        self.yaw = yaw

    def set_roll(self, roll):
        '''sets roll'''
        self.roll = roll

    def set_thrust(self, thrust):
        '''sets thrust'''
        self.thrust = thrust
