'''Vehicle state manager'''
class VechicleState():
    '''current state of vehicle'''
    def __init__(self, name, body):
        self.name = name
        self.body = body

def land_state():
    '''creates a land state'''
    return VechicleState("land", {})

def takeoff_state(alt):
    '''creates a takeoff state'''
    return VechicleState("land", {"alt": alt})
