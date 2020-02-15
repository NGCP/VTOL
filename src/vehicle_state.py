class Vechicle_State():
    def __init__(self, name, body):
        self.name = name
        self.body = body

def land_state():
    '''creates a land state'''
    return Vechicle_State("land", {})

def takeoff_state(alt):
    '''creates a takeoff state'''
    return Vechicle_State("land", {"alt": alt})
