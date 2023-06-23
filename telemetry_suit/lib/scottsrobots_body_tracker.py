from scottsrobots_cp_quaternion import Quaternion
class Joint():
    def __init__(self,bno085=None):
        self.bno085 = bno085
        self.raw_orientation = Quaternion()
        self.offset_multiplier = Quaternion()
        self.offset_orientation = Quaternion()
        self.servo_orientation = Quaternion()
        self.parent = None
        self.children = []
    def update_raw_orientation(self):
        self.raw_orientation = Quaternion.from_tuple(self.bno085.quaternion)
    def update_offset_multiplier(self):
        #offset from zero postion
        self.offset_multiplier = Quaternion.conjugate(self.raw_orientation)
    def update_offset_orientation(self):
        self.offset_orientation = Quaternion.multiply(self.raw_orientation,self.offset_multiplier)
    def update_servo_orientation(self):
        #for this we need to multiply the offset_orientation by all ancestors
        current_joint = self
        return_orientation = Quaternion.from_quaternion(self.offset_orientation)
        while current_joint.parent != None:
            current_joint = current_joint.parent
            return_orientation = Quaternion.multiply(return_orientation, Quaternion.conjugate(current_joint.parent.offset_orientation))
        self.servo_orientation = return_orientation
    def add_child(self, child_joint):
        child_joint.parent = self
        self.children.append(child_joint)
    def add_new_child(self, bno085 = None):
        new_child = joint(bno085 = bno085)
        self.add_child(Joint())
    def update_orientation(self, update_offset_multiplier = False, update_children=True):
        self.update_raw_orientation()
        if update_offset_multiplier: self.update_offset_multiplier()
        self.update_offset_orientation()
        self.update_servo_orientation()
        if update_children:
            for child in self.children:
                child.update_orientation(update_offset_multiplier)


class TrackedBody():
    def __init__(self):
        self.joint_chest = Joint()
        #self.joint_head = self.joint_chest.add_new_child()
        self.joint_root = self.joint_chest
    def initialize_orientations(self):
        self.joint_root.update_orientation(update_offset_multiplier = True)
    def update_orientations(self):
        self.joint_root.update_orientation()
