class Node:
    #
    def __init__(self, label="", coords=(0,0)):
        self.label=label
        self.coords=coords
    #
    def set_label(self, label_p):
        self.label=label_p
    def get_label(self):
        return self.label
    #
    def set_coords(self, coords_p):
        self.coords=coords_p
    def get_coords(self):
        return self.coords
    #
    def __str__(self):
        return self.label
