import numpy as np    
    

class WayPoints():
    def __init__(self):
        self.pos_list = [
                [0,-98], # 0th wp
                [10,-98],
                [20,-98],
                [30,-98],
                [40,-98],
                [50,-98],
                [60,-98],
                [70,-98],
                [80,-98],
                [90,-98],
                [100,-98], # 10th
                [110,-98],
                [120,-98],
                [130,-98],
                [140,-98],
                [150,-98],
                [160,-98],
                [170,-98],
                [180,-98],
                [190,-98],
                [200,-98], # 20th
                [210,-98],
                [220,-97],
                [230,-95],
                [240,-92],
                [250,-87],
                [260,-81],
                [270,-74],
                [280,-63],
                [290,-49],
                [295,-37], # 30 th
                [300,-22],
                [303,-5],
                [303,15],
                [303,25],
                [303,35],
                [303,50],
                [301,63],
                [295,77],
                [290,83],
                [285,87], # 40th
                [280,90],
                [275,93],
                [270,94.5],
                [250,96.5],
                [240,96.5],
                [230,96.5],
                [220,96.5],
                [210,96.5],
                [200,96.5],
                [190,96.5], # 50th
                [180,96.5],
                [170,96.5],
                [160,96.5],
                [150,96.5],
                [140,96.5],
                [130,96.5],
                [120,96.5],
                [110,96.5],
                [100,96.5],
                [90,96.5], # 60th
                [80,96.5],
                [70,96.5],
                [60,96.5],
                [50,96.5],
                [40,99],
                [30,103],
                [20,109],
                [15,115],
                [9,125],
                [5,135], # 70th
                [4,145],
                [3,157],
                [-0.5,170],
                [-7,180],
                [-14,186.5],
                [-29,194],
                [-45,197],
                [-56.5,194.5],
                [-65.5,191.5],
                [-81.5,177.5], # 80th
                [-87,169.5],
                [-90,160],
                [-92,152],
                [-97.5,123.5],
                [-104.5,113],
                [-118,103],
                [-132,98.5],
                [-144,96.5],
                [-155,95],
                [-167,91], # 90th
                [-180.5,79],
                [-187,68.5], 
                [-191,53],
                [-191,33],
                [-191,13],
                [-191,-3],
                [-189,-20],
                [-184.5,-34.5],
                [-178.5,-50.5],
                [-166.5,-66], # 100th
                [-150,-80.5],
                [-138.5,-87.5], 
                [-125,-93.5],
                [-113.5,-96.5],
                [-96,-98],
                [-86,-98],
                [-76,-98],
                [-66,-98],
                [-56,-98],
                [-46,-98],
                [-36,-98],
                [-26,-98],
                [-16,-98], # 113-th wp
    ]

    def getWayPoints(self):
        return self.pos_list