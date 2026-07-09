import math

class BMP388Calib:
    def __init__(self):
        self.t1 = 27504.0
        self.t2 = 1.0
        self.t3 = -0.5
        self.p1 = 0.0
        self.p2 = 0.0
        self.p3 = 0.0
        self.p4 = 0.0
        self.p5 = 0.0
        self.p6 = 0.0
        self.p7 = 0.0
        self.p8 = 0.0
        self.p9 = 0.0
        self.p10 = 0.0
        self.p11 = 0.0

calib = BMP388Calib()

def compensateTemp(uncomp_temp):
    partial_data1 = uncomp_temp - calib.t1
    partial_data2 = partial_data1 * calib.t2
    return partial_data2 + (partial_data1 * partial_data1) * calib.t3

def compensatePress(uncomp_press, t_lin):
    partial_data1 = calib.p6 * t_lin
    partial_data2 = calib.p7 * (t_lin * t_lin)
    partial_data3 = calib.p8 * (t_lin * t_lin * t_lin)
    partial_out1 = calib.p5 + partial_data1 + partial_data2 + partial_data3

    partial_data1 = calib.p2 * t_lin
    partial_data2 = calib.p3 * (t_lin * t_lin)
    partial_data3 = calib.p4 * (t_lin * t_lin * t_lin)
    partial_out2 = uncomp_press * (calib.p1 + partial_data1 + partial_data2 + partial_data3)

    partial_data1 = uncomp_press * uncomp_press
    partial_data2 = calib.p9 + calib.p10 * t_lin
    partial_data3 = partial_data1 * partial_data2
    partial_data4 = partial_data3 + (uncomp_press * uncomp_press * uncomp_press) * calib.p11

    return partial_out1 + partial_out2 + partial_data4

print("Parsing check")
