#include <iostream>
#include <iomanip>
#include <cmath>

using namespace std;

struct BmpCalib {
    double t1, t2, t3;
    double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
};

BmpCalib _bmpCalib;

double compensateTemp(double uncomp_temp) {
    double partial_data1 = uncomp_temp - _bmpCalib.t1;
    double partial_data2 = partial_data1 * _bmpCalib.t2;
    return partial_data2 + (partial_data1 * partial_data1) * _bmpCalib.t3;
}

double compensatePress(double uncomp_press, double t_lin) {
    double partial_data1;
    double partial_data2;
    double partial_data3;
    double partial_data4;
    double partial_out1;
    double partial_out2;

    partial_data1 = _bmpCalib.p6 * t_lin;
    partial_data2 = _bmpCalib.p7 * (t_lin * t_lin);
    partial_data3 = _bmpCalib.p8 * (t_lin * t_lin * t_lin);
    partial_out1 = _bmpCalib.p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = _bmpCalib.p2 * t_lin;
    partial_data2 = _bmpCalib.p3 * (t_lin * t_lin);
    partial_data3 = _bmpCalib.p4 * (t_lin * t_lin * t_lin);
    partial_out2 = uncomp_press * (_bmpCalib.p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = uncomp_press * uncomp_press;
    partial_data2 = _bmpCalib.p9 + _bmpCalib.p10 * t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + (uncomp_press * uncomp_press * uncomp_press) * _bmpCalib.p11;

    return partial_out1 + partial_out2 + partial_data4;
}

int main() {
    // Dummy calib
    _bmpCalib.t1 = 27504.0;
    _bmpCalib.t2 = 1.0;
    _bmpCalib.t3 = -0.005;
    
    _bmpCalib.p1 = 1.0;
    _bmpCalib.p2 = 0.0;
    _bmpCalib.p3 = 0.0;
    _bmpCalib.p4 = 0.0;
    _bmpCalib.p5 = 100000.0;
    _bmpCalib.p6 = 0.0;
    _bmpCalib.p7 = 0.0;
    _bmpCalib.p8 = 0.0;
    _bmpCalib.p9 = 0.0;
    _bmpCalib.p10 = 0.0;
    _bmpCalib.p11 = 0.0;

    double t_lin = compensateTemp(27504.0 + 20.0);
    double press = compensatePress(100.0, t_lin);
    cout << "Temp: " << t_lin << " Press: " << press << endl;
    return 0;
}
