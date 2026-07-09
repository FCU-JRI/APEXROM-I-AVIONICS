#include <iostream>
#include <cstdint>

struct BmpCalibrationData {
    uint16_t par_t1 = 27598;
    uint16_t par_t2 = 17565;
    int8_t par_t3 = -10;
    int16_t par_p1 = 34567;
    int16_t par_p2 = 5678;
    int8_t par_p3 = 10;
    int8_t par_p4 = 0;
    uint16_t par_p5 = 18000;
    uint16_t par_p6 = 20000;
    int8_t par_p7 = -5;
    int8_t par_p8 = -5;
    int16_t par_p9 = 5000;
    int8_t par_p10 = 0;
    int8_t par_p11 = 0;
    int64_t t_lin;
} _bmpCalib;

int64_t compensateTemp(uint32_t uncomp_temp) {
    int64_t partial_data1 = ((int64_t)uncomp_temp - (256 * _bmpCalib.par_t1));
    int64_t partial_data2 = _bmpCalib.par_t2 * partial_data1;
    int64_t partial_data3 = (partial_data1 * partial_data1);
    int64_t partial_data4 = (int64_t)partial_data3 * _bmpCalib.par_t3;
    int64_t partial_data5 = ((int64_t)(partial_data2 * 262144) + partial_data4);
    int64_t partial_data6 = partial_data5 / 4294967296LL;
    _bmpCalib.t_lin = partial_data6;
    return (int64_t)((partial_data6 * 25) / 16384);
}

uint64_t compensatePress(uint32_t uncomp_press) {
    int64_t partial_data1 = _bmpCalib.t_lin * _bmpCalib.t_lin;
    int64_t partial_data2 = partial_data1 / 64;
    int64_t partial_data3 = (partial_data2 * _bmpCalib.t_lin) / 256;
    int64_t partial_data4 = (_bmpCalib.par_p8 * partial_data3) / 32;
    int64_t partial_data5 = (_bmpCalib.par_p7 * partial_data1) * 16;
    int64_t partial_data6 = (_bmpCalib.par_p6 * _bmpCalib.t_lin) * 4194304LL;
    int64_t offset = ((int64_t)_bmpCalib.par_p5 * 140737488355328LL) + partial_data4 + partial_data5 + partial_data6;
    
    partial_data2 = (_bmpCalib.par_p4 * partial_data3) / 32;
    partial_data4 = (_bmpCalib.par_p3 * partial_data1) * 4;
    partial_data5 = ((int64_t)_bmpCalib.par_p2 - 16384) * _bmpCalib.t_lin * 2097152LL;
    int64_t sensitivity = (((int64_t)_bmpCalib.par_p1 - 16384) * 70368744177664LL) + partial_data2 + partial_data4 + partial_data5;
    
    partial_data1 = (sensitivity / 16777216LL) * uncomp_press;
    partial_data2 = _bmpCalib.par_p10 * _bmpCalib.t_lin;
    partial_data3 = partial_data2 + (65536LL * _bmpCalib.par_p9);
    partial_data4 = (partial_data3 * uncomp_press) / 8192;

    partial_data5 = (uncomp_press * (partial_data4 / 10)) / 512;
    partial_data5 = partial_data5 * 10;
    partial_data6 = (int64_t)((uint64_t)uncomp_press * (uint64_t)uncomp_press);
    partial_data2 = (_bmpCalib.par_p11 * partial_data6) / 65536LL;
    partial_data3 = (partial_data2 * uncomp_press) / 128;
    partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3;
    return (((uint64_t)partial_data4 * 25) / (uint64_t)1099511627776LL);
}

int main() {
    uint32_t raw_temp = 8388608; // Typical mid-range raw value
    uint32_t raw_press = 8388608;
    int64_t temp = compensateTemp(raw_temp);
    uint64_t press = compensatePress(raw_press);
    std::cout << "Temp: " << temp << " (expected ~2500 for 25C)" << std::endl;
    std::cout << "Press: " << press << " (expected ~10132500 for 1atm)" << std::endl;
    return 0;
}
