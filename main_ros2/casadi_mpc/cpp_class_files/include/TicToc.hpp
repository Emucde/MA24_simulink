#ifndef TIC_TOC_HPP
#define TIC_TOC_HPP

#include <chrono>  // for time measurement
#include <iostream>
#include <sstream> // for std::ostringstream
#include <iomanip> // for std::setprecision

class TicToc
{
public:
    TicToc();

    void tic(bool reset = false);

    double toc();

    double get_time();

    std::string get_time_str(const std::string &additional_text = "time");

    void print_time(const std::string &additional_text = "time");

    void reset();

    double get_frequency();

    void print_frequency(std::string additional_text = "Frequency: ");

    std::string format_frequency(double frequency, int precision = 2);

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    double elapsed_time;       // current elapsed time in seconds
    double elapsed_total_time; // total elapsed time in seconds
    bool running;

    std::string format_time(double elapsed_time, int precision = 2);
};

#endif  // TIC_TOC_HPP
