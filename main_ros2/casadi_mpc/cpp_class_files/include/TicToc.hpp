#ifndef TIC_TOC_HPP
#define TIC_TOC_HPP

#include <chrono>  // for high_resolution_clock, time_point
#include <string>  // for string, allocator

/*
This class provides a simple timing utility to measure elapsed time.
It can be used to measure the time taken for specific operations or processes.
It includes methods to start the timer, stop it, get the elapsed time,
and print the time in a formatted string.
It also provides methods to calculate and print the frequency of operations.
The class uses high-resolution clock for accurate timing.
*/
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
