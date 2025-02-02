#include "TicToc.hpp"

#include <chrono>    // for time_point, duration, high_resolution_clock, ope...
#include <iomanip>   // for operator<<, setprecision
#include <iostream>  // for operator<<, basic_ostream, char_traits, basic_os...

TicToc::TicToc() : start_time(), elapsed_time(0), elapsed_total_time(0), running(false) {}

void TicToc::tic(bool reset)
{
    if (reset)
    {
        start_time = std::chrono::time_point<std::chrono::high_resolution_clock>();
        elapsed_total_time = 0;
        running = false;
    }
    else
    {
        start_time = std::chrono::high_resolution_clock::now();
        running = true;
    }
}

double TicToc::toc()
{
    if (!running)
    {
        std::cerr << "Error: Tic not started.\n";
        return 0.0;
    }

    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_time_act = current_time - start_time;
    elapsed_time = elapsed_time_act.count();
    elapsed_total_time += elapsed_time;
    start_time = current_time;
    return elapsed_time;
}

double TicToc::get_time()
{
    return elapsed_total_time;
}

std::string TicToc::get_time_str(const std::string &additional_text)
{
    return "\033[92mElapsed " + additional_text + ": " + format_time(elapsed_total_time) + "\033[0m";
}

void TicToc::print_time(const std::string &additional_text)
{
    std::string time_str = get_time_str(additional_text);
    std::cout << time_str;
}

void TicToc::reset()
{
    start_time = std::chrono::time_point<std::chrono::high_resolution_clock>();
    elapsed_total_time = 0;
    running = false;
}

double TicToc::get_frequency()
{
    if (elapsed_total_time == 0)
    {
        std::cerr << "Error: No elapsed time. Cannot calculate frequency.\n";
        return 0;
    }
    double frequency = 1.0 / elapsed_time; // Frequency in Hz
    return frequency;
}

void TicToc::print_frequency(std::string additional_text)
{
    if (elapsed_total_time == 0)
    {
        std::cerr << "Error: No elapsed time. Cannot calculate frequency.\n";
        return;
    }
    double frequency = get_frequency();
    std::cout << "\033[92m" << additional_text << ": " << format_frequency(frequency) << "\033[0m";
}

std::string TicToc::format_frequency(double frequency, int precision)
{
    std::ostringstream oss;
    if (frequency < 1)
    {
        oss << std::fixed << std::setprecision(precision) << frequency << " Hz"; // Hertz
    }
    else if (frequency < 1e3)
    {
        oss << std::fixed << std::setprecision(precision) << frequency << " Hz"; // Hertz
    }
    else if (frequency < 1e6)
    {
        oss << std::fixed << std::setprecision(precision) << frequency / 1e3 << " kHz"; // kHz
    }
    else
    {
        oss << std::fixed << std::setprecision(precision) << frequency / 1e6 << " MHz"; // MHz
    }
    return oss.str();
}

std::string TicToc::format_time(double elapsed_time, int precision)
{
    std::ostringstream oss;
    if (elapsed_time == 0)
    {
        return "0 s";
    }
    else if (elapsed_time < 1e-6)
    {
        oss << std::fixed << std::setprecision(precision) << elapsed_time * 1e9 << " ns"; // Nanoseconds
    }
    else if (elapsed_time < 1e-3)
    {
        oss << std::fixed << std::setprecision(precision) << elapsed_time * 1e6 << " µs"; // Microseconds
    }
    else if (elapsed_time < 1)
    {
        oss << std::fixed << std::setprecision(precision) << elapsed_time * 1e3 << " ms"; // Milliseconds
    }
    else if (elapsed_time < 60)
    {
        oss << std::fixed << std::setprecision(precision) << elapsed_time << " s"; // Seconds
    }
    else
    {
        int minutes = static_cast<int>(elapsed_time / 60);
        double seconds = elapsed_time - minutes * 60;
        oss << minutes << " min " << std::fixed << std::setprecision(precision) << seconds << " s";
    }

    return oss.str();
}
