#pragma once

#include <vector>
#include <string>

/// @brief Calculates what the time would be 1 ms after 'last time'
/// @param last_time the time to which 1 ms should be added
/// @return The last time plus one millisecond
struct timespec add_one_millisecond(const struct timespec &last_time);

/// @brief Calculates the time in microseconds between given times
/// @param start The start time
/// @param end The end time
/// @return The microseconds difference
long micros_between_timestamps(const struct timespec &start, const struct timespec &end);

/// @brief Calculates whether the given integer is a prime. Intended to keep CPU busy
/// @param n The number to check
/// @return Whether n is prime
bool is_prime(int number);

/// @brief Performs some random calculations (check is prime) to keep the CPU busy
/// Supposed to keep the CPU busy for a fraction of a millisecond
void do_random_calculations();

/// @brief Performs time measurements
/// @param n_measurements Amount of measurements to take
/// @return The results
std::vector<int> perform_measurements(int n_measurements);

/// @brief Performs realtime time measurements
/// @param n_measurements Number of measurements to take
/// @return the measurements
std::vector<int> perform_evl_measurements(int n_measurements);

/// @brief Calculates the standard deviation
/// @param samples
/// @param n_samples
/// @param mean
/// @return standard deviation
double calculate_standard_deviation(int samples[], int n_samples, double mean);

/// @brief Calculates the mean and standard deviation of given samples
/// @param samples The samples
/// @return The mean and standard deviation
std::pair<double, double> calculate_stats(std::vector<int> &samples);

/// @brief Writes measurements
/// @param measurements The measurements to write
/// @param filename The filename to store it in
/// @return Success
bool write_measurements_to_file(std::vector<int> &measurements, std::string filename);

/// @brief Function to be run on a non-EVL thread, which runs measurements.
/// @param arg Thread arguments
void *posix_thread_function(void *arg);

/// @brief Function to be run on an EVL thread, which runs measurements.
/// @param arg Thread arguments
void *evl_thread_function(void *arg);