#include "performance_timer.h"

#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <memory>
#include <vector>
#include <numeric>
#include <string.h>

#include <evl/thread.h>
#include <evl/clock.h>
#include <evl/evl.h>

#define ONE_S_IN_NS 1000000000
#define ONE_MS_IN_NS 1000000
#define TOTAL_MEASUREMENTS 5 * 1000 // 5 seconds * 1000 ms
#define LOAD_CALCULATION_ITERS 100

struct timespec add_one_millisecond(const struct timespec &last_time)
{
	struct timespec next_time = last_time;

	next_time.tv_nsec += ONE_MS_IN_NS;

	// Make sure we add new seconds as well
	if (next_time.tv_nsec >= ONE_S_IN_NS)
	{
		// Remove a second worth in the nanoseconds
		next_time.tv_nsec -= ONE_S_IN_NS;
		// Add second to the seconds
		next_time.tv_sec++;
	}
	return next_time;
}

bool is_prime(int number)
{
	/// Implementation taken from StackOverflow
	if (number < 2)
		return false;
	if (number == 2)
		return true;
	if (number % 2 == 0)
		return false;
	for (int i = 3; (i * i) <= number; i += 2)
	{
		if (number % i == 0)
			return false;
	}
	return true;
}

void do_random_calculations()
{
	for (int i = 0; i < LOAD_CALCULATION_ITERS; i++)
	{
		is_prime(i);
	}
}

long micros_between_timestamps(const struct timespec &start, const struct timespec &end)
{
	long s_difference = end.tv_sec - start.tv_sec;
	long ns_difference = end.tv_nsec - start.tv_nsec;

	// Handle nanosecond underflow
	if (ns_difference < 0)
	{
		s_difference -= 1;
		ns_difference += ONE_S_IN_NS;
	}

	return s_difference * ONE_MS_IN_NS + (ns_difference / 1000);
}

double calculate_standard_deviation(int samples[], int n_samples, double mean)
{
	double diff_sum = 0;
	for (int i = 0; i < n_samples; i++)
	{
		diff_sum += static_cast<double>(samples[i]) - mean;
	}
	double variance = diff_sum / static_cast<double>(n_samples);
	return sqrt(variance);
}

std::vector<int> perform_measurements(int n_measurements)
{
	auto measurements = std::vector<int>();

	struct timespec start_measurement, expected_measurement, end_measurement;

	clock_gettime(CLOCK_MONOTONIC, &start_measurement);
	expected_measurement = add_one_millisecond(start_measurement);

	for (int i = 0; i < n_measurements; i++)
	{

		do_random_calculations();

		int result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &expected_measurement, NULL);
		// Check if sleeping went correctly
		if (result != 0)
		{
			printf("Error happened during clock_nanosleep");
			exit(EXIT_FAILURE);
		}

		clock_gettime(CLOCK_MONOTONIC, &end_measurement);

		// Now, we can calculate how long it took exactly
		int timediff = static_cast<int>(micros_between_timestamps(expected_measurement, end_measurement));

		measurements.push_back(timediff);

		// And update our next and last measurement timestamps
		start_measurement = expected_measurement;
		expected_measurement = add_one_millisecond(start_measurement);
	}

	return measurements;
}

std::vector<int> perform_evl_measurements(int n_measurements)
{
	auto measurements = std::vector<int>();

	struct timespec start_time, expected_time, end_time;

	// Attach the thread to the EVL real-time core
	auto efd = evl_attach_self("evl_thread");
	if (efd < 0)
	{
		evl_printf("Error: failed to attach thread: %s\n", strerror(-efd));
		exit(0);
	}

	// If we are not in band, exit
	if (evl_is_inband())
	{
		evl_printf("Error: Not out of band!");
		exit(0);
	}

	// Read the current EVL clock time
	evl_read_clock(EVL_CLOCK_MONOTONIC, &start_time);
	expected_time = add_one_millisecond(start_time);

	for (int i = 0; i < n_measurements; i++)
	{
		do_random_calculations(); // Simulate load

		// Sleep until the expected timestamp
		int result = evl_sleep_until(EVL_CLOCK_MONOTONIC, &expected_time);
		if (result != 0)
		{
			perror("Error during evl_sleep");
			exit(EXIT_FAILURE);
		}

		// Read actual wake-up time
		evl_read_clock(EVL_CLOCK_MONOTONIC, &end_time);

		// Calculate the time difference
		int timediff = static_cast<int>(micros_between_timestamps(expected_time, end_time));
		measurements.push_back(timediff);

		// Update timestamps for next cycle
		start_time = expected_time;
		expected_time = add_one_millisecond(start_time);
	}

	return measurements;
}

std::pair<double, double> calculate_stats(std::vector<int> &samples)
{
	size_t sample_size = samples.size();

	double mean = std::accumulate(samples.begin(), samples.end(), 0.0) / sample_size;

	double variance = std::accumulate(samples.begin(), samples.end(), 0.0,
									  [mean](double sum, int value)
									  {
										  return sum + (value - mean) * (value - mean);
									  }) /
					  sample_size;

	double standard_deviation = sqrt(variance);

	return std::make_pair(mean, standard_deviation);
}

bool write_measurements_to_file(std::vector<int> &measurements, std::string filename)
{
	std::ofstream file(filename);
	if (!file)
	{
		return false;
	}
	for (int measurement : measurements)
	{
		file << measurement << "," << std::endl;
	}
	return true;
}

void *posix_thread_function(void *arg)
{
	auto measurements = perform_measurements(TOTAL_MEASUREMENTS);

	auto stats = calculate_stats(measurements);

	printf("Mean: %f microseconds, std: %f microseconds\n", stats.first, stats.second);

	write_measurements_to_file(measurements, "posix_results.csv");

	return NULL;
}

void *evl_thread_function(void *arg)
{
	auto measurements = perform_evl_measurements(TOTAL_MEASUREMENTS);

	auto stats = calculate_stats(measurements);

	evl_printf("Mean: %f microseconds, std: %f microseconds\n", stats.first, stats.second);

	write_measurements_to_file(measurements, "evl_results.csv");

	return NULL;
}

int main()
{
	pthread_t posix_thread_id, evl_thread_id;
	pthread_attr_t thread_attributes;
	struct sched_param thread_schedule_parameters;
	cpu_set_t cpuset;

	// Set thread attributes
	pthread_attr_init(&thread_attributes);
	pthread_attr_setinheritsched(&thread_attributes, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&thread_attributes, SCHED_FIFO);
	thread_schedule_parameters.sched_priority = 99;
	pthread_attr_setschedparam(&thread_attributes, &thread_schedule_parameters);
	CPU_ZERO(&cpuset);
	CPU_SET(1, &cpuset);

	printf("Starting posix benchmark\n");
	// Create a POSIX thread
	pthread_create(&posix_thread_id, &thread_attributes, posix_thread_function, NULL);

	// And set it's CPU affinity
	if (pthread_setaffinity_np(posix_thread_id, sizeof(cpu_set_t), &cpuset) != 0)
	{
		perror("Failed to set posix thread affinity");
		exit(0);
	}

	// Wait for the thread to finish
	pthread_join(posix_thread_id, NULL);

	printf("\nStarting evl benchmark\n");
	// Create EVL thread
	pthread_create(&evl_thread_id, &thread_attributes, evl_thread_function, NULL);
	// And set it's CPU affinity
	if (pthread_setaffinity_np(evl_thread_id, sizeof(cpu_set_t), &cpuset) != 0)
	{
		perror("Failed to set evl thread affinity");
		exit(0);
	}

	// Wait for the thread to finish
	pthread_join(evl_thread_id, NULL);

	return 0;
}