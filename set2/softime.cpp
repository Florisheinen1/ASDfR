#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <cmath>

#define NS2S 1000000000

// Thread function to perform timed execution
void* prog_thread(void* arg) {
    struct timespec period, start_time, end_time, next;
    period.tv_sec = 0;
    period.tv_nsec = 1000000; // Convert milliseconds to nanoseconds

    // Get current time
    clock_gettime(CLOCK_MONOTONIC, &next); // or CLOCK_REALTIME
    int i = 0;
    long final_time;
    long list[10000];
    while (i<10000) {
        clock_gettime(CLOCK_MONOTONIC, &start_time);
        next.tv_sec += (next.tv_nsec + period.tv_nsec) / NS2S;
        next.tv_nsec = (next.tv_nsec + period.tv_nsec) % NS2S;

        // should approx take little less than 1 ms
        for(int j=0;j<=50000;j++){
          j*j*+j/j-j%j;
        } 

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL); // or CLOCK_REALTIME
        clock_gettime(CLOCK_MONOTONIC, &end_time);

        final_time = (end_time.tv_nsec - start_time.tv_nsec) + (end_time.tv_sec - start_time.tv_sec) * NS2S;
        final_time -=  1000000;
        list[i] = final_time;
        i++;
    }
    printf("The final time is: %ld ns\n", final_time);
    std::ofstream outFile("list_normal.csv");
    
    for (int i = 1; i < 10000; ++i) {
        outFile << list[i];
        if (i != 9999) 
            outFile << ","; // Avoid adding a comma after the last number
    }
    
    outFile.close(); // Close the file stream
    return NULL;
}

int main() {
    pthread_t thread_id;

    // Create a POSIX thread
    pthread_create(&thread_id, NULL, prog_thread, NULL);

    // Wait for the thread to finish
    pthread_join(thread_id, NULL);

    return 0;
}