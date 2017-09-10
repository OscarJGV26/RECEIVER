#include "RECEIVER.h"
#include "Timing.h"
#include <stdio.h>
#include <unistd.h>
#include <math.h>

RECEIVER receiver;

int main(int argc, char *argv[]){
    receiver.begin(0);
    double sampling_time;
    sscanf(argv[1],"%lf",&sampling_time);
    timer_begin(sampling_time);
    int count=0;
    double sampling_time_precision=0.001;  
    double writting_time=0;
    double avg_writting_time=0;
    double sum_writting_time=0;
    double period_difference;
    long i=0;
    while(1){
        while(execute==0);
        i++;
        rt_clock_begin=real_time_clock_begin.tv_sec+(double)real_time_clock_begin.tv_nsec/1000000000.0f;
        receiver.read();
        rt_clock_end=real_time_clock_end.tv_sec+(double)real_time_clock_end.tv_nsec/1000000000.0f;
        period_difference=real_period-sampling_time;
        if(period_difference>sampling_time_precision||period_difference<-sampling_time_precision){
            printf("\nDifference detected = %10.10f\n",period_difference);
            count++;
        }
        writting_time=rt_clock_end-rt_clock_begin;
        sum_writting_time+=writting_time;
        avg_writting_time=sum_writting_time/i;
        printf("Printing..., Real Period = %lf, Average Writting Time = %lf, Detected sampling time violations = %d \r", real_period, avg_writting_time,count-2);
        sleep(1);
    }
    printf("Program Terminated!");
}
