#include "Group15.hpp"

Group15::Group15(uint write_decimator_freq, uint monitor_freq) :
    XenoFrt20Sim(write_decimator_freq, monitor_freq, file, &data_to_be_logged),
    file(1,"/home/asdfr-15/logs/log","bin"), controller()
{
     printf("%s: Constructing rampio\n", __FUNCTION__);
    // Add variables to logger to be logged, has to be done before you can log data
    logger.addVariable("this_is_a_int", integer);
    logger.addVariable("this_is_a_double", double_);
    logger.addVariable("this_is_a_float", float_);
    logger.addVariable("this_is_a_char", character);
    logger.addVariable("this_is_a_bool", boolean);
    
    // To infinite run the controller, uncomment line below
    controller.SetFinishTime(0.0);
}

Group15::~Group15()
{
    
}

int Group15::initialising()
{
    // Set physical and cyber system up for use in a 
    // Return 1 to go to initialised state

    monitor.printf("Hello from initialising\n");      // Do something

    // The logger has to be initialised at only once
    logger.initialise();
    // The FPGA has to be initialised at least once
    ico_io.init();

    return 1;
}

int Group15::initialised()
{
    // Keep the physical syste in a state to be used in the run state
    // Call start() or return 1 to go to run state

    monitor.printf("Hello from initialised\n");       // Do something

    return 0;
}

int Group15::run()
{
    // Do what you need to do
    // Return 1 to go to stopping state

    // Start logger
    logger.start();                             
    monitor.printf("Hello from run\n");  
    //  Change some data for logger            
    data_to_be_logged.this_is_a_bool = !data_to_be_logged.this_is_a_bool;
    data_to_be_logged.this_is_a_int++;
    if(data_to_be_logged.this_is_a_char == 'R')
        data_to_be_logged.this_is_a_char = 'A';
    else if (data_to_be_logged.this_is_a_char == 'A')
        data_to_be_logged.this_is_a_char = 'M';
    else
        data_to_be_logged.this_is_a_char = 'R';
    data_to_be_logged.this_is_a_float = data_to_be_logged.this_is_a_float/2;
    data_to_be_logged.this_is_a_double = data_to_be_logged.this_is_a_double/4; 

    controller.Calculate(u, y);
    if(controller.IsFinished())
        return 1;


    return 0;
}

int Group15::stopping()
{
    // Bring the physical system to a stop and set it in a state that the system can be deactivated
    // Return 1 to go to stopped state
    logger.stop();                                // Stop logger
    monitor.printf("Hello from stopping\n");          // Do something

    return 1;
}

int Group15::stopped()
{
    // A steady state in which the system can be deactivated whitout harming the physical system

    monitor.printf("Hello from stopping\n");          // Do something

    return 0;
}

int Group15::pausing()
{
    // Bring the physical system to a stop as fast as possible without causing harm to the physical system

    monitor.printf("Hello from pausing\n");           // Do something
    return 1 ;
}

int Group15::paused()
{
    // Keep the physical system in the current physical state

    monitor.printf("Hello from paused\n");            // Do something
    return 0;
}

int Group15::error()
{
    // Error detected in the system 
    // Can go to error if the previous state returns 1 from every other state function but initialising 

    monitor.printf("Hello from error\n");             // Do something

    return 0;
}
