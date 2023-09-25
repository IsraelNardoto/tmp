
# Program with Astro

## Contents

1. [Introduction](#introduction)

2. [CARMEN Style Guide](#carmen-style-guide)

    2.1. [The Prime Directive](#the-prime-directive)

    2.2. [Units and Coordinates](#units-and-coordinates)

    2.3. [Naming Conventions](#naming-conventions)

    2.4. [Memory Management, System Calls, & Functions with Side Effects](#memory-management-system-calls-functions-with-side-effects)

    2.5. [IPC](#ipc)

    2.6. [Graphics](#graphics)

3. [Getting Data from Carmen](#getting-data-from-carmen)

    3.1. [Sensor Data from the Base](#sensor-data-from-the-base)

    3.2. [Map-based Navigation Messages](#map-based-navigation-messages)

4. [Commanding the Robot](#commanding-the-robot)

    4.1. [Moving the Robot](#moving-the-robot)

    4.2. [Initializing Localize](#initializing-localize)

    4.3. [Autonomous Motion](#autonomous-motion)

5. [Getting Maps](#getting-maps)

    5.1. [Grid Maps](#grid-maps)

    5.2. [Map Placelists](#map-placelists)

    5.3. [Off Limits Regions](#off-limits-regions)

    5.4. [Navigator Maps](#navigator-maps)

6. [Getting Parameters](#getting-parameters)

    6.1. [Subscribing to Changes](#subscribing-to-changes)

    6.2. [Map Placelists](#map-placelists)

    6.3. [The Parameter Factory](#the-parameter-factory)

    6.4. [Specifying Parameters from the Command Line](#specifying-parameters-from-the-command-line)



## <a name="introduction"></a>1. Introduction

This document is designed to get CARMEN users started in writing new programs for integration into the program set. Because of the diverse skills and habits of programmers, the first section is a Style Guide to ensure compatability and clarity of new programs. The next secions describe the commands for getting information and issuing commands within CARMEN.

## <a name="carmen-style-guide"></a>2. CARMEN Style Guide

### <a name="the-prime-directive"></a>2.1. The Prime Directive

You are not the only person who will ever have to read, understand and modify your code.

### <a name="units-and-coordinates"></a>2.2. Units and Coordinates

- Always represent all units in MKS. All distances are always in metres. All angles are always, always in radians. 

- All floating point numbers should be doubles, not floats, and all fixed point numbers should be ints, not chars or shorts. 

- The only known exceptions are large, low-precision data chunks, i.e. laser data and maps.All co-ordinate frames, internal and external, are right-handed. 

- This means that q always increases counter-clockwise, from positive x to positive y. This is the opposite of screen graphics.

- q = 0 always points along the positive x axis. 

- There are exactly three allowable co-ordinate frames.

    - The robot's frame of reference. Distances are in metres, and the robot always faces along the positive x axis.

    - The global frame of reference. Distances are in metres, and q = 0 is with respect to a map. This is a meaningless frame of reference without a map. 
    
    - The map frame of reference. Distances are in grid cells, and q = 0 is with respect to a map. This is a meaningless frame of reference without a map. 
    
- Never convert between radians and degrees yourself. Always use:

        carmen_radians_to_degrees 

    and

        carmen_degrees_to_radians

- Angles are always between -p and p. Never normalize angles yourself. Always use:
    
        carmen_normalize_theta

- Never use asin, acos or atan to recover angles distances. Always use atan2 (3).

        theta = atan2(y, x);

- should always be used instead of:

        theta = atan(y/x);

- If you need the hypotenuse of something, use hypot (3) - do not take the sum of squares and find the square root. hypot (3) should be used for code clarity.

- Try not to invent your own data structures. Use one of:

        carmen_point_t
        carmen_traj_point_t
        carmen_map_point_t
        carmen_world_point_t


    making sure that you use the right data structure to store the right kind of data.

- When converting between coordinate frames, use the helper functions in:

         map_interface.h 
         
    and 
         
         global.h

- When drawing to the screen, do not maintain internal representations of data in screen co-ordinates. Convert to screen co-ordinates only immediately before calling extern graphics functions. Use the helper functions in global_graphics.h.

### <a name="naming_conventions"></a>2.3. Naming Conventions

- The most important naming convention is to expect that your code could be converted into a library one day. Therefore, it is important to think about global name spaces.

- As many functions and global variables should be static as possible. In general, try to avoid global variables.

- Static global variables with accessor functions are preferable to non-static globals.

- Any non-static functions and global variables must have the prepend carmen_{module-name}. e.g., carmen_base_subscribe_odometry.


### <a name="memory_management_system_calls_Functions_with_side_effects"></a>2.4. Memory Management, System Calls, & Functions with Side Effects

- Never use a system call without checking the return value. This includes any memory allocation. We have provided a function carmen_test_alloc that facilitates memory checking. CVS will not allow code to be committed to core CARMEN that contains a call to malloc/calloc/realloc and does not have a call to carmen_test_alloc on the subsequent line.

- When using statically allocated arrays, especially strings, never make the array ``just big enough''.


        Wrong: char buffer[11] for a string of length 10.

        Right: char buffer[1024] for a string of length 10.


    Why? You minimize the probability of off-by-one errors writing into memory you don't own. Memory is cheap. If you really need to create ``just-big-enough'' memory arrays because you're running out of memory, you're solving the wrong problem.

- Never use fscanf, gets, etc to read into buffers without limit.

- Never make a system call, and check its side effect in one step. For example:


        fd = open(filename, O_RDONLY);
             if (fd < 0)
             return -1;

    should always be used instead of:

        if ((fd = open(filename, O_RDONLY)) < 0)
            return -1;

    This is for two reasons:

    - Code clarity : it is easier to read the former than the latter.
Debugging : it is easier to use a debugger with the former than the latter.

    - Avoid macros. If you must have a macro, write an inline function instead. Macros can hurt you in the following manner:

            #define MAX(X, Y) ((X) > (Y) ? (X) : (Y))
             ...
            max_x = MAX(X++, Y++);

- The larger of the two fields will be incremented twice. Instead, use:

        inline int max_int(int x, int y) {
            3return (x > y ? x : y);
        }

    Under compiler flags -O, these two code fragments are compiled identically under gcc, but the macro has unexpected side-effects.

- Never generate random numbers yourself. You will do it wrong. Always use one of:

        carmen_int_random, 
        carmen_uniform_random 
        carmen_gaussian_random

    - Consult the documentation for these functions to find out which random algorithm they use.

- Never create random number seeds yourself. You will do it wrong. Always use:

        carmen_randomize

    This function randomizes by reading a seed from /dev/random.


### <a name="ipc"></a>2.5. IPC


- Do not initialize IPC yourself. Use:

        carmen_ipc_initialize

- If you write a stand-alone module, there should be three separate files with your module:

        {module}_messages.h 
        
    this contains the IPC message definitions.

        {module}_interface.h  
        
    this contains the function prototypes for communicating with your module.

        {module}_interface.c 
    
    this contains the actual functions for communicating with your module. This file should be compiled into a library.

    We expect that all modules will have well-defined interface libraries that relieve the user of having to worry about marshalling data.

- Every IPC message should have a timestamp and hostname field. The hostname should be 10 chars long, and should be a canonical representation of the machine on which the process is running that created the message. Use the helper function:

        carmen_get_tenchar_host_name() 
    to generate the hostname.

- The timestamp should also reflect the time of creation of the data, not the time the message was published. For instance, the laser message timestamp is the time the data was read from the serial port. Use the helper function:
        
        carmen_get_time_ms() 
    to generate the timestamp as a double.


### <a name="graphics"></a>2.6. Graphics

- Keep graphics and robot functionality in separate processes.

    Notice that none of the core robot functionality (**base_services, robot, navigator, localize**) link against graphics libraries. This is for multiple reasons:

    1. It increases the likelihood that your code will work on a standalone robot in the absence of a network. If your process is displaying output to a remote X window, and the network drops out, your process will wedge until the network comes back. If, however, your process is sending IPC messages to an external display process, then only the display process will wedge, not your (presumably more important) robot process.

    2. You are removed from the temptation of displaying internal information not accessible to external code. If you want to see the state of your process, the odds are very high that eventually, someone else will want to use the state of your process. By forcing you to write an IPC message and interface handler for that information just to get it into the display, you make life much easier for people who come after you.

    3. Allowing your code to compile without graphics makes it more portable, and makes it more distributable, in the sense that it can run anywhere - it is not constrained by the speed of network connections between machines.

    For obvious reasons, programs like map_editor and mapper have displays compiled in - these programs are not intended for autonomous use on the robot.

### <a name="getting-data-from-carmen"></a>3. Getting Data from Carmen

Sensor data currently comes from one of two sources: the base module (such as scout, pioneer, etc.) provides raw odometry data and may provide sonar data, bumper data and infra-red (IR) data. The laser module may provide laser data.

#### 3.0.1. Subscribing

All of the following messages can be subscribed to by using helper functions in the appropriate xxx_interface library, e.g., base_interface. The helper functions are all of the form:

    void 
    carmen_robot_subscribe_xxxx_message(carmen_robot_xxx_message *msg,
                                    carmen_handler_t handler, 
                                        carmen_subscribe_t subscribe_how);

where carmen_handler_t and carmen_subscribe_t are defined as

    typedef enum {CARMEN_UNSUBSCRIBE, CARMEN_SUBSCRIBE_LATEST, 
                CARMEN_SUBSCRIBE_ALL} carmen_subscribe_t;
    typedef void (*carmen_handler_t)(void *);

If the msg field of the subscribe function is NULL, then a static message is automatically allocated and returned as the argument to handler, otherwise the message pointer passed to the subscribe function is always used. In all cases, the same memory is re-used for all handled messages of the same message name, and passed as an argument to the handler function.

If the handler field of the subscribe function is NULL, no handler is called, but the memory pointed to by msg is updated regularly. If both handler and msg are both NULL, your program will spend a fair chunk of time doing nothing useful for you.

The subscribe_how field allows the user to either unsubscribe, or to start a new subscription. Subscribing only to the latest message allows the module to fall behind in processing messages without serious consequences. It should be pointed out that subscribing to all messages (CARMEN_SUBSCRIBE_ALL) does not guarantee all messages. Currently, the upper limit for the queue size is 1000 messages. If an IPC process actually subscribes to all messages and falls seriously behind (or wedges), central can run out of memory, or even worse, the TCP stack can overflow. Consequently, the Carmen subscription functions limit the maximum message queue size to 1000. A resourceful programmer can increase this queue (or even remove the queue size), but it is not clear this would ever be necessary.

#### 3.0.2. Requesting Data Explicitly

Some of these messages can also be obtained using explicit queries. To date, the only robot data that can be obtained using queries are from localize and navigator. Specifically, carmen_localize_globalpos_message, carmen_localize_particle_message, carmen_navigator_status_message and carmen_navigator_plan_message can all be obtained using specific query interface functions, which return the appropriate messages directly.

These functions create new memory every time they return successfully; consequently, they should be used with care.

3.1  Sensor Data from the Base

### <a name="sensor-data-from-the-base"></a>3.1. Sensor Data from the Base

The timestamp field in all messages is defined to be the time when the data was first created or acquired (e.g, by scout or simulator).

#### 3.1.1  Odometry

    typedef struct {
    double timestamp;
    char *host;
    double x, y, theta;
    double tv, rv;
    double acceleration;
    } carmen_base_odometry_message;

    void 
    carmen_base_subscribe_odometry_message(carmen_base_odometry_message *odometry,
                                        carmen_handler_t handler,
                                        carmen_subscribe_t subscribe_how);

The x, y, theta fields are the raw odometry, from the time the robot was turned on. The tv and rv fields are the translational and rotational velocities of the robot. For robots that have differential drive (as opposed to synchrodrive), these velocities are computed from the left and right wheel velocities that base actual uses.

#### 3.1.2  Sonar

Sonar sensing is not properly supported by Carmen right now, and so subscribing to carmen_base_sonar_message messages may sometimes not work properly. But, if you care, it looks like:



    typedef struct {
    double timestamp;
    char *host;
    int num_sonars;
    double sensor_angle; //width of sonar cone
    double *range;
    carmen_point_p positions;
    } carmen_base_sonar_message;


    typedef struct {
    double timestamp;
    char *host;
    int rate;
    int num_sonars;
    int *order;
    carmen_point_t *sonar_offsets;
    } carmen_base_sonar_conf_message;


    void carmen_base_subscribe_sonar_message(carmen_base_sonar_message *sonar,
                                            carmen_handler_t handler,
                                            carmen_subscribe_t subscribe_how);

The sonar message reports a recent set of sonar range data from the base. There should be as many range points and offset points as described by num_sonars. The sonar_offsets describes the physical location and orientation of each transducer from the centre of the robot.

There is currently no way to query the firing rate or order of the sonar transducers, and the sonar_conf message is not yet supported. (Or even defined by any module.)

#### 3.1.3  Bumper and IR sensors

    typedef struct {
    double timestamp;
    char *host;
    int num_bumpers;
    unsigned char *state;
    } carmen_base_bumper_message;

    typedef struct {
    double timestamp;
    char *host;
    int num_irs;
    double *range;
    } carmen_base_ir_message;

    typedef struct {
    double timestamp;
    char *host;
    int history;
    int num_irs;
    int order[32];
    } carmen_base_ir_conf_message;

These messages will probably change, and are not yet supported.

#### 3.1.4  Laser data

Laser data is defined as a set of ranges, of number given by the num_readings field, contained in range. It furthermore provides remission values (if available) using num_remissions and the field remission. The order of the values is right-handed (counter-clockwise). The configuration of the laser is privided by

    typedef struct {

    /* what kind of laser is this */
    carmen_laser_laser_type_t  laser_type;  

    /* angle of the first beam relative to to the center of the laser */
    double start_angle;                     

    /* field of view of the laser */
    double fov;                             

    /* angular resolution of the laser */
    double angular_resolution;              

    /* the maximum valid range of a measurement  */
    double maximum_range;                   

    /* error in the range measurements*/
    double accuracy;                        

    /* if and what kind of remission values are used */	
    carmen_laser_remission_type_t remission_mode;  
    } carmen_laser_laser_config_t;

All these values are given relative to the heading of the laser.

The timestamp field in all messages is defined to be the time when the data was first created or acquired (e.g, by laser or simulator), not the timestamp of some intermediate process (such as the correction applied by robot when applying odometry interpolation and correction). Similarly, the hostfield is defined to be the hostname associated with the origin of the data, not the hostname of some intermediary converting the data from raw form to interpolated form.

    typedef struct {
    double timestamp;
    char *host;
    carmen_laser_laser_config_t config;
    int num_readings;
    float *range;
    int num_remissions;
    float *remission;
    } carmen_laser_laser_message;

    typedef enum {SICK_LMS, SICK_PLS, HOKUYO_URG, 
                SIMULATED_LASER, UMKNOWN_PROXIMITY_SENSOR}   
    carmen_laser_laser_type_t;

    typedef enum {OFF, DIRECT, NORMALIZED}          
    carmen_laser_remission_type_t;


Carmen currently supports up to 4 laser range finder. The subscribe message are given by:

    void
    carmen_laser_subscribe_laser1_message(carmen_laser_laser_message *laser,
                        carmen_handler_t handler,
                        carmen_subscribe_t subscribe_how);

    void
    carmen_laser_subscribe_laser2_message(carmen_laser_laser_message *laser,
                        carmen_handler_t handler,
                        carmen_subscribe_t subscribe_how);

    void
    carmen_laser_subscribe_laser3_message(carmen_laser_laser_message *laser,
                        carmen_handler_t handler,
                        carmen_subscribe_t subscribe_how);

    void
    carmen_laser_subscribe_laser4_message(carmen_laser_laser_message *laser,
                        carmen_handler_t handler,
                        carmen_subscribe_t subscribe_how);

    void
    carmen_laser_subscribe_alive_message(carmen_laser_alive_message *alive,
                        carmen_handler_t handler,
                        carmen_subscribe_t subscribe_how);


This message is defined by laser and by simulator, and the same message struct is used by both carmen_laser_frontlaser and carmen_laser_rearlaser messages. As a consequence, there is no way to tell from a message itself whether or not the message is a front laser message or a rear laser message. This might be fixed in a future release.

#### 3.1.5  Robot messages

These messages are defined and emitted by robot.

- **carmen_robot_laser_message**

        typedef struct {
        double timestamp;
        char *host;
        carmen_laser_laser_config_t config;
        int num_readings;
        float *range;
        char *tooclose;
        int num_remissions;
        float *remission;
        carmen_point_t laser_pose; //position of the center of the laser
        carmen_point_t robot_pose; //position of the center of the robot
        double tv, rv;
        double forward_safety_dist, side_safety_dist;
        double turn_axis;
        } carmen_robot_laser_message;

        void carmen_robot_subscribe_frontlaser_message(carmen_robot_laser_message *laser,
                                                    carmen_handler_t handler,
                                                    carmen_subscribe_t subscribe_how);

        void carmen_robot_subscribe_rearlaser_message(carmen_robot_laser_message *laser,               
                                                    carmen_handler_t handler,
                                                    carmen_subscribe_t subscribe_how);


The carmen_robot_laser_message has raw odometry attached to it. The robot module attempts to adjust for clock skews and interpolate the true robot position of the data correctly based on time stamps. Consequently, after a carmen_laser_laser_message is received, the corresponding carmen_robot_laser_message will not be emitted until a carmen_base_odometry_message with a later timestamp is received. The odometry fields is laser_pose. This data is in fact the interpolated position of this laser, based on the laser offset parameters for this laser. The interpolated odometry for the robot itself is given in robot_pose. Consequently, front and rear laser messages with the same timestamps should have different values for laser_pose but not fot robot_pose.

The robot module is also used to perform collision avoidance, stopping the robot if the laser measurements indicate an obstacle inside safety margins. The tooclose array labels each range measurement as to whether or not it lies inside the robot safety margins. There are as many tooclose elements as there are range elements (as given by the num_readings field). If msg.tooclose[i] is 1, then the range msg.range[i] is inside the safety margin.

Like the carmen_laser_laser_message, the carmen_robot_laser_message provides also information about the configuration of the laser and remission values

The same message struct is used by both the carmen_robot_frontlaser and carmen_robot_rearlaser messages. Again, there is no way to tell from a message itself whether or not the message is a front laser message or a rear laser message. This hopefully will be fixed in a future release.

- **carmen_robot_sonar_message**

        typedef struct {
        double timestamp;
        char *host;
        int num_sonars;
        double sensor_angle;          //width of sonar cone
        double *ranges;
        carmen_point_t *positions;
        carmen_point_t robot_pose;
        double tv, rv;
        } carmen_robot_sonar_message;

The exact meaning of some these fields is a little bit of a mystery. This message is still in the experimental stage.

### <a name="map-based-navigation-messages"></a>3.2. Map-based Navigation Messages



- **carmen_localize_globalpos_message**

            typedef struct {
            double timestamp;
            char *host;
            carmen_point_t globalpos, globalpos_std;
            carmen_point_t odometrypos;
            double globalpos_xy_cov;
            int converged;
            } carmen_localize_globalpos_message;

            void 
            carmen_localize_subscribe_globalpos_message(
                        carmen_localize_globalpos_message *globalpos,
                                    carmen_handler_t handler,
                                    carmen_subscribe_t subscribe_how);

This message reports on the current robot pose estimate, given by localize.

The globalpos field is mean robot position (computed from the particle filter), and is given in the global frame of reference. (See the Carmen Style Guide.) The odometrypos is the odometry of the robot at the time the current estimate was computed. It is therefore possible to estimate the true position of the robot for a short duration after the last carmen_localize_globalpos_message by finding the relative displacement (translational and rotational) of the robot between the current odometry, and the odometry of the last carmen_localize_globalpos_message, and then adding this displacement to the globalpos field. There is a helper function in liblocalize_interface called carmen_localize_correct_odometry that does exactly this.

Notice that Carmen localize no longer explicitly provides correction parameters, but instead provides a functional way to correct odometry.

The globalpos_std gives the variances of the position estimates, s<sub>x</sub><sup>2</sup>, s<sub>y</sub><sup>2</sup>, s<sub>q</sub><sup>2</sup>. The globalpos_xy_cov field gives the covariance s<sub>x</sub><sub>y</sub>.

The converged field indicates whether localize is currently in global or tracking mode. If localize has converged (is in tracking mode) then the position estimate has high confidence. When localize believes it is lost, it switches back to global localization mode and the converged field switches to 1.

Additional messages about the state of localize are:

carmen_localize_particle_message - This message gives the full state of the pose particle filter (and people particle filters, if people tracking is running.)
carmen_localize_sensor_message - This message contains information about how localize has used each laser reading, such as whether a laser readings was integrated (or not), and which person filter the reading was assigned (if any).
carmen_localize_people_message - This message contains the current state of the person tracker, if it is running.

#### 3.2.2  Autonomous Navigation

- **carmen_navigator_plan_message**

        typedef struct {
        double timestamp;
        char *host;
        int autonomous;
        int goal_set;
        carmen_point_t goal;
        carmen_traj_point_t robot;
        } carmen_navigator_status_message;

        void 
        carmen_navigator_subscribe_status_message(carmen_navigator_status_message *status,
                                                carmen_handler_t handler,
                                                carmen_subscribe_t subscribe_how);

        int 
        carmen_navigator_query_status(carmen_navigator_status_message **status);


The autonomous field is 1 if the robot is currently trying to autonomously navigate to the goal, perhaps because a user clicked the Autonomous button in the navigatorgui display. When the robot changes to non-autonomous mode, a carmen_navigator_autonomous_stopped_message is emitted (see below), and contains the reason for changing to non-autonomous mode.

The goal_set field is 1 if the navigator has received any goal at all. If no goal has been set, then it is not possible for the navigator go into autonomous mode.

The goal field reports on what the navigator's current goal. The navigator does not (and should not ever) support multiple goal destinations.

The robot field is the navigator's estimate of the current robot position in the global reference frame (see the Carmen Style Guide). This is based on the latest estimate from localize, combined with any subsequent odometric updates the navigator has received. The robot position field reported by the navigator should never lag behind (in time) localize's estimate.

- **carmen_navigator_plan_message**

        typedef struct {
        double timestamp;
        char *host;
        carmen_traj_point_t *path;
        int path_length;
        } carmen_navigator_plan_message;


        void 
        carmen_navigator_subscribe_plan_message(carmen_navigator_plan_message *plan,
                                                carmen_handler_t handler,
                                                carmen_subscribe_t subscribe_how);

        int 
        carmen_navigator_query_plan(carmen_navigator_plan_message **plan);


If the path length is 0, then there is no path from the current robot location to the goal. The first point in the path should always be the robot's current position as reported by the carmen_navigator_status_message, and the last point in the path should always be the goal as reported by carmen_navigator_status_message.

- **carmen_navigator_autonomous_stopped_message**

        typedef struct {
        double timestamp;
        char *host;
        carmen_navigator_reason_t reason;
        } carmen_navigator_autonomous_stopped_message;

        void 
        carmen_navigator_subscribe_autonomous_stopped_message
                (carmen_navigator_autonomous_stopped_message *autonomous_stopped,
                carmen_handler_t handler,
                carmen_subscribe_t subscribe_how);

- The reason field can take one of three values:

        CARMEN_NAVIGATOR_GOAL_REACHED_v 
    
- This means that the robot has reached the goal destination (that is, is within the robot_approach_dist of the goal).

        CARMEN_NAVIGATOR_USER_STOPPED_v 
    
- This means that some other process (such as the navigatorgui) published a carmen_navigator_stop_message.

         CARMEN_NAVIGATOR_UNKNOWN_v 
    
- This means some (unknown) reason caused autonomous navigation to stop. The navigator does not currently ever emit this reason.


### <a name="commanding-the-robot"></a>4. Commanding the Robot

While it is (obviously) possible to send messages directly to the base module, this is not an exposed interface. Sending velocities directly to the base side-steps the last-mile collision avoidance module, and can also result in all kinds of pathologies as modules fight for control of the robot.

#### <a name="moving-the-robot"></a>4.1. Moving the Robot

- **carmen_robot_velocity_message**

        typedef struct {
        double timestamp;
        char *host;
        double tv, rv;
        } carmen_robot_velocity_message;

        void carmen_robot_velocity_command(double tv, double rv);

Publishing this message will tell the robot module to make the robot go at the specified translation velocity tv specified rotational velocity rv.

The robot may not be able to go at these speeds, because either they exceed the maximum velocity, or because an obstacle is too close. There is no diagnostic for the first condition yet. The second condition can be detected by examining the tooclose field of the carmen_robot_laser_message.

- **carmen_robot_vector_move_message**

        typedef struct {
        double timestamp;
        char *host;
        double distance;
        double theta;
        } carmen_robot_vector_move_message;

        void carmen_robot_move_along_vector(double distance, double theta);

Publishing this message will take advantage of a PD loop in the robot module to make the robot go to specific target point. This control loop contains no planning, so if an obstacle intervenes between the robot and the target, the robot will stop.

The target point set by this message is given by the distance and theta fields, which are in metres and radians respectively, and are relative to the robot's current pose. Consequently, a positive distance with a theta of 0 would drive the robot forward distance metres. Similarly, a distance of 0 and a theta of p would cause the robot to rotate 180°, regardless of current orientation.


#### <a name="initializing-localize"></a>4.2. Initializing Localize

    carmen_localize_initialize_message

- This message provides a way to initialize localization.


        typedef struct {
        double timestamp;
        char *host;
        int distribution;
        int num_modes;
        carmen_point_t *mean, *std;
        } carmen_localize_initialize_message;

The distribution specifies the kind of distribution to use for initialization. At the moment only one type of distribution is supported: **CARMEN_INITIALIZE_GAUSSIAN**. (The localize_messages.h file also lists a **CARMEN_INITIALIZE_UNIFORM** distribution type, but this is not currently supported by localize itself.)

The 3-dimensional point mean specifies the x, y, q mean of the gaussian, and std specified the s<sub>x</sub>, s<sub>y</sub>, s<sub>q</sub> standard deviations of the gaussian. Reasonable values for the standard deviations are (0.2m, 0.2m, 4.0°).

It is also possible to initialize localize through the navigator by using the ```carmen_navigator_set_robot```  or ```carmen_navigator_set_robot_map``` messages, but these messages are deprecated.

#### 4.2.1  Setting A Goal

This message provides a way to set the goal or destination for navigation. It is not possible (nor should it ever be possible) to set multiple goals inside the navigator.

    typedef struct {
    double timestamp;
    char *host;
    double x, y;
    } carmen_navigator_set_goal_message;

    int carmen_navigator_set_goal(double x, double y);

    typedef struct {
    double timestamp;
    char *host;
    char *placename;
    } carmen_navigator_placename_message;

    int carmen_navigator_set_goal_place(char *name);


The (x, y) fields should be self-explanatory as the goal position, in the global reference frame (see the Carmen Style Guide), as always in metres.

If the map contains place names, then it is also possible to set the goal position using a carmen_navigator_placename_message, and the carmen_navigator_set_goal_place helper function. This has no effect if the map does not contain a place name that matches.

#### <a name="autonomous-motion"></a>4.3. Autonomous Motion

These messages toggle the navigator in and out of autonomous motion.

    typedef carmen_default_message carmen_navigator_stop_message;
    typedef carmen_default_message carmen_navigator_go_message;

    int carmen_navigator_stop(void);

    int carmen_navigator_go(void);


If the robot is already at the current goal position, then the carmen_navigator_go_command will cause the navigator to change momentarily into autonomous mode, and then switch back again, emitting a ```carmen_navigator_autonomous_stopped_message``` with **CARMEN_NAVIGATOR_GOAL_REACHED_v** as the reason.

When the navigator receives a ```carmen_navigator_stop_message```, then a ```carmen_navigator_autonomous_stopped_message``` is emitted with **CARMEN_NAVIGATOR_USER_STOPPED_v** as the reason.


### <a name="getting-maps"></a>5. Getting Maps


#### <a name="grid-maps"></a>5.1. Grid Maps

    typedef carmen_default_message carmen_gridmap_request_message;

    int carmen_map_get_gridmap(carmen_map_p map);


#### <a name="map-placelists"></a>5.2. Map Placelists

    typedef struct {  
    double timestamp;
    char *host;
    carmen_place_p places;
    int num_places;
    } carmen_map_placelist_message;

    int carmen_map_get_placelist(carmen_map_placelist_p placelist);



#### <a name="off-limits-regions"></a>5.3. Off Limits Regions

    typedef struct {  
    double timestamp;
    char *host;
    carmen_offlimits_p offlimits_list;
    int list_length;
    } carmen_map_offlimits_message;

    int carmen_map_get_offlimits(carmen_offlimits_p *offlimits, int *list_length);

#### <a name="off-limits-regions"></a>5.4. Navigator Maps

    typedef struct {
    double timestamp;
    char *host;
    unsigned char *data;    
    int size;
    int compressed;
    carmen_map_config_t config;
    carmen_navigator_map_t map_type;
    } carmen_navigator_map_message;  

    int carmen_navigator_get_map(carmen_navigator_map_t map_type, 
                            carmen_navigator_map_message **map_pointer);



#### <a name="getting-parameters"></a>6. Getting Parameters 

Parameters can be acquired from the parameter server using functions in libparam_interface, eg:

    int carmen_param_get_int(char *variable, int *return_value);
    int carmen_param_get_double(char *variable, double *return_value);
    int carmen_param_get_onoff(char *variable, int *return_value);
    int carmen_param_get_string(char *variable, char **return_value);

The conversion of parameters to ints, doubles, etc. is done on demanded by the interface library. If you do not wish the library to convert the parameter to the appropriate type, simply request the parameter as a string.

If there is no definition for the parameter requested, then the library will output a warning to the terminal, unless this warning has been turned off using ```carmen_param_allow_unfound_variables(1);```.

Also, as a convience, variables can be requested either by specifying the fully qualified module_param-name name, or by first specifying a module using ```carmen_param_set_module(char *)```, and the specifying just the param-name form.



#### <a name="subscribing-to-changes"></a>6.1. Subscribing to Changes

Some processes may wish to subscribe to changes to parameters during their execution, for example changing the robot speed or acceleration profile, or changing the robotgraph display parameters. Of course, some processes should not suscribe to some parameter changes: changing the number of particles localize uses during execution would result in disaster.

Parameter changes can be subscribed using the functions below:

    void carmen_param_subscribe_int(char *module, 
                    char *variable, 
                    int *variable_address, 
                                    carmen_param_change_handler_t handler);
    void carmen_param_subscribe_double(char *module, 
                    char *variable, 
                                    double *variable_address, 
                                    carmen_param_change_handler_t handler);
    void carmen_param_subscribe_onoff(char *module, 
                    char *variable, 
                    int *variable_address, 
                                    carmen_param_change_handler_t handler);
    void carmen_param_subscribe_string(char *module, 
                    char *variable, 
                    char **variable_address, 
                                    carmen_param_change_handler_t handler);

These functions take a module and variable name as parameters. The subscription mechanism can either silently change variable values as parameters change, or can invoke a callback when a parameter is changed. If the variable_address parameter is non-NULL, then the new parameter value is stored at this address (in the case of strings, this is a pointer to some newly-malloc'd memory containing the new string definition. If the variable address is non-NULL when the parameter changes, the old memory is freed.) If the handler parameter is non-NULL, then function pointed to by handler is invoked whenever the parameter changes. If both are non-NULL, then the variable changes and then the callback invoked. If both are NULL, then the subscription mechanism does not do much.


#### <a name="the-parameter-factory"></a>6.2. The Parameter Factory

Parameters can be loaded in a single step using the parameter factory methods, much like the gtk menu item factory methods. The set of parameters to be loaded should be described in an array of carmen_param_t, and passed to

    void carmen_param_install_params(int argc, char *argv[], carmen_param_p param_list, 
                                    int num_items);

    Each parameter in the array of type carmen_param_t has the form:

    typedef struct {
    char *module;
    char *variable;
    carmen_param_type_t type;
    void *user_variable;
    int subscribe;
    carmen_param_change_handler_t handler;
    } carmen_param_t;

where module is the module name, variable is the variable name, type is one of CARMEN_PARAM_INT, CARMEN_PARAM_DOUBLE, CARMEN_PARAM_ONOFF or CARMEN_PARAM_STRING. The parameter is loaded into user_variable, whose original type should match that specified in the type field.
If subscribe is set to 1, then the process will subscribe to changes to the parameter, and set up a callback on the function specified in handler (if not NULL). The callback parameter is ignored if subscribe is set to 0, and the parameter is only loaded once. There is no way to use the parameter factory methods, subscribe to a variable and not have the variable's value updated automatically.

If carmen_param_allow_unfound_variables() is set to 0 (by default), then carmen_param_install_params will exit with an error on the first parameter absent from the parameter server, reporting what the problematic parameter is.

If a process loads its parameter set using the parameter factory methods, then running the process with the -h or -help command line option will print out a list of parameters used by that process, their expected types and whether or not the process subscribes to changes.

6.3  Specifying Parameters from the Command Line

#### <a name="specifying-parameters-from-the-command-line"></a> 6.3. Specifying Parameters from the Command Line

Parameter values can be temporarily over-ridden from the command line of a given process, for that process only. For example:

    % robot -max_t_vel 0.1

will specify a max_t_vel of 0.1 m/s for the robot process only.

This parameter specification method is not advised, is a convenience only, and may be removed from future versions of carmen. Command-line parameters are assumed to have no module name, and attempts are made to find a parameter with any module name and matching parameter name. If a process uses multiple parameters with different module names and the same parameter name, the behaviour of command-line specification is undefined.
