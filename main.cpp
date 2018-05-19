/*----------------------------------*/
/*  Author:     Francesco Sasso     */
/*  Date:       03 May 2018         */
/*----------------------------------*/

/* LIBRARIES */
#include <allegro.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>
#include <random>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <typeinfo>
#include <unistd.h>

/* NAMESPACES */
using namespace std;
using namespace Eigen;

/* CONSTANTS */
#define COL_CYAN                3
#define COL_GRAY                7
#define COL_GREEN               10
#define COL_RED                 4

#define DISPLAY_PERIOD          10000000 // 0.01 [sec] (given in nanosecond)
#define KEYBOARD_PERIOD         10000000 // 0.01 [sec] (given in nanosecond)

#define KALMAN_PERIOD           10000000 // 0.01 [sec] (given in nanosecond)
#define FRAME_PER_SECOND        100      // (1 / 0.01) [1 / sec]

#define MAX_NOISE               10
#define MIN_NOISE               1

#define MAX_TRAIL_LENGTH        150
#define MIN_TRAIL_LENGTH        20

#define MAX_PREDICT_AMOUNT      1.5     // 1.5 [sec]
#define MIN_PREDICT_AMOUNT      0.1     // 0.1 [sec]

#define MEASUREMENT_LENGHT      20

#define POINT_RADIUS            2

#define	NUM_TASKS	            3

#define WINDOW_HEIGHT           480
#define WINDOW_WIDTH            640

/* GLOBAL VARIABLES */
char s[100];
float measurement[MEASUREMENT_LENGHT][2];
float predict_amount;
int trail[MAX_TRAIL_LENGTH][2];
int prediction[2];
int exit_flag;
int prediction_flag;
int noise; 
int trail_length;

struct timespec t0;

Matrix4f A, B, H, Q, R, I;
Vector4f c;

Matrix4f P;
Vector4f x;
Vector4f prediction_x;

pthread_t _tid[NUM_TASKS];

struct task_par {
    struct timespec period;	    // task's period
    struct timespec next_act;   // next activation time
};
struct task_par	_tp[NUM_TASKS];

/* UTILITY FUNCTIONS */
// Initialize thread parameters and creates a thread
int task_create(int i, void* (*task)(void *), int period) {
    _tp[i].period.tv_sec = 0;
	_tp[i].period.tv_nsec = period;
	return pthread_create(&_tid[i], NULL, task, (void*)(&_tp[i]));
}

// Suspends the calling task until the task with index i is terminated	
void wait_for_task_end(int i) {
	pthread_join(_tid[i], NULL);
}

// Add timespec t2 to timespec t1
void timespec_add(struct timespec *t1, struct timespec *t2) {
    t1->tv_sec += t2->tv_sec;
    t1->tv_nsec += t2->tv_nsec;
    if (t1->tv_nsec >= 1000000000) { 
        t1->tv_sec++; 
        t1->tv_nsec -= 1000000000;
    }
}

// Update the next action variable and wait for the next period
void wait_for_period(struct timespec *next_act, struct timespec *period) {
    timespec_add(next_act, period);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next_act, NULL);
}

// Read the key pressed by the user
void get_keycodes(char *scan, char *ascii) {
    int k;
    k = readkey();  // block until a key is pressed
    *ascii = k;     // get ascii code
    *scan = k >> 8; // get scan code
}

// Initialize all the global variables and the allegro's window
void init() {

    /* Initialize the trail's length, the noise, the exit flag and the prediction flag */
    trail_length = MAX_TRAIL_LENGTH;
    noise = MIN_NOISE;
    exit_flag = 1;
    prediction_flag = 1;
    predict_amount = 0.5;

    // Kalman filter's parameters
    A << 1, 0, .2, 0,
    0, 1, 0, .2,
    0, 0, 1, 0,
    0, 0, 0, 1;
    
    B << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;
    
    H << 1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 0, 0,
    0, 0, 0, 0;
    
    Q << 0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, .1, 0,
    0, 0, 0, .1;
    
    R << .1, 0, 0, 0,
    0, .1, 0, 0,
    0, 0, .1, 0,
    0, 0, 0, .1;

    I << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;

    c << 0, 0, 0, 0;

    // Kalman filter's inputs
    x << WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2, 0, 0;

    prediction_x = x;

    P << 0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0;

    // Initialize the trail, the measurements, and the prediction
    for (int i = 0; i < MAX_TRAIL_LENGTH; i++) {
        trail[i][0] = (int) WINDOW_WIDTH / 2;
        trail[i][1] = (int) WINDOW_HEIGHT / 2;
    }


    for (int i = 0; i < MEASUREMENT_LENGHT; i++) {
        measurement[i][0] = (int) WINDOW_WIDTH / 2;
        measurement[i][1] = (int) WINDOW_HEIGHT / 2;
    }

    prediction[0] = trail[MAX_TRAIL_LENGTH - 1][0];
    prediction[1] = trail[MAX_TRAIL_LENGTH - 1][1];

    // Initialize the allegro's window
    allegro_init();
    install_keyboard();
    install_mouse();
    
    set_color_depth(8);
    set_gfx_mode(GFX_AUTODETECT_WINDOWED, WINDOW_WIDTH, WINDOW_HEIGHT, 0, 0);

    clock_gettime(CLOCK_MONOTONIC, &t0);

}

// Implement the Kalman's filter algorithm
void kalman_iterate(Vector4f c, Vector4f m) {
    Matrix4f K;

    // Prediction step
    x = A * x + B * c;
    P = A * P * A.transpose() + Q;

    // Correction step (as in the link provided in the project assignment)
    K = P * H.transpose() * ( H * P * H.transpose() + R ).inverse();
    x = x + K * (m - H * x);
    P = ( I - K * H ) * P;

    // If prediction is enabled, the prediction step is looped for FRAME_PER_SECOND * PREDICT_AMOUNT frames
    if (prediction_flag == 1) {
        prediction_x = x;
        for (int i = 1; i < (int) FRAME_PER_SECOND * predict_amount; i++) {
            prediction_x = A * prediction_x + B * c;
        }
    }

}

/* ROUTINES */
// Manage the display
void *display_task(void *arg_in) {
    BITMAP *buffer; 
    struct task_par *input;

    //Create the buffer
    buffer = create_bitmap(WINDOW_WIDTH, WINDOW_HEIGHT);

    // Get the inputs and initialize the next action time
    input = (struct task_par *) arg_in;
    input->next_act = t0;

    do {
        // exit the program if exit_flag is 0
        if (exit_flag == 0) {
            exit(0);
        }

        // clear the buffer
        clear_to_color(buffer, 0);

        // show the measurements and some info messages related to it
        for (int i = 0; i < MEASUREMENT_LENGHT; i++) {
            circlefill(buffer, (int) measurement[i][0], (int) measurement[i][1], POINT_RADIUS, COL_GRAY);
        }

        sprintf(s, "noise: %i (press KEY_UP/KEY_DOWN to increase/decrease it).", noise);
        textout_ex(buffer, font, s, 5, 24, COL_CYAN, -1);

        // show the trail and, eventually, the prediction, with related info messages
        for (int i = MAX_TRAIL_LENGTH - trail_length; i < MAX_TRAIL_LENGTH - 1; i++) {
            line(buffer, trail[i][0], trail[i][1], trail[i + 1][0], trail[i + 1][1], COL_GREEN);
        }

        if (prediction_flag == 1) {
            line(buffer, trail[MAX_TRAIL_LENGTH - 1][0], trail[MAX_TRAIL_LENGTH - 1][1], prediction[0], prediction[1], COL_RED);
            sprintf(s, "prediction enabled (press KEY_SPACE to disable it).");
            textout_ex(buffer, font, s, 5, 40, COL_CYAN, -1);
            sprintf(s, "prediction step: %.1f (press S/A to increase/decrease the prediction step).", predict_amount);
            textout_ex(buffer, font, s, 5, 56, COL_CYAN, -1);
        } else {
            sprintf(s, "prediction disabled (press KEY_SPACE to enable it).");
            textout_ex(buffer, font, s, 5, 40, COL_CYAN, -1);
        }

        sprintf(s, "trail's length: %i (press KEY_LEFT/KEY_RIGHT to increase/decrease it).", trail_length);
        textout_ex(buffer, font, s, 5, 8, COL_CYAN, -1);
        
        // color the screen
        blit(buffer, screen, 0, 0, 0, 0,  WINDOW_WIDTH, WINDOW_HEIGHT);

        // update the next action variable and wait for the next period
        wait_for_period(&(input->next_act), &(input->period));
        
    } while (exit_flag == 1);
    destroy_bitmap(buffer);
    
    pthread_exit(NULL);
    
}

// Manage the Kalman's filter algorithm
void *kalman_task(void *arg_in) {
    struct task_par *input;
    default_random_engine generator;
    Vector4f m;

    // Get the inputs and initialize the next action time
    input = (struct task_par *) arg_in;
    input->next_act = t0;

    do {
        // get the last measurement and update the vector of measurements accordingly
        for (int i = 0; i < MEASUREMENT_LENGHT - 1; i++) {
            measurement[i][0] = measurement[i + 1][0];
            measurement[i][1] = measurement[i + 1][1];
        }
        normal_distribution<double> distribution(0, noise);
        measurement[MEASUREMENT_LENGHT - 1][0] = mouse_x + distribution(generator);
        measurement[MEASUREMENT_LENGHT - 1][1] = mouse_y + distribution(generator);

        // update the trail by means of the Kalman's filter iterate
        for (int i = 0; i < MAX_TRAIL_LENGTH - 1; i++) {
            trail[i][0] = trail[i + 1][0];
            trail[i][1] = trail[i + 1][1];
        }

        m(0) = measurement[MEASUREMENT_LENGHT - 1][0];
        m(1) = measurement[MEASUREMENT_LENGHT - 1][1];
        m(2) = (float) (m(0) - measurement[MEASUREMENT_LENGHT - 2][0]) * 1000000000 / KALMAN_PERIOD;
        m(3) = (float) (m(1) - measurement[MEASUREMENT_LENGHT - 2][1]) * 1000000000 / KALMAN_PERIOD;

        kalman_iterate(c, m);

        trail[MAX_TRAIL_LENGTH - 1][0] = (int) x(0);    
        trail[MAX_TRAIL_LENGTH - 1][1] = (int) x(1);

        if (prediction_flag == 1) {
            prediction[0] = (int) prediction_x(0);
            prediction[1] = (int) prediction_x(1);
        }

        // update the next action variable and wait for the next period
        wait_for_period(&(input->next_act), &(input->period));

    } while (exit_flag == 1); 
    pthread_exit(NULL);
    
}

// Manage the keyboard
void *keyboard_task(void *arg_in) {
    struct task_par *input;
    char scan, ascii;

    // Get the inputs and initialize the next action time
    input = (struct task_par *) arg_in;
    input->next_act = t0;

    do {
        // get the key pressed by the user
        get_keycodes(&scan, &ascii);
        // switch the key pressed
        switch (scan) {
            case 'T': // KEY_UP: increase the noise
                if (noise < MAX_NOISE) {
                    noise++;
                }
                break;
            case 'U': // KEY_DOWN: decrease the noise
                if (noise > MIN_NOISE) {
                    noise--;
                }
                break;
            case 'S': // KEY_RIGHT: increase the trail's length
                if (trail_length < MAX_TRAIL_LENGTH) {
                    trail_length++;
                }
                break;
            case 'R': // KEY_LEFT: decrease the trail's length
                if (trail_length > MIN_TRAIL_LENGTH) {
                    trail_length--;
                }
                break;
            case 'K': // KEY_SPACE: disable/enable the prediction flag
                prediction_flag = - prediction_flag;
                break;
            case ';': // KEY_ESC: exit the program
                exit_flag = - exit_flag;
                break;
        }
        switch (ascii) {
            case 's': // S KEY: increase the prediction step
                if (predict_amount < MAX_PREDICT_AMOUNT) {
                    predict_amount = predict_amount + 0.1;
                }
                break;
            case 'a': // A KEY: decrease the prediction step
                if (predict_amount > MIN_PREDICT_AMOUNT) {
                    predict_amount = predict_amount - 0.1;
                }
                break;
        }
        
        // update the next action variable and wait for the next period
        wait_for_period(&(input->next_act), &(input->period));
        
    } while (exit_flag == 1); 
    pthread_exit(NULL);
    
}

/* MAIN */
int main(){

    // initialize the program
    init();

    // create the tasks
    task_create(0, display_task, DISPLAY_PERIOD);
    task_create(1, keyboard_task, KEYBOARD_PERIOD);
    task_create(2, kalman_task, KALMAN_PERIOD);

    // wait for the termination of the tasks
    for (int i = 0; i < NUM_TASKS; i++) wait_for_task_end(i);
    
    // close allegro's window
    allegro_exit(); 
    
    return 0;
}
