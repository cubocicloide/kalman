/*
Author:     Francesco Sasso
Date:       03 May 2018
*/

/* LIBRARIES */
#include <allegro.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>
#include <pthread.h>
#include <random>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <typeinfo>
#include <unistd.h>

/* CONSTANTS */
#define COL_CYAN                3
#define COL_GRAY                7
#define COL_GREEN               10
#define COL_RED                 4

#define DISPLAY_PERIOD          10000000 // 0.01 [sec] (given in nanosecond)
#define KEYBOARD_PERIOD         10000000 // 0.01 [sec] (given in nanosecond)

#define KALMAN_PERIOD           10000000 // 0.01 [sec] (given in nanosecond)
#define FRAME_PER_SECOND        100      // (1 / 0.01) [1 / sec]
#define PREDICT_AMOUNT          0.5      // 0.5 [sec]

#define MAX_NOISE               10
#define MIN_NOISE               1

#define MAX_TRAIL_LENGTH        150
#define MIN_TRAIL_LENGTH        20

#define MEASUREMENT_LENGHT      20

#define POINT_RADIUS            2

#define WINDOW_HEIGHT           480
#define WINDOW_WIDTH            640

/* NAMESPACES */
using namespace std;
using namespace Eigen;

/* GLOBAL VARIABLES */
char s[100];
float measurement[MEASUREMENT_LENGHT][2];
int trail[MAX_TRAIL_LENGTH][2];
int prediction[2];
int exit_flag;
int prediction_flag;
int noise; 
int trail_length;

pthread_mutex_t mux_measurement;
pthread_mutex_t mux_noise;
pthread_mutex_t mux_prediction;
pthread_mutex_t mux_trail;
pthread_mutex_t mux_trail_length;

pthread_t thread_display;
pthread_t thread_kalman;
pthread_t thread_keyboard;

struct timespec t0, tx0;

Matrix4f A, B, H, Q, R, I;

Matrix4f P;
Vector4f x;
Vector4f prediction_x;

/* UTILITY FUNCTIONS */

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

    // mutex 
    mux_measurement = PTHREAD_MUTEX_INITIALIZER;
    mux_noise = PTHREAD_MUTEX_INITIALIZER;
    mux_prediction = PTHREAD_MUTEX_INITIALIZER;
    mux_trail = PTHREAD_MUTEX_INITIALIZER;
    mux_trail_length = PTHREAD_MUTEX_INITIALIZER;

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
    // Correction step (as I know from the theory):
    // P = ( I - K * H ) * P * (I - K * H).transpose() + K * R * K.transpose();
    

    // If prediction is enabled, the prediction step is looped for FRAME_PER_SECOND * PREDICT_AMOUNT frames
    if (prediction_flag == 1) {
        prediction_x = x;
        for (int i = 1; i < (int) FRAME_PER_SECOND * PREDICT_AMOUNT; i++) {
            prediction_x = A * prediction_x + B * c;
        }
    }

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

/* ROUTINES */

// Manage the display
void *display_task(void *arg_in) {
    BITMAP *buffer; 
	struct timespec next_act, period;

    //Create the buffer
    buffer = create_bitmap(WINDOW_WIDTH, WINDOW_HEIGHT);

    // Initialize the period of the task and the next action time
    period.tv_sec = 0;
    period.tv_nsec = DISPLAY_PERIOD; 

    next_act = tx0;

    do {
        // exit the program if exit_flag is 0
        if (exit_flag == 0) {
            exit(0);
        }

        // clear the buffer
        clear_to_color(buffer, 0);

        // show the measurements and some info messages related to it
        pthread_mutex_lock(&mux_measurement);
            for (int i = 0; i < MEASUREMENT_LENGHT; i++) {
                circlefill(buffer, (int) measurement[i][0], (int) measurement[i][1], POINT_RADIUS, COL_GRAY);
            }

            pthread_mutex_lock(&mux_noise);
                sprintf(s, "noise: %i (press KEY_UP/KEY_DOWN to increase/decrease it).", noise);
                textout_ex(buffer, font, s, 5, 24, COL_CYAN, -1);
            pthread_mutex_unlock(&mux_noise);

        pthread_mutex_unlock(&mux_measurement);

        // show the trail and, eventually, the prediction, with related info messages
        pthread_mutex_lock(&mux_trail);
            pthread_mutex_lock(&mux_trail_length);

                for (int i = MAX_TRAIL_LENGTH - trail_length; i < MAX_TRAIL_LENGTH - 1; i++) {
                    line(buffer, trail[i][0], trail[i][1], trail[i + 1][0], trail[i + 1][1], COL_GREEN);
                }

                pthread_mutex_lock(&mux_prediction);
                    if (prediction_flag == 1) {
                        line(buffer, trail[MAX_TRAIL_LENGTH - 1][0], trail[MAX_TRAIL_LENGTH - 1][1], prediction[0], prediction[1], COL_RED);
                        sprintf(s, "prediction enabled (press KEY_SPACE to disable it).");
                        textout_ex(buffer, font, s, 5, 40, COL_CYAN, -1);
                    } else {
                        sprintf(s, "prediction disabled (press KEY_SPACE to enable it).");
                        textout_ex(buffer, font, s, 5, 40, COL_CYAN, -1);
                    }
                pthread_mutex_unlock(&mux_prediction);

                sprintf(s, "trail's length: %i (press KEY_LEFT/KEY_RIGHT to increase/decrease it).", trail_length);
                textout_ex(buffer, font, s, 5, 8, COL_CYAN, -1);

            pthread_mutex_unlock(&mux_trail_length);
        pthread_mutex_unlock(&mux_trail);
        
        // color the screen
        blit(buffer, screen, 0, 0, 0, 0,  WINDOW_WIDTH, WINDOW_HEIGHT);

        // update the next action variable and wait for the next period
        timespec_add(&next_act, &period);
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_act, NULL);
        
    } while (exit_flag == 1);
    destroy_bitmap(buffer);
    
    pthread_exit(NULL);
    
}

// Manage the Kalman's filter algorithm
void *kalman_task(void *arg_in) {
	struct timespec next_act, period;
    default_random_engine generator;

    Vector4f c, m;

    // Initialize the period of the task and the next action time
    period.tv_sec = 0;
    period.tv_nsec = KALMAN_PERIOD; 

    next_act = tx0;

    do {
        // get the last measurement and update the vector of measurements accordingly
        pthread_mutex_lock(&mux_measurement);
            for (int i = 0; i < MEASUREMENT_LENGHT - 1; i++) {
                measurement[i][0] = measurement[i + 1][0];
                measurement[i][1] = measurement[i + 1][1];
            }
            normal_distribution<double> distribution(0, noise);
            measurement[MEASUREMENT_LENGHT - 1][0] = mouse_x + distribution(generator);
            measurement[MEASUREMENT_LENGHT - 1][1] = mouse_y + distribution(generator);
        pthread_mutex_unlock(&mux_measurement);

        // update the trail by means of the Kalman's filter iterate
        pthread_mutex_lock(&mux_trail);
            for (int i = 0; i < MAX_TRAIL_LENGTH - 1; i++) {
                trail[i][0] = trail[i + 1][0];
                trail[i][1] = trail[i + 1][1];
            }

            c << 0, 0, 0, 0;
            m(0) = measurement[MEASUREMENT_LENGHT - 1][0];
            m(1) = measurement[MEASUREMENT_LENGHT - 1][1];
            m(2) = (float) (m(0) - measurement[MEASUREMENT_LENGHT - 2][0]) * 1000000000 / KALMAN_PERIOD;
            m(3) = (float) (m(1) - measurement[MEASUREMENT_LENGHT - 2][1]) * 1000000000 / KALMAN_PERIOD;

            pthread_mutex_lock(&mux_prediction);
                kalman_iterate(c, m);

                trail[MAX_TRAIL_LENGTH - 1][0] = (int) x(0);    
                trail[MAX_TRAIL_LENGTH - 1][1] = (int) x(1);

                if (prediction_flag == 1) {
                    prediction[0] = (int) prediction_x(0);
                    prediction[1] = (int) prediction_x(1);
                }
            pthread_mutex_unlock(&mux_prediction);
        pthread_mutex_unlock(&mux_trail);

        // update the next action variable and wait for the next period
        timespec_add(&next_act, &period);
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_act, NULL);

    } while (exit_flag == 1); 
    pthread_exit(NULL);
    
}

// Manage the keyboard
void *keyboard_task(void *arg_in) {
	struct timespec next_act, period;
    char scan, ascii;

    // Initialize the period of the task and the next action time
    period.tv_sec = 0;
    period.tv_nsec = KEYBOARD_PERIOD; 

    next_act = tx0;

    do {
        // get the key pressed by the user
        get_keycodes(&scan, &ascii);
        // switch the key pressed
        switch (scan) {
            case 'T': // KEY_UP: increase the noise
                pthread_mutex_lock(&mux_noise);
                    if (noise < MAX_NOISE) {
                        noise++;
                    }
                pthread_mutex_unlock(&mux_noise);
                break;
            case 'U': // KEY_DOWN: decrease the noise
                pthread_mutex_lock(&mux_noise);
                    if (noise > MIN_NOISE) {
                        noise--;
                    }
                pthread_mutex_unlock(&mux_noise);
                break;
            case 'S': // KEY_RIGHT: increase the trail's length
                pthread_mutex_lock(&mux_trail_length);
                    if (trail_length < MAX_TRAIL_LENGTH) {
                        trail_length++;
                    }
                pthread_mutex_unlock(&mux_trail_length);
                break;
            case 'R': // KEY_LEFT: decrease the trail's length
                pthread_mutex_lock(&mux_trail_length);
                    if (trail_length > MIN_TRAIL_LENGTH) {
                        trail_length--;
                    }
                pthread_mutex_unlock(&mux_trail_length);
                break;
            case 'K': // KEY_SPACE: disable/enable the prediction flag
                prediction_flag = - prediction_flag;
                break;
            case ';': // KEY_ESC: exit the program
                exit_flag = - exit_flag;
                break;
        }

        // update the next action variable and wait for the next period
        timespec_add(&next_act, &period);
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_act, NULL);
        
    } while (exit_flag == 1); 
    pthread_exit(NULL);
    
}

/* MAIN */
int main(){

    // initialize the program
    init();

    // initialize the clock
    clock_gettime(CLOCK_REALTIME, &t0);     // "zero" time
    clock_gettime(CLOCK_MONOTONIC, &tx0);
    
    // create the threads
    pthread_create(&thread_display, NULL, display_task, NULL);
    pthread_create(&thread_keyboard, NULL, keyboard_task, NULL);
    pthread_create(&thread_kalman, NULL, kalman_task, NULL);

    // wait for the threads to finish
    pthread_join(thread_display, NULL);
    pthread_join(thread_keyboard, NULL);
    pthread_join(thread_kalman, NULL);
    
    // close allegro's window
    allegro_exit(); 
    
    return 0;
}
