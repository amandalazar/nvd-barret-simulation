/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <webots/camera_recognition_object.h>

#define TIME_STEP 16 /* In ms */
#define TARGET_POINTS_SIZE 2
#define DISTANCE_TOLERANCE 1.5
#define MAX_SPEED 7.0
#define WALKING_SPEED 5.0
#define TURN_COEFFICIENT 4.0
#define MAX_DIST_RANGE 10.0
#define FOLLOW_ME_HEIGHT_THRESHOLD 175
#define FOLLOW_ME_LEFT_THRESHOLD 300
#define FOLLOW_ME_RIGHT_THRESHOLD 340
#define ONE_SECOND_IN_TIME_STEPS 1000.0 / TIME_STEP
#define TWO_SECONDS_IN_TIME_STEPS 2000.0 / TIME_STEP
#define MAX_FOLLOW_ME_PATH_DURATION 100 /* In s */

enum XYZAComponents { X, Y, Z, ALPHA };
enum Sides { LEFT, RIGHT };

typedef struct _Vector {
  double u;
  double v;
} Vector;

static WbDeviceTag motors[8];
static WbDeviceTag gps;
static WbDeviceTag compass;
static WbDeviceTag dist_sensor_front;
static WbDeviceTag dist_sensor_left;
static WbDeviceTag dist_sensor_right;
static WbDeviceTag camera;

static Vector follow_me_path[MAX_FOLLOW_ME_PATH_DURATION];
static int current_target_index = 0;
static bool autopilot = false;
static bool old_autopilot = false;
static bool follow_me_mode = true;
static bool autonomy_complete = false;
static int old_key = -1;
static bool object_detected = false;
static int elapsed_time_steps = 0;
static int follow_me_path_index = -1;
static int follow_me_path_size;
static int follow_me_complete_counter = 0;
static int autonomy_complete_counter = 0;
static bool instructions_printed = false;

static double modulus_double(double a, double m) {
  int div_i = (int)(a / m);
  double div_d = (double)div_i;
  double r = a - div_d * m;
  if (r < 0.0)
    r += m;
  return r;
}

// set left and right motor speed [rad/s]
static void robot_set_speed(double left, double right) {
  int i;
  for (i = 0; i < 4; i++) {
    wb_motor_set_velocity(motors[i + 0], left);
    wb_motor_set_velocity(motors[i + 4], right);
  }
}

static bool is_robot_stopped() {
  bool stopped = true;
  int i;
  for (i = 0; i < 4; i++) {
    double left_velocity = wb_motor_get_velocity(motors[i + 0]);
    double right_velocity = wb_motor_get_velocity(motors[i + 4]);
    if (left_velocity > 0.0 || right_velocity > 0.0)
      stopped = false;
      break;
  }

  return stopped;
}

static void check_keyboard() {
  double speeds[2] = {0.0, 0.0};

  int key = wb_keyboard_get_key();
  if (key >= 0) {
    switch (key) {
      case WB_KEYBOARD_UP:
        speeds[LEFT] = MAX_SPEED;
        speeds[RIGHT] = MAX_SPEED;
        autopilot = false;
        break;
      case WB_KEYBOARD_DOWN:
        speeds[LEFT] = -MAX_SPEED;
        speeds[RIGHT] = -MAX_SPEED;
        autopilot = false;
        break;
      case WB_KEYBOARD_RIGHT:
        speeds[LEFT] = MAX_SPEED;
        speeds[RIGHT] = -MAX_SPEED;
        autopilot = false;
        break;
      case WB_KEYBOARD_LEFT:
        speeds[LEFT] = -MAX_SPEED;
        speeds[RIGHT] = MAX_SPEED;
        autopilot = false;
        break;
      case 'P':
        if (key != old_key) {  // perform this action just once
          const double *pos3D = wb_gps_get_values(gps);
          printf("position: {%f, %f}\n", pos3D[X], pos3D[Z]);
        }
        break;
      case 'A':
        if (key != old_key)  // perform this action just once
          follow_me_mode = false;
          autopilot = !autopilot;
        break;
      case 'F':
        if (key != old_key) {  // perform this action just once
          autopilot = false;
          follow_me_mode = !follow_me_mode;
        }
        break;
    }
  }
  if (autopilot != old_autopilot) {
    old_autopilot = autopilot;
    if (autopilot)
      printf("Autonomous control commenced...\n");
    else
      printf("manual control\n");
  }

  robot_set_speed(speeds[LEFT], speeds[RIGHT]);
  old_key = key;
}

static void check_obstacles() {
  double dist_front = wb_distance_sensor_get_value(dist_sensor_front);
  double dist_left = wb_distance_sensor_get_value(dist_sensor_left);
  double dist_right = wb_distance_sensor_get_value(dist_sensor_right);

  // Stop when obstacle is near
  if (dist_front < MAX_DIST_RANGE || dist_left < MAX_DIST_RANGE || dist_right < MAX_DIST_RANGE) {
    if (!object_detected) {
      printf("OBSTACLE DETECTED: Stopping...\n");
      robot_set_speed(0.0, 0.0);
      object_detected = true;
    }
  } else {
    object_detected = false;
  }
}

// ||v||
static double norm(const Vector *v) {
  return sqrt(v->u * v->u + v->v * v->v);
}

// v = v/||v||
static void normalize(Vector *v) {
  double n = norm(v);
  v->u /= n;
  v->v /= n;
}

// v = v1-v2
static void minus(Vector *v, const Vector *v1, const Vector *v2) {
  v->u = v1->u - v2->u;
  v->v = v1->v - v2->v;
}

// compute the angle between two vectors
// return value: [0, 2Pi[
static double angle(const Vector *v1, const Vector *v2) {
  return modulus_double(atan2(v2->v, v2->u) - atan2(v1->v, v1->u), 2.0 * M_PI);
}

// autopilot
// pass trough the predefined target positions
// TODO: Alter function to wait for some keyboard input to continue moving after reaching back to point A
static void run_autopilot() {
  if (autonomy_complete && autonomy_complete_counter < TWO_SECONDS_IN_TIME_STEPS) {
    autonomy_complete_counter++;
    return;
  }

  // prepare the speed array
  double speeds[2] = {0.0, 0.0};

  // read gps position and compass values
  const double *pos3D = wb_gps_get_values(gps);
  const double *north3D = wb_compass_get_values(compass);

  // compute the 2D position of the robot and its orientation
  Vector pos = {pos3D[X], pos3D[Z]};
  Vector north = {north3D[X], north3D[Z]};
  Vector front = {-north.u, north.v};

  // compute the direction and the distance to the target
  Vector dir;
  minus(&dir, &(follow_me_path[current_target_index]), &pos);
  double distance = norm(&dir);
  normalize(&dir);

  // compute the target angle
  double beta = angle(&front, &dir) - M_PI;

  // a target position has been reached
  if (distance < DISTANCE_TOLERANCE) {
    if (current_target_index == 0) {
      printf("Final target reached! Stopping and waiting to be loaded before proceeding back along mapped path...\n");

      // Sleep while stopped for a small amount of time to complete follow-me demo (TODO: Remove this)
      speeds[LEFT] = 0.0;
      speeds[RIGHT] = 0.0;
      autonomy_complete = true;
    }
    else
      printf("Target #%d reached\n", current_target_index + 1);
    current_target_index--;
    if (current_target_index < 0)
      current_target_index = follow_me_path_size - 1;
    // TODO: Figure out how original controller runs through target array forwards then backwards,
    // while mine runs through follow_me_path array backwards then backwards again
  }
  // move the robot to the next target
  else {
    speeds[LEFT] = MAX_SPEED - M_PI + TURN_COEFFICIENT * beta;
    speeds[RIGHT] = MAX_SPEED - M_PI - TURN_COEFFICIENT * beta;
  }

  // set the motor speeds
  robot_set_speed(speeds[LEFT], speeds[RIGHT]);
}

static void run_follow_me_mode() {
  WbCameraRecognitionObject detectedObject = wb_camera_recognition_get_objects(camera)[0];

  // prepare the speed array
  double speeds[2] = {0.0, 0.0};

  int viewedHeight = detectedObject.size_on_image[1];
  int viewedHorizantalDisplacement = detectedObject.position_on_image[0];

  if (wb_camera_recognition_get_number_of_objects(camera) == 1) {
    if (viewedHeight < FOLLOW_ME_HEIGHT_THRESHOLD) {
      // Move forward
      speeds[LEFT] = WALKING_SPEED;
      speeds[RIGHT] = WALKING_SPEED;
    }
    if (viewedHorizantalDisplacement < FOLLOW_ME_LEFT_THRESHOLD) {
      // Move right
      speeds[LEFT] = -WALKING_SPEED;
      speeds[RIGHT] = WALKING_SPEED;
    } else if (viewedHorizantalDisplacement > FOLLOW_ME_RIGHT_THRESHOLD) {
      // Move left
      speeds[LEFT] = WALKING_SPEED;
      speeds[RIGHT] = -WALKING_SPEED;
    }

    // set the motor speeds
    robot_set_speed(speeds[LEFT], speeds[RIGHT]);

    if (is_robot_stopped()) {
      if (follow_me_complete_counter >= TWO_SECONDS_IN_TIME_STEPS) {
        follow_me_path_size = follow_me_path_index;
        current_target_index = follow_me_path_size - 1;
        autopilot = true;
      }
      follow_me_complete_counter++;
    } else {
      follow_me_complete_counter = 0;

      if (elapsed_time_steps >= ONE_SECOND_IN_TIME_STEPS) {
        // Every second that passes by in active follow-me mode, save the current position
        const double *pos3D = wb_gps_get_values(gps);
        printf("Saving current position #%d: {%f, %f}\n", ++follow_me_path_index + 1, pos3D[X], pos3D[Z]);
        Vector currPosition = { .u = pos3D[X], .v = pos3D[Z] };
        follow_me_path[follow_me_path_index] = currPosition;
        elapsed_time_steps = 0;
      } else {
        elapsed_time_steps++;
      }
    }
  }
}

int main(int argc, char *argv[]) {
  printf("Initializing robot...\n");

  // initialize webots communication
  wb_robot_init();

  wb_robot_step(1000);

  const char *names[8] = {"left motor 1",  "left motor 2",  "left motor 3",  "left motor 4",
                          "right motor 1", "right motor 2", "right motor 3", "right motor 4"};

  // get motor tags
  int i;
  for (i = 0; i < 8; i++) {
    motors[i] = wb_robot_get_device(names[i]);
    wb_motor_set_position(motors[i], INFINITY);
  }

  // get gps tag and enable
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  // get compass tag and enable
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);

  // get distance sensors tags and enable
  dist_sensor_front = wb_robot_get_device("dist_sensor_front");
  wb_distance_sensor_enable(dist_sensor_front, TIME_STEP);
  dist_sensor_left = wb_robot_get_device("dist_sensor_left");
  wb_distance_sensor_enable(dist_sensor_left, TIME_STEP);
  dist_sensor_right = wb_robot_get_device("dist_sensor_right");
  wb_distance_sensor_enable(dist_sensor_right, TIME_STEP);

  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  wb_camera_recognition_enable(camera, TIME_STEP);

  // enable keyboard
  wb_keyboard_enable(TIME_STEP);

  // start forward motion
  robot_set_speed(MAX_SPEED, MAX_SPEED);

  // main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    if (!instructions_printed) {
      printf("Starting follow-me mode...\n");
      instructions_printed = true;
    }
    check_obstacles();
    check_keyboard();
    if (autopilot && !object_detected)
      run_autopilot();
    else if (follow_me_mode && !object_detected)
      run_follow_me_mode();
  }

  wb_robot_cleanup();

  return 0;
}
