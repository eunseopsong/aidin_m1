#include "convexMPC_interface.h"
#include "common_types.h"
#include "SolverMPC.h"
#include <eigen3/Eigen/Dense>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

#define K_NUM_LEGS 4

#define K_NUM_LEGS 4

problem_setup problem_configuration;
u8 gait_data[K_MAX_GAIT_SEGMENTS];
pthread_mutex_t problem_cfg_mt;
pthread_mutex_t update_mt;
update_data_t update;
pthread_t solve_thread;

u8 first_run = 1;

void initialize_mpc()
{
  //printf("Initializing MPC!\n");
  if(pthread_mutex_init(&problem_cfg_mt,NULL)!=0)
    printf("[MPC ERROR] Failed to initialize problem configuration mutex.\n");

  if(pthread_mutex_init(&update_mt,NULL)!=0)
    printf("[MPC ERROR] Failed to initialize update data mutex.\n");

#ifdef K_DEBUG
  printf("[MPC] Debugging enabled.\n");
    printf("[MPC] Size of problem setup struct: %ld bytes.\n", sizeof(problem_setup));
    printf("      Size of problem update struct: %ld bytes.\n",sizeof(update_data_t));
    printf("      Size of MATLAB floating point type: %ld bytes.\n",sizeof(mfp));
    printf("      Size of flt: %ld bytes.\n",sizeof(flt));
#else
  //printf("[MPC] Debugging disabled.\n");
#endif
}

void setup_problem(double dt, int horizon, double mu, double f_max)
{
  //mu = 0.6;
  if(first_run)
  {
    first_run = false;
    initialize_mpc();
  }

#ifdef K_DEBUG
  printf("[MPC] Got new problem configuration!\n");
    printf("[MPC] Prediction horizon length: %d\n      Force limit: %.3f, friction %.3f\n      dt: %.3f\n",
            horizon,f_max,mu,dt);
#endif

  //pthread_mutex_lock(&problem_cfg_mt);

  problem_configuration.horizon = horizon;
  problem_configuration.f_max = f_max;
  problem_configuration.mu = mu;
  problem_configuration.dt = dt;

  //pthread_mutex_unlock(&problem_cfg_mt);

  resize_qp_mats(horizon);
}








