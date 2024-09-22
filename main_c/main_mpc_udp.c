// #include "/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/MA24_simulink/main_c/main_mpc_udp.h"
#include "main_mpc_udp.h"

#define PORT_C	         8081
#define PORT_SIMULINK	 8080
#define MAXLINE 48

void delay_ms(long ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

#define N_MPC 5
#define N_step_MPC 10

#define ROWS 7
#define COLS (N_MPC+1)

#define BLOCK_SIZE (ROWS * COLS)

#define TOTAL_UPDATES 5000 // Total number of data updates
#define NUM_DOUBLES 12 // for received robot state: 12 doubles = 2*n data q, qp
#define NUM_BYTES NUM_DOUBLES*8


void read_trajectory_block(FILE* file, long int data_start, casadi_real* data, const int* indices) {
    for (int j = 0; j < COLS; j++) {
        fseek(file, data_start + indices[j] * ROWS * sizeof(casadi_real), SEEK_SET);
        fread(&data[j * ROWS], sizeof(casadi_real), ROWS, file);
    }
}

void read_file(FILE* file, long int data_start, casadi_real* data, const int data_len) {
    for (int j = 0; j < data_len; j++) {
        fseek(file, data_start + j * sizeof(casadi_real), SEEK_SET);
        fread(&data[j], sizeof(casadi_real), 1, file);
    }
}

struct shared_data shared;

// Function to create a precise delay
void precise_nanosleep(long ns) {
    struct timespec req, rem;
    req.tv_sec = 0;
    req.tv_nsec = ns;
    while (nanosleep(&req, &rem) == -1) {
        req = rem;
    }
}

// Function to update data using MPC8
void *update_data(void *arg) {
    casadi_real *u_opt_res_ptr = shared.res[0]; // zeigt auf u_opt
    casadi_real* a = 0;
    const casadi_real* r=0;
    int n=0;
    int indices[COLS] = {0, 1, 100, 200, 300, 400};
    casadi_real data[BLOCK_SIZE] = {0};
    FILE *traj_file = shared.traj_file;
    setvbuf(stdout, NULL, _IOFBF, BUFSIZ);
    long int firstrun = 1;
    long int init_guess_len = (shared.res[2] - shared.res[1])*sizeof(casadi_real);
    casadi_real *init_guess_in = (casadi_real *) shared.arg[1];
    casadi_real *init_guess_sol = shared.res[1];
    casadi_int flag = 0;
    CasadiFunPtr_t casadi_fun = shared.casadi_fun;

    for (int i = 0; i < TOTAL_UPDATES; i++)
    {

        n = recvfrom(shared.sockfd, (casadi_real *) shared.input_buffer, NUM_BYTES,
            MSG_WAITALL, ( struct sockaddr *) &shared.udp_c_receive_addr,
            &shared.udp_c_receive_addrlen);      

        // pthread_mutex_lock(&shared.mutex); // Lock mutex before updating data
        //read trajectory
        read_trajectory_block(traj_file, 8, &data[0], &indices[0]);
        memcpy(&shared.w[12], data, sizeof(data));
        
        // Call MPC8 to update data
        flag = casadi_fun(shared.arg, shared.res, shared.iw, shared.w_end_addr, 0);

        CASADI_PRINTF("Empfangene Daten: ");
        for (int j = 0; j < NUM_DOUBLES; j++) {
            CASADI_PRINTF("%2.4g ", shared.input_buffer[j]);
        }

        //r=shared.w+510;
        // CASADI_PRINTF("\n  u_opt:\n");
        // for (int j=0; j<6; ++j) CASADI_PRINTF("%2.4g ", *r++);
        // CASADI_PRINTF("\n  u:\n");
        // for (int j=6; j<36; ++j) CASADI_PRINTF("%2.4g ", *r++);
        // CASADI_PRINTF("\n  x_opt:\n");
        // for (int j=36; j<36+72; ++j) CASADI_PRINTF("%2.4g ", *r++);
        // CASADI_PRINTF("\n\n");

        r=shared.w;
        CASADI_PRINTF("ALL INPUT:\n");
        for (int j=0; j<510; ++j) CASADI_PRINTF("%2.4g ", *r++);
        CASADI_PRINTF("\n\n");
        CASADI_PRINTF("ALL OUTPUT:\n");
        for (int j=510; j<797; ++j) CASADI_PRINTF("%2.4g ", *r++);
        CASADI_PRINTF("\n\n");

        fflush(stdout);
        
        if (flag != 0) {
            printf("Error in MPC8: flag = %d\n", flag);
            shared.should_exit = 1;
            pthread_mutex_unlock(&shared.mutex);
            return NULL;
        }

        // if(firstrun == 1)
        // {
        //   u_opt_res_ptr = shared.res[0]; // send u_opt_0
        //   firstrun = 0;
        // }
        // else
        // {
        //   u_opt_res_ptr = shared.res[1]+6; // send u_opt_1
        // }

        for(int j=0; j<6; j++)
        {
          shared.u_opt_new[j] = u_opt_res_ptr[j];
          indices[j] = indices[j] + 1; // set indices to next prediction horizon
        }

        // set current solution as initial guess:
        memcpy(init_guess_in, init_guess_sol, init_guess_len);
        
        // pthread_mutex_unlock(&shared.mutex); // Unlock mutex after updating

        // delay_ms(1);
        // precise_nanosleep(500000);
    }
    
    shared.should_exit = 1; // Signal to exit after all updates
    return NULL;
}

// Function to send data
void *send_data(void *arg) {
    struct timespec next_send;
    clock_gettime(CLOCK_MONOTONIC, &next_send);

    while (!shared.should_exit) {
        // Wait until it's time to send the data
        while (1) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            long elapsed_ns = (now.tv_sec - next_send.tv_sec) * 1000000000L + 
                              (now.tv_nsec - next_send.tv_nsec);
            if (elapsed_ns >= shared.send_interval_ns) {
                break; // Time to send data
            }
            precise_nanosleep(100); // Sleep for a short duration to reduce CPU load
        }

        // pthread_mutex_lock(&shared.mutex); // Lock mutex before sending

        // Send data to simulink. Info: shared.buffers shows at adress of u_opt
        sendto(shared.sockfd, shared.u_opt_new, shared.u_opt_new_size,
                  MSG_CONFIRM, (const struct sockaddr *) &shared.udp_c_send_addr, shared.udp_c_send_addrlen);

        // printf("Gesendete Daten: ");
        // for (int i = 0; i < 6; i++) {
        //     printf("%g ", shared.u_opt_new[i]);
        // }
        // printf("\n");
        
        // pthread_mutex_unlock(&shared.mutex); // Unlock mutex after sending

        // Calculate next send time
        next_send.tv_nsec += shared.send_interval_ns;
        if (next_send.tv_nsec >= 1000000000) {
            next_send.tv_sec += 1;
            next_send.tv_nsec -= 1000000000;
        }
    }
    
    return NULL;
}

void threaded_send(int sockfd, casadi_real *u_opt_new, size_t u_opt_new_size,
                   casadi_real *input_buffer, size_t input_buffer_size,
                   const struct sockaddr_in udp_c_send_addr,
                   const struct sockaddr_in udp_c_receive_addr,
                   unsigned long update_interval_ms, unsigned long send_interval_ms,
                   const casadi_real** arg, casadi_real** res,
                   casadi_int* iw, casadi_real* w, casadi_real* w_end_addr,
                   FILE *traj_file, CasadiFunPtr_t casadi_fun) {
    
    pthread_t update_thread, send_thread;

    // Initialize shared data
    shared.u_opt_new = u_opt_new;
    shared.u_opt_new_size = u_opt_new_size;
    shared.input_buffer = input_buffer;
    shared.input_buffer_size = input_buffer_size;
    shared.sockfd = sockfd;
    shared.udp_c_send_addr = udp_c_send_addr;//client: simulink
    shared.udp_c_send_addrlen = sizeof(udp_c_send_addr);
    shared.udp_c_receive_addr = udp_c_receive_addr;//server: C
    shared.udp_c_receive_addrlen = sizeof(udp_c_receive_addr);

    shared.update_interval_ns = update_interval_ms * 1000000; // Convert ms to ns
    shared.send_interval_ns = send_interval_ms * 1000000;     // Convert ms to ns

    shared.should_exit = 0; 
    shared.arg = arg;
    shared.res = res; 
    shared.iw = iw; 
    shared.w = w;
    shared.w_end_addr = w_end_addr;

    shared.traj_file = traj_file;

    shared.casadi_fun = casadi_fun;

    clock_gettime(CLOCK_MONOTONIC, &shared.start_time); // Record start time

    pthread_mutex_init(&shared.mutex, NULL); 

    pthread_create(&update_thread, NULL, update_data, NULL);
    pthread_create(&send_thread, NULL, send_data, NULL);

    pthread_join(update_thread, NULL);
    pthread_join(send_thread, NULL);

    pthread_mutex_destroy(&shared.mutex); 
}

casadi_int main_MPC(const casadi_real** arg, casadi_real** res,
                    casadi_int* iw, casadi_real* w, casadi_real* w_end_addr, int mem,
                    CasadiFunPtr_t casadi_fun)
{
  casadi_int flag=0;
  FILE *traj_file;
  FILE *init_guess_file;
  unsigned int rows, cols;
  long int data_start;
  long int data_start_init_guess;
  double block[BLOCK_SIZE];
  casadi_real data[BLOCK_SIZE] = {0};
  casadi_real init_guess_data[510] = {0};
  int indices[COLS] = {0, 1, 100, 200, 300, 400};

  traj_file = fopen("traj.bin", "rb");
  if (traj_file == NULL) {
      printf("Error opening traj_file\n");
      return 1;
  }

  // // Read the dimensions
  fread(&rows, sizeof(unsigned int), 1, traj_file);
  fread(&cols, sizeof(unsigned int), 1, traj_file);

  printf("Trajectory dimensions: %d x %d\n", rows, cols);

  // Calculate the starting position of the data
  data_start = ftell(traj_file);

  read_trajectory_block(traj_file, data_start, &data[0], &indices[0]);

  init_guess_file = fopen("mpc8_init_allinputs.bin", "rb");
  if (traj_file == NULL) {
      printf("Error opening traj_file\n");
      return 1;
  }

  fread(&rows, sizeof(unsigned int), 1, init_guess_file);
  fread(&cols, sizeof(unsigned int), 1, init_guess_file);

  printf("init_guess dimensions: %d x %d\n", rows, cols);

  data_start_init_guess = ftell(init_guess_file);

  read_file(init_guess_file, data_start_init_guess, init_guess_data, 510);

  // printf("Gelesene Daten:\n");
  // for (int i = 0; i < BLOCK_SIZE; i++) {
      
  //     printf("%2.17f ", data[i]);
  //     if(i > 0 && (i+1) % ROWS == 0) printf("\n");
  // }

  // UDP CONNECTION

	int sockfd;
	struct sockaddr_in udp_c_receive_addr, udp_c_send_addr;
		
	// Creating socket file descriptor
	if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("socket creation failed");
		exit(EXIT_FAILURE);
	}
		
	memset(&udp_c_receive_addr, 0, sizeof(udp_c_receive_addr));
	memset(&udp_c_send_addr, 0, sizeof(udp_c_send_addr));
		
	// Filling server information
	udp_c_receive_addr.sin_family = AF_INET; // IPv4
	udp_c_receive_addr.sin_addr.s_addr = INADDR_ANY;
	udp_c_receive_addr.sin_port = htons(PORT_C);

	udp_c_send_addr.sin_family = AF_INET; // IPv4
	udp_c_send_addr.sin_addr.s_addr = INADDR_ANY;
	udp_c_send_addr.sin_port = htons(PORT_SIMULINK);
		
	// Bind the socket with the server address
	if ( bind(sockfd, (const struct sockaddr *)&udp_c_receive_addr,
			sizeof(udp_c_receive_addr)) < 0 )
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}

  memcpy(w, init_guess_data, sizeof(init_guess_data));
  memcpy(&w[12], data, sizeof(data));

  flag = casadi_fun(arg, res, iw, w_end_addr, 0);
  if (flag) return flag;

  // send only u_opt = q_pp
  casadi_real u_opt[6] = {0};
  casadi_real *u_opt_res_ptr = res[0];
  for(int i=0; i<6; i++)
  {
    u_opt[i] = u_opt_res_ptr[i];
  }

  casadi_real *input_robot_state = &w[0]; // idee: beim empfangen von daten anfangszustand direkt ueberschreiben.

  threaded_send(sockfd, &u_opt[0], 6 * sizeof(casadi_real),
                input_robot_state, 6*2 * sizeof(casadi_real),
                udp_c_send_addr,
                udp_c_receive_addr,
                1,    // Update interval in milliseconds
                0.1,   // Send interval in milliseconds
                arg, res, iw, w, w_end_addr,
                traj_file, casadi_fun);

  fflush(stdout);
  // Close the file
  fclose(traj_file);
}

// int main()
// {
//     printf("done\n");
// }