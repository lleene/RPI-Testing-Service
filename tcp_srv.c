//
// RPI TCP/IP SOCKET -- REMOTE CONTROL GPIO/SPI
// LIEUWE B. LEENE 2019
//

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "bcm2835_test.h"

#define MYPORT 3490    // the port users will be connecting to
#define MAXDATASIZE 50 // max number of bytes we can get at once
#define BACKLOG 4      // how many pending connections queue will hold
#define OK_STR "ACK"
#define NK_STR "NAK"
#define ID_STR "VYGR-V1"
#define CMD_STR "XXXX-XXXX"

static const char *CST_STR = "CLOK-STRT";
static const char *CSP_STR = "CLOK-STOP";
static const char *END_STR = "VYGR-TERM";
static const char *RND_STR = "VYGR-RNDM";
static const char *CFG_STR = "VYGR-CNFG";
static const char *RCD_STR = "VYGR-RECD";
static const char *DST_STR = "VDAC-STRT";
static const char *DSP_STR = "VDAC-STOP";
static const char *DFG_STR = "VDAC-CNFG";

void sigchld_handler(int s) {
  while (waitpid(-1, NULL, WNOHANG) > 0)
    ;
}

int main(void) {
  int sockfd, new_fd, numbytes; // listen on sock_fd, new connection on new_fd
  char vygr_config[32];
  syth_cnfg_t synth_config = {}; // initialize with zeros
  char buf[MAXDATASIZE];

  // setup pheripheral modules
  printf("Running RPI_V3 Mode\n");
  if (!bcm2835_test_AKIRA_init()) {
    perror("memmap");
    exit(1);
  }

  pthread_t tid = -1;
  pthread_attr_t tattr;
  pthread_attr_init(&tattr);

  struct sockaddr_in my_addr;    // my address information
  struct sockaddr_in their_addr; // connector's address information
  socklen_t sin_size;
  struct sigaction sa;
  int yes = 1;

  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    perror("socket");
    exit(1);
  }

  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1) {
    perror("setsockopt");
    exit(1);
  }

  my_addr.sin_family = AF_INET;         // host byte order
  my_addr.sin_port = htons(MYPORT);     // short, network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY; // automatically fill with my IP
  memset(&(my_addr.sin_zero), '\0', 8); // zero the rest of the struct

  if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) ==
      -1) {
    perror("bind");
    exit(1);
  }

  if (listen(sockfd, BACKLOG) == -1) {
    perror("listen");
    exit(1);
  }

  sa.sa_handler = sigchld_handler; // reap all dead processes
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_RESTART;
  if (sigaction(SIGCHLD, &sa, NULL) == -1) {
    perror("sigaction");
    exit(1);
  }

  while (1) { // main accept() loop DO-WHILE
    synth_config.recording_index = 0;
    sin_size = sizeof(struct sockaddr_in);
    if ((new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &sin_size)) ==
        -1) {
      perror("accept");
      continue;
    }
    printf("server: got connection from %s\n",
           inet_ntoa(their_addr.sin_addr)); // Connection recieved
    usleep(256);
    if (send(new_fd, ID_STR, sizeof(ID_STR), 0) == -1)
      perror("send"); // SEND HOST ID
    while ((numbytes = recv(new_fd, buf, MAXDATASIZE - 1, 0)) !=
           -1) { // GET REQUESTS FROM CLIENT
      if (numbytes == 0 && recv(new_fd, buf, MAXDATASIZE - 1, 0) == 0) {
        break; // lost connection
      }
      if (numbytes == sizeof(CMD_STR)) {
        send(new_fd, OK_STR, sizeof(OK_STR), 0);

        if (memcmp(buf, CST_STR, sizeof(CMD_STR)) == 0) { // ENABLE CLK
          bcm2835_test_start_clk(&vygr_config[0]);
        }

        if (memcmp(buf, CSP_STR, sizeof(CMD_STR)) == 0) { // DISABLE CLK
          bcm2835_test_stop_clk(&vygr_config[0]);
        }

        if (memcmp(buf, RCD_STR, sizeof(CMD_STR)) == 0) { // RECORD SEGMENT
          synth_config.recording_index += 32;
        }

        if (memcmp(buf, CFG_STR, sizeof(CMD_STR)) == 0) {
          numbytes =
              recv(new_fd, buf, MAXDATASIZE - 1, 0); // GET REQUESTS FROM CLIENT
          if (numbytes == sizeof(vygr_config)) {
            memcpy(vygr_config, buf, sizeof(vygr_config));
            bcm2835_test_configure_akira(&vygr_config[0], &synth_config);
            bcm2835_test_print_config(&vygr_config[0]);
            send(new_fd, OK_STR, sizeof(OK_STR), 0);
          } else
            send(new_fd, NK_STR, sizeof(NK_STR), 0);
        }

        if (memcmp(buf, RND_STR, sizeof(CMD_STR)) == 0) {

        }

        if (memcmp(buf, DST_STR, sizeof(CMD_STR)) == 0) {
          synth_config.status_code = SYNTHCORE_ENABLE;
          if (tid != -1) {
            fprintf(stdout, "Thread is already running....\n");
          } else { // Channel may be Paused or Stopped
            pthread_create(&tid, &tattr, bcm2835_test_sdmdma_1ch_proc,
                           &synth_config);
            fprintf(stdout, "Synthesizer started....\n");
          }
        }

        if (memcmp(buf, DSP_STR, sizeof(CMD_STR)) == 0) {
          synth_config.status_code = SYNTHCORE_STOP;
          if (tid != -1 && pthread_join(tid, NULL) != 0) {
            send(new_fd, NK_STR, sizeof(NK_STR), 0);
            perror("threadlost");
            exit(1);
          } else {
            send(new_fd, OK_STR, sizeof(OK_STR), 0);
            fprintf(stdout, "Synthesizer Stopped....\n");
            tid = -1;
          }
        }

        if (memcmp(buf, DFG_STR, sizeof(CMD_STR)) == 0) {
          numbytes =
              recv(new_fd, buf, MAXDATASIZE - 1, 0); // GET REQUESTS FROM CLIENT
          if (numbytes == 8 * sizeof(int)) {
            memcpy(&(synth_config.amplitude[0]), buf, 8 * sizeof(int));
            send(new_fd, OK_STR, sizeof(OK_STR), 0);
          } else
            send(new_fd, NK_STR, sizeof(NK_STR), 0);
        }

        if (memcmp(buf, END_STR, sizeof(CMD_STR)) == 0) {
          break;
        }

      } else
        send(new_fd, NK_STR, sizeof(NK_STR), 0);
    }
    close(new_fd);
    printf("server: connection closed\n");
  }

  close(sockfd); // Clean up parent process
  bcm2835_test_VYGR2_end();
  fprintf(stdout, "Clean Exit. \n");
  return 0;
}

/*

for(int i=0;i<4;i++) fprintf(stdout,"FRQ:%d AMP:%d\n",synth_config.frequency[i],synth_config.amplitude[i]);

void* bcm2835_test_sdmdma_proc(void* args){
  syth_cnfg_t* configuration = args;
  while(configuration->status_code != SYNTHCORE_STOP){
    sleep(1);
    for(int i=0;i<4;i++) {
                fprintf(stdout,"FRQ:%d
AMP:%d\n",configuration->frequency[i],configuration->amplitude[i]);
        }
  }
  return NULL;
}

void bcm2835_test_send_config(char* test_config){
        int j;
        uint32_t config_word=0;
        for(j=0;j<32;j++){
                if( test_config[j] == '1' ) config_word |= 0x1<<j;
        }

        for(j=0;j<32;j++){
                if( config_word & (0x1<<j) ) printf("1");
                else printf("0");
        }
        printf("\n");
}

*/
