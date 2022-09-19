#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <errno.h>

int main() {
    int pid = fork();
    if (pid == 0) {
        int ex = execl("./routing.py", "./routing.py", NULL);
        if (ex == 0) {
            puts("successfully executed");
            return 0;
        } else {
            puts("error happened!");
            return 1;
        }
    } else {
        printf("child pid is %d\n", pid);
        char pid_buff[4];
	sprintf(pid_buff, "%d", pid);
	int ext = execl("./run_detect.sh", "./run_detect.sh", "--pid", pid_buff, NULL);
	if (ext != 0) {
	    printf("%d\n", errno);
	}
	/*
        puts("Parent woke up");
        kill(pid, SIGINT);        
        int status;
        waitpid(pid, &status, 0);
	*/
    }

    return 0;
}
