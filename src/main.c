#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "marvelmind_api.h"
#include "marvelmind_example.h"
#include "marvelmind_utils.h"
#ifdef WIN32
#include <conio.h>
#else
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#endif

#ifndef WIN32
int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}
#endif

// Check input command
static void checkCommand() {
    #ifdef WIN32
    if (!kbhit()) return;
    if (getch() != ' ') return;
    #else
    if (!kbhit()) return;
    if (getchar() != ' ') return;
    #endif

    char str [80];
    MarvelmindDeviceVersion version;

    printf("Enter command: ");
    fgets(str, 79, stdin);

    char *token1 = strtok(str, " ");
    trim(token1);
    if (strcmp(token1,"quit") == 0) {
        exit(0);
        return;
    }

    char *token2 = strtok(NULL, " ");
    char *token3 = NULL;
    if (token2 != NULL) {
        trim(token2);

        token3 = strtok(NULL, " ");
        if (token3 != NULL) {
            trim(token3);
        }
    }

    char *token4 = NULL;
    if (token3 != NULL) {
        trim(token3);

        token4 = strtok(NULL, " ");
        if (token4 != NULL) {
            trim(token4);
        }
    }

    char *token5 = NULL;
    if (token4 != NULL) {
        trim(token4);

        token5 = strtok(NULL, " ");
        if (token5 != NULL) {
            trim(token5);
        }
    }

    mmGetVersionAndId(255, &version);
    mmGetVersionAndId(255, &version);
    sleep_ms(10);

    if (marvelmindCheckVersionCommand(token1)) {
        return;
    }

    if (marvelmindCheckWakeCommand(token1, token2)) {
        return;
    }

    if (marvelmindCheckSleepCommand(token1, token2)) {
        return;
    }

    if (marvelmindCheckDefaultCommand(token1, token2)) {
        return;
    }

    if (marvelmindCheckTelemetryCommand(token1, token2)) {
        return;
    }

    if (marvelmindCheckSubmapCommand(token1, token2, token3)) {
        return;
    }

    if (marvelmindCheckMapCommand(token1, token2)) {
        return;
    }

    if (marvelmindCheckRateCommand(token1, token2, token3)) {
        return;
    }

    if (marvelmindCheckUltrasoundCommand(token1, token2, token3)) {
        return;
    }

    if (marvelmindCheckAxesCommand(token1, token2, token3, token4)) {
        return;
    }

    if (marvelmindCheckReadDumpCommand(token1, token2, token3)) {
        return;
    }

    if (marvelmindCheckWriteDumpCommand(token1, token2, token3)) {
        return;
    }

    if (marvelmindCheckResetCommand(token1, token2)) {
        return;
    }

    if (marvelmindCheckTemperatureCommand(token1, token2, token3)) {
        return;
    }

    if (marvelmindCheckSetLocCommand(token1, token2, token3, token4, token5)) {
        return;
    }

    if (marvelmindCheckSetDistanceCommand(token1, token2, token3, token4)) {
        return;
    }

    if (marvelmindCheckSetMotorsCommand(token1, token2, token3, token4, token5)) {
        return;
    }

    if (marvelmindCheckHeightCommand(token1, token2, token3, token4, token5)) {
        return;
    }

    if (marvelmindCheckRealtimePlayerCommand(token1, token2, token3)) {
        return;
    }

    if (marvelmindCheckGeoreferencingCommand(token1, token2, token3, token4)) {
        return;
    }

    if (marvelmindCheckUpdateLocationModeCommand(token1, token2, token3)) {
        return;
    }

    if (marvelmindCheckSendUpdateLocationCommand(token1)) {
        return;
    }

    if (marvelmindCheckSetAlarmStateCommand(token1, token2, token3, token4)) {
        return;
    }

    if (marvelmindCheckPayloadCommand(token1, token2, token3)) {
        return;
    }

    if (marvelmindCheckDistanceCommand(token1, token2, token3, token4)) {
        return;
    }
}

int main(int argc, char * argv[])
{
    marvelmindStart(argc, argv);

    while(1) {
        marvelmindCycle();

        sleep_ms(1);

        checkCommand();
    }

    marvelmindFinish();

    return 0;
}
