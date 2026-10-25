// RCOM 2026/2027
// Link layer protocol implementation

#define _DEFAULT_SOURCE 1
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include "link_layer.h"
#include "serial_port.h"

// Constants
#define FLAG 0x7E
#define ESC 0x7D
#define A_CMD 0x03
#define A_RSP 0x01
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B
#define C_RR0 0x05
#define C_RR1 0x85
#define C_REJ0 0x01
#define C_REJ1 0x81
#define C_I0 0x00
#define C_I1 0x40

typedef enum { START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP_STATE } State;

int alarmEnabled = FALSE;
int alarmCount = 0;
int timeout_g = 0;
int nRetransmissions_g = 0;
int tx_s = 0;
int rx_r = 0;

// Estatisticas
static int stat_framesSent = 0, stat_retrans = 0, stat_rej = 0, stat_timeouts = 0;
static int stat_framesOk = 0, stat_dup = 0, stat_bad = 0;

void alarmHandler(int signal) {
    (void)signal;
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarme #%d recebido\n", alarmCount);
}

void sendSupervisionFrame(unsigned char a, unsigned char c) {
    unsigned char frame[5] = {FLAG, a, c, a ^ c, FLAG};
    writeBytesSerialPort(frame, 5);
}

int receiveSupervisionFrame(unsigned char expectedA, unsigned char expectedC, int useAlarm) {
    State state = START;
    unsigned char byte, a = 0, c = 0;
    while (state != STOP_STATE) {
        if (useAlarm && alarmEnabled == FALSE) return -1;
        if (readByteSerialPort(&byte) > 0) {
            switch (state) {
                case START: if (byte == FLAG) state = FLAG_RCV; break;
                case FLAG_RCV:
                    if (byte == FLAG) state = FLAG_RCV;
                    else if (byte == expectedA) { a = byte; state = A_RCV; }
                    else state = START;
                    break;
                case A_RCV:
                    if (byte == FLAG) state = FLAG_RCV;
                    else if (byte == expectedC) { c = byte; state = C_RCV; }
                    else state = START;
                    break;
                case C_RCV:
                    if (byte == FLAG) state = FLAG_RCV;
                    else if (byte == (a ^ c)) state = BCC_OK;
                    else state = START;
                    break;
                case BCC_OK:
                    if (byte == FLAG) state = STOP_STATE;
                    else state = START;
                    break;
                default: break;
            }
        }
    }
    return 0;
}

int llOpenTx(LinkLayer llParameters) {
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0) return -1;
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    sigaction(SIGALRM, &act, NULL);

    timeout_g = llParameters.timeout;
    nRetransmissions_g = llParameters.nRetransmissions;
    tx_s = 0;
    
    alarmCount = 0;
    int uaReceived = FALSE;
    while (alarmCount < nRetransmissions_g && !uaReceived) {
        sendSupervisionFrame(A_CMD, C_SET);
        alarmEnabled = TRUE;
        alarm(timeout_g);
        if (receiveSupervisionFrame(A_CMD, C_UA, 1) == 0) {
            uaReceived = TRUE;
            alarm(0);
        }
    }
    if (!uaReceived) { closeSerialPort(); return -1; }
    return 0;
}

int llOpenRx(LinkLayer llParameters) {
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0) return -1;
    rx_r = 0;
    if (receiveSupervisionFrame(A_CMD, C_SET, 0) == 0) {
        sendSupervisionFrame(A_CMD, C_UA);
        return 0;
    }
    closeSerialPort();
    return -1;
}

int llSend(const unsigned char *buf, int bufSize) {
    unsigned char bcc2 = 0;
    for (int i = 0; i < bufSize; i++) bcc2 ^= buf[i];

    unsigned char* frame = malloc(6 + (bufSize + 1) * 2);
    frame[0] = FLAG;
    frame[1] = A_CMD;
    frame[2] = (tx_s == 0) ? C_I0 : C_I1;
    frame[3] = frame[1] ^ frame[2];

    int j = 4;
    for (int i = 0; i < bufSize; i++) {
        if (buf[i] == FLAG) { frame[j++] = ESC; frame[j++] = 0x5E; }
        else if (buf[i] == ESC) { frame[j++] = ESC; frame[j++] = 0x5D; }
        else { frame[j++] = buf[i]; }
    }
    
    if (bcc2 == FLAG) { frame[j++] = ESC; frame[j++] = 0x5E; }
    else if (bcc2 == ESC) { frame[j++] = ESC; frame[j++] = 0x5D; }
    else { frame[j++] = bcc2; }
    frame[j++] = FLAG;

    int accepted = FALSE;
    int attempts = 0;      // tentativas sem resposta util (timeouts)
    int rejs = 0;          // REJ consecutivos (limite proprio, bem mais alto)
    const int MAX_REJ = 30;
    int first = TRUE;
    alarmCount = 0;
    while (attempts < nRetransmissions_g && rejs < MAX_REJ && !accepted) {
        if (!first) stat_retrans++;
        first = FALSE;
        stat_framesSent++;
        writeBytesSerialPort(frame, j);
        alarmEnabled = TRUE;
        alarm(timeout_g);

        State state = START;
        unsigned char byte, c_rcv = 0;
        while (state != STOP_STATE && alarmEnabled) {
            if (readByteSerialPort(&byte) > 0) {
                switch (state) {
                    case START: if (byte == FLAG) state = FLAG_RCV; break;
                    case FLAG_RCV:
                        if (byte == FLAG) state = FLAG_RCV;
                        else if (byte == A_CMD) state = A_RCV;
                        else state = START;
                        break;
                    case A_RCV:
                        if (byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1) { c_rcv = byte; state = C_RCV; }
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (A_CMD ^ c_rcv)) state = BCC_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC_OK:
                        if (byte == FLAG) state = STOP_STATE;
                        else state = START;
                        break;
                    default: break;
                }
            }
        }
        alarm(0);
        if (state == STOP_STATE) {
            if ((c_rcv == C_RR0 && tx_s == 1) || (c_rcv == C_RR1 && tx_s == 0)) {
                accepted = TRUE;
                tx_s = (tx_s + 1) % 2;
            } else if (c_rcv == C_REJ0 || c_rcv == C_REJ1) {
                stat_rej++;
                rejs++;            // REJ: retransmite logo, sem gastar tentativas de timeout
                continue;
            }
            attempts++;            // RR antigo/duplicado
            // RR "antigo" (duplicado): ignora e tenta de novo
        } else {
            stat_timeouts++;
            attempts++;
        }
    }
    free(frame);
    return accepted ? j : -1;
}

int llReceive(unsigned char *packet) {
    int state = 0;
    unsigned char byte, control_field = 0;
    int j = 0, overflow = FALSE;
    const int CAP = MAX_PAYLOAD_SIZE * 2 + 10;
    unsigned char* destuffed = malloc(CAP);
    if (!destuffed) return -1;

    while (1) {
        // --- le uma trama completa (ate ao FLAG final) ---
        state = 0; j = 0; overflow = FALSE;
        while (state != 5) {
            if (readByteSerialPort(&byte) > 0) {
                switch(state) {
                    case 0: if (byte == FLAG) state = 1; break;
                    case 1:
                        if (byte == A_CMD) state = 2;
                        else if (byte == FLAG) state = 1;
                        else state = 0;
                        break;
                    case 2:
                        if (byte == C_I0 || byte == C_I1 || byte == C_SET) { control_field = byte; state = 3; }
                        else if (byte == FLAG) state = 1;
                        else state = 0;
                        break;
                    case 3:
                        if (byte == (A_CMD ^ control_field)) { state = 4; j = 0; overflow = FALSE; }
                        else if (byte == FLAG) state = 1;
                        else state = 0;
                        break;
                    case 4:
                        if (byte == FLAG) state = 5;
                        else if (j < CAP) destuffed[j++] = byte;
                        else overflow = TRUE;   // lixo/ruido: evita buffer overflow
                        break;
                }
            }
        }

        // SET repetido (o UA anterior perdeu-se): responde outra vez
        if (control_field == C_SET) {
            sendSupervisionFrame(A_CMD, C_UA);
            continue;
        }

        // --- destuffing ---
        int k = 0;
        for (int i = 0; i < j; i++) {
            if (destuffed[i] == ESC && i + 1 < j) {
                i++;
                if (destuffed[i] == 0x5E) destuffed[k++] = FLAG;
                else if (destuffed[i] == 0x5D) destuffed[k++] = ESC;
                else { overflow = TRUE; }       // sequencia de escape invalida
            } else destuffed[k++] = destuffed[i];
        }

        int seq_rcv = (control_field == C_I1) ? 1 : 0;
        int dup = (seq_rcv != rx_r);

        int bcc_ok = FALSE;
        if (!overflow && k >= 1) {
            unsigned char calc = 0;
            for (int i = 0; i < k - 1; i++) calc ^= destuffed[i];
            bcc_ok = (calc == destuffed[k - 1]);
        }

        if (bcc_ok) {
            if (!dup) {
                rx_r = (rx_r + 1) % 2;
                stat_framesOk++;
                memcpy(packet, destuffed, k - 1);
                sendSupervisionFrame(A_CMD, (rx_r == 1) ? C_RR1 : C_RR0);
                free(destuffed);
                return k - 1;
            }
            // duplicado valido: confirma de novo e descarta
            stat_dup++;
            sendSupervisionFrame(A_CMD, (rx_r == 1) ? C_RR1 : C_RR0);
            continue;
        }

        stat_bad++;
        if (dup) {
            // trama ja recebida mas corrompida: so reenviar RR (nao pedir retransmissao)
            sendSupervisionFrame(A_CMD, (rx_r == 1) ? C_RR1 : C_RR0);
        } else {
            sendSupervisionFrame(A_CMD, (rx_r == 1) ? C_REJ1 : C_REJ0);
        }
    }
}

static void printStats(const char *who) {
    printf("\n--- Estatisticas (%s) ---\n", who);
    if (strcmp(who, "Tx") == 0)
        printf("Tramas I enviadas: %d | retransmissoes: %d | REJ recebidos: %d | timeouts: %d\n",
               stat_framesSent, stat_retrans, stat_rej, stat_timeouts);
    else
        printf("Tramas I aceites: %d | duplicadas: %d | com erros: %d\n",
               stat_framesOk, stat_dup, stat_bad);
}

int llCloseTx() {
    alarmCount = 0;
    int discReceived = FALSE;
    while (alarmCount < nRetransmissions_g && !discReceived) {
        sendSupervisionFrame(A_CMD, C_DISC);
        alarmEnabled = TRUE;
        alarm(timeout_g);
        if (receiveSupervisionFrame(A_RSP, C_DISC, 1) == 0) {
            discReceived = TRUE;
            alarm(0);
        }
    }
    if (discReceived) sendSupervisionFrame(A_RSP, C_UA);
    printStats("Tx");
    return closeSerialPort();
}

int llCloseRx() {
    if (receiveSupervisionFrame(A_CMD, C_DISC, 0) == 0) {
        sendSupervisionFrame(A_RSP, C_DISC);
        receiveSupervisionFrame(A_RSP, C_UA, 0);
    }
    printStats("Rx");
    return closeSerialPort();
}