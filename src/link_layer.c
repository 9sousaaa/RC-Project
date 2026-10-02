// RCOM 2026/2027
// Link layer protocol implementation

#define _DEFAULT_SOURCE 1
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "link_layer.h"
#include "serial_port.h"

// Definição das constantes do protocolo de ligação de dados
#define FLAG 0x7E
#define A_TX 0x03
#define A_RX 0x01
#define C_SET 0x03
#define C_UA 0x07

// Estados da máquina de estados para receção de tramas de supervisão
typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_STATE
} State;

// Variáveis globais para gestão do alarme e retransmissões
int alarmEnabled = FALSE;
int alarmCount = 0;

// Função auxiliar para envio de tramas através da porta série
void sendFrameTx(unsigned char *frame, int size)
{
    writeBytesSerialPort(frame, size);
}

// Função tratadora do sinal de alarme (SIGALRM)
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarme #%d recebido\n", alarmCount);
}

// Máquina de estados para processamento de tramas de supervisão (SET / UA)
int receiveSupervisionFrame(unsigned char expectedA, unsigned char expectedC, int useAlarm)
{
    State state = START;
    unsigned char byte;
    unsigned char a = 0, c = 0;

    while (state != STOP_STATE)
    {
        if (useAlarm && alarmEnabled == FALSE)
        {
            return -1;
        }

        int res = readByteSerialPort(&byte);
        if (res <= 0)
        {
            if (useAlarm && alarmEnabled == FALSE)
            {
                return -1;
            }
            continue;
        }

        switch (state)
        {
            case START:
                if (byte == FLAG)
                    state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if (byte == FLAG)
                {
                    state = FLAG_RCV;
                }
                else if (byte == expectedA)
                {
                    a = byte;
                    state = A_RCV;
                }
                else
                {
                    state = START;
                }
                break;

            case A_RCV:
                if (byte == FLAG)
                {
                    state = FLAG_RCV;
                }
                else if (byte == expectedC)
                {
                    c = byte;
                    state = C_RCV;
                }
                else
                {
                    state = START;
                }
                break;

            case C_RCV:
                if (byte == FLAG)
                {
                    state = FLAG_RCV;
                }
                else if (byte == (a ^ c))
                {
                    state = BCC_OK;
                }
                else
                {
                    state = START;
                }
                break;

            case BCC_OK:
                if (byte == FLAG)
                {
                    state = STOP_STATE;
                }
                else
                {
                    state = START;
                }
                break;

            default:
                break;
        }
    }

    if (state == STOP_STATE)
        return 0;

    return -1;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llOpenTx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Porta de serie %s aberta com sucesso (Tx)\n", llParameters.serialPort);

    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;

    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        closeSerialPort();
        return -1;
    }

    unsigned char setFrame[5] = {FLAG, A_TX, C_SET, A_TX ^ C_SET, FLAG};

    alarmCount = 0;
    int uaReceived = FALSE;

    while (alarmCount < llParameters.nRetransmissions && !uaReceived)
    {
        printf("A enviar trama SET (tentativa %d de %d)...\n", alarmCount + 1, llParameters.nRetransmissions);
        sendFrameTx(setFrame, 5);

        alarmEnabled = TRUE;
        alarm(llParameters.timeout);

        if (receiveSupervisionFrame(A_TX, C_UA, 1) == 0)
        {
            uaReceived = TRUE;
            alarm(0);
            printf("Trama UA recebida com sucesso! Ligacao estabelecida.\n");
        }
    }

    if (!uaReceived)
    {
        printf("Erro: Nao foi possivel receber a trama UA apos %d tentativas.\n", llParameters.nRetransmissions);
        closeSerialPort();
        return -1;
    }

    return 0;
}

int llOpenRx(LinkLayer llParameters)
{
    if (openSerialPort(llParameters.serialPort, llParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }

    printf("Porta de serie %s aberta com sucesso (Rx)\n", llParameters.serialPort);

    printf("A aguardar pela rececao da trama SET...\n");
    if (receiveSupervisionFrame(A_TX, C_SET, 0) == 0)
    {
        printf("Trama SET recebida e validada com sucesso!\n");

        unsigned char uaFrame[5] = {FLAG, A_TX, C_UA, A_TX ^ C_UA, FLAG};
        sendFrameTx(uaFrame, 5);
        printf("Trama UA enviada com sucesso! Ligacao estabelecida.\n");

        return 0;
    }

    closeSerialPort();
    return -1;
}

////////////////////////////////////////////////
// LLSEND
////////////////////////////////////////////////
int llSend(const unsigned char *buf, int bufSize)
{
    return 0;
}

////////////////////////////////////////////////
// LLRECEIVE
////////////////////////////////////////////////
int llReceive(unsigned char *packet)
{
    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llCloseTx()
{
    return closeSerialPort();
}

int llCloseRx()
{
    return closeSerialPort();
}