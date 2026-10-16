// RCOM 2026/2027
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <string.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer linkLayer;
    strncpy(linkLayer.serialPort, serialPort, sizeof(linkLayer.serialPort) - 1);
    linkLayer.serialPort[sizeof(linkLayer.serialPort) - 1] = '\0';
    linkLayer.baudRate = baudRate;
    linkLayer.nRetransmissions = nTries;
    linkLayer.timeout = timeout;

    printf("A iniciar o estabelecimento da ligacao...\n");

    if (strcmp(role, "tx") == 0)
    {
        if (llOpenTx(linkLayer) < 0)
        {
            printf("Erro: Nao foi possivel estabelecer a ligacao (Tx).\n");
            return;
        }
        printf("Ligacao estabelecida com sucesso (Tx)!\n");
        llCloseTx();
    }
    else if (strcmp(role, "rx") == 0)
    {
        if (llOpenRx(linkLayer) < 0)
        {
            printf("Erro: Nao foi possivel estabelecer a ligacao (Rx).\n");
            return;
        }
        printf("Ligacao estabelecida com sucesso (Rx)!\n");
        llCloseRx();
    }
    else
    {
        printf("Erro: Papel desconhecido (%s)\n", role);
    }
}